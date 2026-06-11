import os
import time
import logging
import argparse
import subprocess
import shutil
import json
import traceback
from collections import deque
from datetime import datetime
from typing import Optional
from mpi4py import MPI
from enum import Enum, auto

from partition_tree import ParallelNode, ParallelTree
from partition_tree import NodeStatus, NodeReason
from partition_tree import terminate_subprocess
from control_message import TerminateMessage, ControlMessage, MemoryOutMessage

def raise_error(error_info):
    logging.error(error_info)
    raise Exception(error_info)

class CoordinatorStatus(Enum):
    idle = auto()
    waiting = auto()
    solving = auto()
    splitting = auto()
    
    def is_idle(self):
        return self == CoordinatorStatus.idle

    def is_waiting(self):
        return self == CoordinatorStatus.waiting
    
    def is_solving(self):
        return self == CoordinatorStatus.solving
    
    def is_splitting(self):
        return self == CoordinatorStatus.splitting

class Coordinator:
    def __init__(self):
        self.solving_round = 0
        # terminate on demand
        # self.terminate_threshold = [1200.0, 200.0, 100.0]
        self.terminate_threshold = [1200.0, 400.0, 300.0, 200.0, 0.0]
        
        self.coordinator_start_time = time.time()
        self.rank = MPI.COMM_WORLD.Get_rank()
        self.leader_rank = MPI.COMM_WORLD.Get_size() - 1
        self.isolated_rank = self.leader_rank - 1
        self.num_dist_coords = self.isolated_rank
        self.init_params()
        
        if self.get_model_flag:
            self.model = ''
            self.get_model_done = False
        
        self.coord_temp_folder_path = f'{self.temp_dir}/Coordinator-{self.rank}'
        # assign a core to coordinator and partitioner
        # self.available_cores -= 1
        
        self.init_logging()
        os.makedirs(self.coord_temp_folder_path, exist_ok=True)
        
        self.raise_fd_soft_limit()
        self.init_memory_limits()
        self.memory_out_reported = False
        
        self.status: CoordinatorStatus = CoordinatorStatus.idle
        self.result = NodeStatus.unsolved
        
        self.original_process = None
        self.tree = None
        
        logging.debug(f'rank: {self.rank}, leader_rank: {self.leader_rank}')
        logging.debug(f'get-model-flag: {self.get_model_flag}')
        logging.debug(f'temp_folder_path: {self.coord_temp_folder_path}')
        logging.debug(f'coordinator-{self.rank} init done!')
    
    def init_logging(self):
        log_dir_path = f'{self.output_folder_path}/logs'
        os.makedirs(log_dir_path, exist_ok=True)
        if self.rank == self.isolated_rank:
            log_file_path = f'{log_dir_path}/coordinator-isolated.log'
        else:
            log_file_path = f'{log_dir_path}/coordinator-{self.rank}.log'
        logging.basicConfig(format='%(relativeCreated)d - %(levelname)s - %(message)s', 
                filename=log_file_path, level=logging.DEBUG)
        current_time = datetime.now()
        formatted_time = current_time.strftime("%Y-%m-%d %H:%M:%S")
        logging.info(f'start-time {formatted_time} ({self.coordinator_start_time})')
    
    def init_params(self):
        arg_parser = argparse.ArgumentParser()
        common_args = arg_parser.add_argument_group('Common Arguments')
        common_args.add_argument('--temp-dir', type=str, required=True,
                                help='temp dir path')
        common_args.add_argument('--output-dir', type=str, required=True,
                                help='output dir path')
        common_args.add_argument('--get-model-flag', type=int, required=True,
                                help='get model flag')
        common_args.add_argument('--max-worker-memory-mb', type=int, default=0,
                                help='memory limit per worker in MB (0 means unlimited)')
        common_args.add_argument('--debug', action='store_true',
                                help='enable debug mode (keep temp artifacts)')
        
        leader_args = arg_parser.add_argument_group('Leader Arguments')
        leader_args.add_argument('--file', type=str, required=True,
                        help='input instance file path')
        leader_args.add_argument('--time-limit', type=int, default=0,
                                help='time limit, 0 means no limit')
        
        coordinator_args = arg_parser.add_argument_group('Coordinator Arguments')
        coordinator_args.add_argument('--solver', type=str, required=True,
                                help="solver path")
        coordinator_args.add_argument('--partitioner', type=str, required=True,
                                help='partitioner path')
        coordinator_args.add_argument('--available-cores-list', type=str, required=True, 
                                help='available cores list')
        
        cmd_args = arg_parser.parse_args()
        self.output_folder_path: str = cmd_args.output_dir
        self.temp_dir: str = cmd_args.temp_dir
        self.get_model_flag: bool = int(cmd_args.get_model_flag)
        self.max_worker_memory_mb: int = int(cmd_args.max_worker_memory_mb)
        self.debug: bool = bool(cmd_args.debug)
        
        self.time_limit: float = float(cmd_args.time_limit)
        
        self.solver_path: str = cmd_args.solver
        self.partitioner_path: str = cmd_args.partitioner
        available_cores_list: list = json.loads(cmd_args.available_cores_list)
        
        self.available_cores: int = available_cores_list[self.rank]
        self.total_available_cores: int = sum(available_cores_list)

    def is_done(self):
        if self.result.is_solved():
            return True
        if self.tree.is_done():
            self.result = self.tree.get_result()
            return True
        return False
    
    def get_result(self):
        return self.result
    
    def get_coordinator_time(self):
        return time.time() - self.coordinator_start_time
    
    def get_solving_time(self):
        return time.time() - self.solving_start_time
    
    def write_line_to_log(self, data: str):
        logging.info(data)

    def raise_fd_soft_limit(self, target: int = 1048576):
        try:
            import resource  # type: ignore
        except ImportError:
            logging.warning('resource module unavailable; skip raising RLIMIT_NOFILE')
            return
        try:
            soft, hard = resource.getrlimit(resource.RLIMIT_NOFILE)
            desired_soft = target
            if hard != resource.RLIM_INFINITY:
                desired_soft = min(target, hard)
            if soft >= desired_soft:
                logging.debug(
                    f'RLIMIT_NOFILE already sufficient (soft={soft}, hard={hard})'
                )
                return
            resource.setrlimit(resource.RLIMIT_NOFILE, (desired_soft, hard))
            logging.info(
                f'raised RLIMIT_NOFILE soft limit from {soft} to {desired_soft} '
                f'(hard={hard})'
            )
        except (PermissionError, ValueError) as exc:
            logging.warning(
                f'failed to raise RLIMIT_NOFILE soft limit to {target}: {exc}'
            )
        except Exception as exc:
            logging.warning(f'unable to adjust RLIMIT_NOFILE: {exc}')

    def init_memory_limits(self):
        self.per_task_memory_mb = max(0, self.max_worker_memory_mb)
        self.preexec_fn = None
        if self.per_task_memory_mb <= 0:
            return
        try:
            import resource  # type: ignore
        except ImportError:
            logging.warning('resource module unavailable; memory limit disabled')
            self.per_task_memory_mb = 0
            return
        limit_bytes = self.per_task_memory_mb * 1024 * 1024
        def _limit():
            try:
                resource.setrlimit(resource.RLIMIT_AS, (limit_bytes, limit_bytes))
            except Exception:
                pass
        self.preexec_fn = _limit
        logging.info(
            f'memory limit enabled: per_worker={self.per_task_memory_mb} MB'
        )

    def is_memory_out(self, rc: int, out_data: str, err_data: str) -> bool:
        if rc is None:
            return False
        text = f'{out_data}\n{err_data}'.lower()
        keywords = (
            'out of memory',
            'memory limit',
            'cannot allocate memory',
            'std::bad_alloc',
            'bad_alloc',
            'oom',
        )
        if rc != 0 and any(key in text for key in keywords):
            return True
        if rc < 0 and -rc in (6, 9, 11):
            return True
        if rc in (134, 137, 139):
            return True
        return False

    def report_memory_out(self, where: str, rc: int, out_data: str, err_data: str):
        if getattr(self, 'memory_out_reported', False):
            raise MemoryOutMessage()
        self.memory_out_reported = True
        detail = (
            f'where={where}, rc={rc}, '
            f'stderr={err_data[-400:]}, stdout={out_data[-400:]}'
        )
        logging.error(f'memory out detected: {detail}')
        MPI.COMM_WORLD.send(ControlMessage.C2L.notify_memory_out,
                            dest=self.leader_rank, tag=1)
        MPI.COMM_WORLD.send(detail, dest=self.leader_rank, tag=2)
        raise MemoryOutMessage()
    
    def check_partitioner_status(self, p: subprocess.Popen):
        rc = p.poll()
        if rc == None:
            return NodeStatus.simplifying

        out_data, err_data = p.communicate()
        
        logging.debug(f'stdout: {out_data}')
        logging.debug(f'stderr: {err_data}')

        if self.is_memory_out(rc, out_data, err_data):
            self.report_memory_out('partitioner', rc, out_data, err_data)
        
        if rc == 233:
            logging.debug('partitioner split failed')
            return NodeStatus.partition_failed
        
        if rc != 0:
            logging.error('partitioner error')
            logging.error(f'stderr: {err_data}')
            return NodeStatus.simplify_failed
        
        assert(rc == 0)
        last_line = ''
        for line in reversed(out_data.splitlines()):
            stripped = line.strip()
            if stripped:
                last_line = stripped
                break
        if last_line == 'sat':
            return NodeStatus.sat
        if last_line == 'unsat':
            return NodeStatus.unsat
        return NodeStatus.simplified
    
    def check_base_solver_status(self, p: subprocess.Popen):
        rc = p.poll()
        if rc == None:
            return NodeStatus.solving

        out_data, err_data = p.communicate()
        if self.is_memory_out(rc, out_data, err_data):
            self.report_memory_out('solver', rc, out_data, err_data)

        if rc != 0:
            logging.error('subprocess error')
            logging.error(f'return code = {rc}')
            logging.error(f'stdout: {out_data}')
            logging.error(f'stderr: {err_data}')
            return NodeStatus.error
            # raise_error(f'return code = {rc}\nstdout: {out_data}\nstderr: {err_data}')

        assert(rc == 0)
        
        if not self.get_model_flag:
            sta: str = out_data.strip('\n').strip(' ')
        else:
            lines = out_data.split('\n')
            sta: str = lines[0]
            if sta == 'sat':
                self.model: str = '\n'.join(lines[1: ])
                self.get_model_done = True
        
        if sta == 'sat':
            return NodeStatus.sat
        elif sta == 'unsat':
            return NodeStatus.unsat
        else:
            logging.error('subprocess error')
            logging.error(f'return code = {rc}')
            logging.error(f'stdout: {out_data}')
            logging.error(f'stderr: {err_data}')
            return NodeStatus.error
            # raise_error(f'subprocess error state: {sta}')
    
    def need_terminate(self, node: ParallelNode):
        return False
        if node.id == 0:
            return False
        solving_time = self.tree.get_node_solving_time(node)
        assert(solving_time is not None)
        remained_time = self.time_limit - self.get_coordinator_time()
        if remained_time < solving_time:
            return False
        num_children = len(node.children)
        # num_start = 0
        child_progress = 0
        if num_children > 0:
            # lc: ParallelNode = node.children[0]
            # if not lc.status.is_unsolved():
            #     num_start += 1
            lc_sta: NodeStatus = node.children[0].status
            if lc_sta.is_solved():
                child_progress += 2
            elif lc_sta.is_solving() or lc_sta.is_terminated():
                child_progress += 1
            if num_children > 1:
                rc_sta: NodeStatus = node.children[1].status
                if rc_sta.is_solved():
                    child_progress += 2
                elif rc_sta.is_solving() or rc_sta.is_terminated():
                    child_progress += 1
        assert(child_progress < 4)
        if child_progress == 0:
            return False
        return solving_time > self.terminate_threshold[child_progress]
    
    def terminate_node(self, node: ParallelNode):
        if node.status.is_ended():
            return
        logging.info(f'terminate node-{node.id}')
        self.tree.terminate_node(node, NodeReason.coordinator)
    
    # True for still simplifying
    def check_simplifying_status(self, node: ParallelNode):
        # logging.debug(f'check_simplifying_status: node-{node.id}')
        if not node.status.is_simplifying():
            return False
        sta: NodeStatus = self.check_partitioner_status(node.assign_to)
        if sta.is_simplifying():
            return True
        node.assign_to = None
        if sta.is_solved():
            logging.info(f'solved-by-partitioner: node-{node.id} is {sta}')
            self.tree.node_solved(node, sta, NodeReason.partitioner)
            self.log_tree_infos()
            return False
        if not os.path.exists(f'{self.solving_folder_path}/task-{node.id}-simplified.smt2'):
            logging.error('partitioner missing simplified output')
            sta = NodeStatus.simplify_failed
        elif sta.is_simplified() and len(node.children) >= 2:
            left_id = node.children[0].id
            right_id = node.children[1].id
            left_path = f'{self.solving_folder_path}/task-{left_id}.smt2'
            right_path = f'{self.solving_folder_path}/task-{right_id}.smt2'
            if not (os.path.exists(left_path) and os.path.exists(right_path)):
                logging.debug('partitioner did not emit child tasks')
                sta = NodeStatus.partition_failed
        logging.info(f'simplified: node-{node.id} is {sta}')
        if sta.is_simplified():
            self.tree.node_simplified(node, sta)
            self.log_tree_infos()
        return False
    
    # True for still solving
    def check_solving_status(self, node: ParallelNode):
        # logging.debug(f'check_solving_status: node-{node.id}')
        if not node.status.is_solving():
            return False
        sta: NodeStatus = self.check_base_solver_status(node.assign_to)
        if sta.is_error():
            self.terminate_node(node)
            return False
        if sta.is_solving():
            if not self.need_terminate(node):
                return True
            logging.info(f'terminate on demand')
            self.terminate_node(node)
            return False
        node.assign_to = None
        logging.info(f'solved: node-{node.id} is {sta}')
        self.tree.node_solved(node, sta)
        self.log_tree_infos()
        return False
    
    def solve_task(self, task_tag: str):
        instance_path = f'{self.solving_folder_path}/task-{task_tag}.smt2'
        cmd =  [self.solver_path,
                instance_path,
            ]
        logging.debug('exec-command {}'.format(' '.join(cmd)))
        p = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                preexec_fn=self.preexec_fn
            )
        return p

    def solve_simplified_task(self, node: ParallelNode):
        p = self.solve_task(f'{node.id}-simplified')
        self.tree.solve_node(node, p)
        logging.debug(f'solve-node {node.id}')
    
    def check_runnings_status(self, runnings: list, running_status: NodeStatus, check_func):
        # logging.debug(f'check_runnings_status: {len(runnings)}')
        still_runnings = []
        for node in runnings:
            node: ParallelNode
            if node.status != running_status:
                continue
            if check_func(node):
                still_runnings.append(node)
            else:
                if self.is_done():
                    return True
        runnings[:] = still_runnings
        return False
    
    def check_simplifyings_status(self):
        return self.check_runnings_status(
                    self.tree.simplifyings,
                    NodeStatus.simplifying,
                    self.check_simplifying_status
                )
    
    def check_solvings_status(self):
        return self.check_runnings_status(
                    self.tree.solvings,
                    NodeStatus.solving,
                    self.check_solving_status
                )
    
    def log_tree_infos(self):
        logging.debug(
            f'nodes: {self.tree.get_node_number()}, '
            f'runnings: {self.tree.get_running_number()}({self.available_cores}), '
            f'simplifyings: {self.tree.get_simplifying_number()}, '
            f'solvings: {self.tree.get_solving_number()}'
        )
        logging.debug(
            f'unsolved: {self.tree.get_unsolved_number()}({self.available_cores}), '
            f'simplified: {self.tree.get_simplified_number()}'
        )
        logging.debug(
            f'solved: {self.tree.update_dict.get((NodeStatus.unsat, NodeReason.itself), 0)}(itself), '
            f'{self.tree.update_dict.get((NodeStatus.unsat, NodeReason.children), 0)}(children), '
            f'{self.tree.update_dict.get((NodeStatus.unsat, NodeReason.ancester), 0)}(ancester), '
            f'{self.tree.update_dict.get((NodeStatus.unsat, NodeReason.partitioner), 0)}(partitioner), '
            f'progress: {self.tree.root.unsat_percent * 100.0 :.2f}%'
            # f'endeds: {self.tree.get_ended_number()}, '
            # f'unendeds: {self.tree.get_unended_number()}'
        )
    
    def simplify_partition_task(self, node: ParallelNode):
        left_child = self.tree.make_node(node.id)
        right_child = self.tree.make_node(node.id)
        assert(left_child.id + 1 == right_child.id)
        cmd =  [self.partitioner_path,
                f'{self.solving_folder_path}/task-{node.id}.smt2',
                '--pp-fdp-dump-prefix', f'{self.solving_folder_path}',
                '--pp-fdp-node-id', f'{node.id}',
                '--pp-fdp-parti-seed', f'{self.rank}',
                '--pp-fdp-next-id', f'{left_child.id}',
            ]
        # if self.per_task_memory_mb > 0:
        #     cmd.extend(['--memory-limit', str(self.per_task_memory_mb)])
        logging.debug(f'exec-command {" ".join(cmd)}')
        p = subprocess.Popen(
                cmd,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                preexec_fn=self.preexec_fn
            )
        self.tree.simplify_partition_node(node, p)
    
    def simplify_partition_tasks(self):
        while self.tree.get_running_number() < self.available_cores \
          and self.tree.get_simplify_total_number() + self.tree.get_solving_number() < self.available_cores:
            node = self.tree.get_next_unsolved_node()
            if node == None:
                break
            self.simplify_partition_task(node)
    
    def pre_partition_tasks(self):
        while self.tree.get_running_number() < self.available_cores:
            node = self.tree.get_next_unsolved_node()
            if node == None:
                break
            self.simplify_partition_task(node)
    
    # run simplifieds by:
    # currently: generation order
    # can be easily change to: priority select
    def solve_simplified_tasks(self):
        while self.tree.get_running_number() < self.available_cores:
            node = self.tree.get_next_simplified_node()
            if node == None:
                break
            self.solve_simplified_task(node)
    
    def start_solving(self):
        self.solving_folder_path = f'{self.coord_temp_folder_path}/tasks/round-{self.solving_round}'
        
        self.status = CoordinatorStatus.solving
        self.result = NodeStatus.unsolved
        self.solving_start_time = time.time()
        self.tree = ParallelTree(
            self.solving_start_time,
            self.solving_folder_path,
            preserve_task_files=self.debug
        )
        
        root = self.tree.make_node(-1)
        self.tree.unsolveds.append(root)

    # coordinator [rank] solved the assigned node
    def send_result_to_leader(self):
        result: NodeStatus = self.get_result()
        assert(result.is_solved())
        model = None
        if self.get_model_flag and result.is_sat():
            assert(self.get_model_done)
            model = self.model
        MPI.COMM_WORLD.send(ControlMessage.C2L.notify_result,
                            dest=self.leader_rank, tag=1)
        MPI.COMM_WORLD.send((result, model), dest=self.leader_rank, tag=2)
    
    # True -> solved
    def parallel_solving(self):
        if self.check_simplifyings_status():
            # self.tree_log_display()
            return True
        if self.check_solvings_status():
            # self.tree_log_display()
            return True
        self.simplify_partition_tasks()
        self.solve_simplified_tasks()
        return False
    
    def select_split_node(self):
        return self.tree.select_split_node()

    def set_node_split(self, node: ParallelNode, assigned_coord: int):
        path_node = node
        while path_node is not None:
            self.terminate_node(path_node)
            path_node = path_node.parent
        self.tree.set_node_split(node, assigned_coord)
    
    def solve_leader_root(self):
        msg_type = MPI.COMM_WORLD.recv(source=self.leader_rank, tag=1)
        assert(isinstance(msg_type, ControlMessage.L2C))
        assert(msg_type.is_assign_node())
        self.start_solving()
    
    def pre_partition(self):
        logging.debug(f'pre-partition start')
        subnodes = [self.tree.root]
        while True:
            # self.receive_message_from_leader()
            if self.check_original_task():
                return True
            if self.check_simplifyings_status():
                # self.tree_log_display()
                return True
            self.pre_partition_tasks()
            if self.is_done():
                return True
            cur_subnodes = []
            for node in subnodes:
                if node.status.is_solved():
                    continue
                if node.status.is_unsolved() or node.status.is_simplifying():
                    cur_subnodes.append(node)
                else:
                    for child in node.children:
                        cur_subnodes.append(child)
                if len(cur_subnodes) >= self.num_dist_coords:
                    break
            subnodes = cur_subnodes
            if len(subnodes) >= self.num_dist_coords:
                break
            if self.get_coordinator_time() > 20.0:
                break
            time.sleep(0.1)
        
        pp_num_nodes = len(subnodes)
        logging.debug(f'pre-partition split {pp_num_nodes} node(s)')
        # assert(pp_num_nodes > 0)
        MPI.COMM_WORLD.send(ControlMessage.C2L.pre_partition_done,
                            dest=self.leader_rank, tag=1)
        MPI.COMM_WORLD.send(pp_num_nodes, 
                            dest=self.leader_rank, tag=2)
        msg_type: ControlMessage.L2C = MPI.COMM_WORLD.recv(source=self.leader_rank, tag=1)
        # assert(isinstance(msg_type, ControlMessage.L2C))
        if msg_type.is_assign_node():
            if pp_num_nodes > 0:
                for i in range(pp_num_nodes):
                    node: ParallelNode = subnodes[i]
                    self.copy_split_node_file(node.id, i)
                    self.send_split_node_to_coordinator(i)
            else:
                logging.debug(f'split root-task to coordinater-0')
                self.send_root_task_to_coordinator(0)
            return False
        elif msg_type.is_terminate_coordinator():
            raise TerminateMessage()
        else:
            assert(False)
    
    def receive_node_from_coordinator(self, coord_rank):
        solving_folder_path = f'{self.coord_temp_folder_path}/tasks/round-{self.solving_round}'
        os.makedirs(solving_folder_path, exist_ok=True)
        instance_path = f'{solving_folder_path}/task-0.smt2'
        instance_data = MPI.COMM_WORLD.recv(source=coord_rank, tag=2)
        with open(instance_path, 'bw') as file:
            file.write(instance_data)
    
    def process_assign_message(self):
        coord_rank = MPI.COMM_WORLD.recv(source=self.leader_rank, tag=2)
        logging.debug(f'receive instance from coordinator-{coord_rank}')
        self.receive_node_from_coordinator(coord_rank)
        self.start_solving()
    
    def send_split_succeed_to_leader(self, target_rank):
        MPI.COMM_WORLD.send(ControlMessage.C2L.split_succeed,
                            dest=self.leader_rank, tag=1)
        MPI.COMM_WORLD.send(target_rank, 
                            dest=self.leader_rank, tag=2)
    
    def send_split_node_to_coordinator(self, target_rank):
        instance_path = f'{self.solving_folder_path}/task-split.smt2'
        logging.debug(f'split task path: {instance_path}')
        with open(instance_path, 'br') as file:
            instance_data = file.read()
        MPI.COMM_WORLD.send(instance_data, 
                            dest=target_rank, tag=2)
        
    def send_root_task_to_coordinator(self, target_rank):
        instance_path = f'{self.solving_folder_path}/task-0.smt2'
        logging.debug(f'split task path: {instance_path}')
        with open(instance_path, 'br') as file:
            instance_data = file.read()
        MPI.COMM_WORLD.send(instance_data, 
                            dest=target_rank, tag=2)
    
    def send_split_failed_to_leader(self, target_rank):
        MPI.COMM_WORLD.send(ControlMessage.C2L.split_failed,
                            dest=self.leader_rank, tag=1)
        MPI.COMM_WORLD.send(target_rank, 
                            dest=self.leader_rank, tag=2)
    
    def copy_split_node_file(self, node_id: int, target_rank: int):
        # copy task node file to task-split.smt2.smt2
        shutil.copyfile(f'{self.solving_folder_path}/task-{node_id}.smt2',
                        f'{self.solving_folder_path}/task-split.smt2')
        logging.debug(f'split node-{node_id} to coordinater-{target_rank}')
    
    def process_split_message(self):
        # split a subnode and assign to coordinator {target_rank}
        target_rank = MPI.COMM_WORLD.recv(source=self.leader_rank, tag=2)
        if self.status.is_idle():
            self.send_split_failed_to_leader(target_rank)
            return
        node = self.select_split_node()
        if node == None:
            self.send_split_failed_to_leader(target_rank)
            return
        logging.debug(f'split target rank: {target_rank}')
        logging.debug(f'split node: {node}')
        self.send_split_succeed_to_leader(target_rank)
        self.copy_split_node_file(node.id, target_rank)
        self.set_node_split(node, target_rank)
    
    def process_transfer_message(self):
        target_rank = MPI.COMM_WORLD.recv(source=self.leader_rank, tag=2)
        logging.debug(f'split node to coordinater-{target_rank}')
        self.send_split_node_to_coordinator(target_rank)
    
    def round_clean_up(self):
        assert(self.tree != None)
        for node_collection in (self.tree.simplifyings, self.tree.solvings):
            for node in node_collection:
                if not isinstance(node, ParallelNode):
                    continue
                if node.assign_to is None:
                    continue
                terminate_subprocess(node.assign_to)
                node.assign_to = None
        
        self.tree = None
        if self.solving_round > 0:
            if not self.debug:
                shutil.rmtree(f'{self.coord_temp_folder_path}/tasks/round-{self.solving_round - 1}')
            else:
                logging.info(
                    f'debug mode: keep round-{self.solving_round - 1} artifacts in '
                    f'{self.coord_temp_folder_path}/tasks'
                )

    def solving_round_done(self):
        self.send_result_to_leader()
        self.round_clean_up()
        self.status = CoordinatorStatus.idle
        self.solving_round += 1
        logging.debug(f'round-{self.solving_round} is done')
    
    # True for terminate
    def receive_message_from_leader(self):
        if MPI.COMM_WORLD.Iprobe(source=self.leader_rank, tag=1):
            msg_type = MPI.COMM_WORLD.recv(source=self.leader_rank, tag=1)
            assert(isinstance(msg_type, ControlMessage.L2C))
            # logging.debug(f'receive {msg_type} message from leader')
            if msg_type.is_request_split():
                # split a subnode
                self.process_split_message()
            elif msg_type.is_transfer_node():
                # transfer the split node to coordinator {target_rank}
                self.process_transfer_message()
            elif msg_type.is_assign_node():
                # solve node from coordinator {rank}
                assert(self.status.is_idle())
                self.process_assign_message()
            elif msg_type.is_terminate_coordinator():
                self.tree_log_display()
                raise TerminateMessage()
            else:
                assert(False)
    
    def interactive_solve(self):
        while True:
            self.receive_message_from_leader()
            if self.status.is_solving():
                if self.parallel_solving():
                    self.solving_round_done()
            # time.sleep(0.01)
    
    def check_original_task(self):
        if self.original_process is None:
            return False
        sta: NodeStatus = self.check_base_solver_status(self.original_process)
        if sta.is_solving():
            return False
        elif sta.is_error():
            self.original_process = None
            return False
        else:
            self.result = sta
            logging.info(f'solved-by-original {sta}')
            return True
    
    # solver original task with base solver
    def solve_original_task(self):
        logging.debug('solve_original_task')
        self.original_process = self.solve_task('0')
    
    def isolated_solve(self):
        self.solve_leader_root()
        if not self.debug:
            self.solve_original_task()

        if self.num_dist_coords > 0:
            if self.pre_partition():
                self.solving_round_done()
        else:
            logging.debug('pre-partition skipped: no distributed coordinators')
        
        while True:
            if MPI.COMM_WORLD.Iprobe(source=self.leader_rank, tag=1):
                msg_type = MPI.COMM_WORLD.recv(source=self.leader_rank, tag=1)
                assert(isinstance(msg_type, ControlMessage.L2C))
                assert(msg_type.is_terminate_coordinator())
                self.tree_log_display()
                raise TerminateMessage()
            
            if self.status.is_solving():
                if self.check_original_task():
                    self.solving_round_done()
                    continue
                
                if self.parallel_solving():
                    self.tree_log_display()
                    self.solving_round_done()
            # time.sleep(0.01)
    
    def tree_log_display(self):
        if self.tree != None:
            self.tree.log_display()
    
    def clean_up(self):
        if self.rank == self.isolated_rank:
            if self.original_process != None:
                terminate_subprocess(self.original_process)
                self.original_process = None
        if self.status.is_solving():
            self.round_clean_up()
    
    def clean_temp_dir(self):
        if self.debug:
            logging.info(f'Coordinator-{self.rank} debug mode: skip deleting temp dir {self.temp_dir}')
            return
        logging.info(f'Coordinator-{self.rank} clean temp dir {self.temp_dir}')
        shutil.rmtree(self.temp_dir, ignore_errors=True)
    
    def __call__(self):
        try:
            if self.rank == self.isolated_rank:
                self.isolated_solve()
            else:
                self.interactive_solve()
        except TerminateMessage:
            logging.info(f'Coordinator-{self.rank} is Terminated by Leader')
        except MemoryOutMessage:
            logging.error(f'Coordinator-{self.rank} detected memory out')
            while True:
                msg_type = MPI.COMM_WORLD.recv(source=self.leader_rank, tag=1)
                assert(isinstance(msg_type, ControlMessage.L2C))
                if msg_type.is_terminate_coordinator():
                    break
        except Exception as e:
            logging.error(f'Coordinator-{self.rank} Exception: {e}')
            logging.error(f'{traceback.format_exc()}')
            MPI.COMM_WORLD.send(ControlMessage.C2L.notify_error,
                            dest=self.leader_rank, tag=1)
            while True:
                msg_type = MPI.COMM_WORLD.recv(source=self.leader_rank, tag=1)
                assert(isinstance(msg_type, ControlMessage.L2C))
                if msg_type.is_terminate_coordinator():
                    break
            # MPI.COMM_WORLD.Abort()
        self.clean_up()
        MPI.COMM_WORLD.Barrier()
        self.clean_temp_dir()
