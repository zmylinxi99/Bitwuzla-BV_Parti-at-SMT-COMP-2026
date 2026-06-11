import os
import shutil
import time
import logging
import argparse
import traceback
from mpi4py import MPI
from datetime import datetime
from collections import deque

from coordinator import CoordinatorStatus
from partition_tree import NodeStatus
from partition_tree import DistributedNode, DistributedTree
from control_message import ControlMessage, CoordinatorErrorMessage, MemoryOutMessage

class CoordinatorInfo:
    def __init__(self, rank, start_time):
        self.status = CoordinatorStatus.idle
        self.assigned = None
        self.solving_round = 0
        self.split_count = 0
        self.last_split = 0.0
        self.last_assign = 0.0
        
        self.rank = rank
        self.start_time = start_time
        
    def get_current_time(self):
        return time.time() - self.start_time
    
    def assign_node(self, node: DistributedNode):
        assert(self.status.is_idle())
        self.status = CoordinatorStatus.solving
        self.assigned = node
        self.solving_round += 1
        self.split_count = 0
        self.last_assign = self.get_current_time()
        self.last_split = self.get_current_time()
        
    def split_node(self):
        assert(self.status.is_splitting())
        self.status = CoordinatorStatus.solving
        self.split_count += 1
        self.last_split = self.get_current_time()
        
    def node_solved(self):
        self.status = CoordinatorStatus.idle
        self.assigned = None

class Leader:
    def __init__(self):
        self.split_tabu = 3.0
        
        self.start_time = time.time()
        self.tree = DistributedTree(self.start_time)
        self.leader_rank = MPI.COMM_WORLD.Get_rank()
        self.isolated_rank = self.leader_rank - 1
        self.num_dist_coords = self.isolated_rank
        
        self.init_params()
        os.makedirs(self.temp_folder_path, exist_ok=True)
        os.makedirs(self.output_folder_path, exist_ok=True)

        self.init_logging()
        logging.debug(f'input_file_path: {self.input_file_path}')
        logging.debug(f'num_dist_coords: {self.num_dist_coords}')
        logging.debug(f'leader_rank: {self.leader_rank}')
        logging.debug(f'temp_folder_path: {self.temp_folder_path}')
        
        self.idle_coordinators = deque()
        # dist coords and isolated coord
        self.coordinators = [CoordinatorInfo(i, self.start_time) for i in range(self.num_dist_coords + 1)]
        ### TBD ### select split coordinator with priority
        self.next_split_rank = 0
        # logging.debug(f'init done!')
        
        if self.get_model_flag:
            self.model = None
        self.memory_out_reason = None
    
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
                                help='enable debug mode (accepted for launcher passthrough)')
        
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
        self.temp_folder_path: str = cmd_args.temp_dir
        self.output_folder_path: str = cmd_args.output_dir
        self.get_model_flag: bool = bool(cmd_args.get_model_flag)
        self.debug: bool = bool(cmd_args.debug)
        
        self.input_file_path: str = cmd_args.file
        self.time_limit: int = cmd_args.time_limit
        
        if not os.path.exists(self.input_file_path):
            print('file-not-found')
            assert(False)
        
        self.instance_name: str = self.input_file_path[ \
            self.input_file_path.rfind('/') + 1: self.input_file_path.find('.smt2')]
    
    def init_logging(self):
        log_dir_path = f'{self.output_folder_path}/logs'
        # os.makedirs(log_dir_path, exist_ok=True)
        logging.basicConfig(format='%(relativeCreated)d - %(levelname)s - %(message)s', 
                filename=f'{log_dir_path}/leader.log', level=logging.DEBUG)
        current_time = datetime.now()
        formatted_time = current_time.strftime("%Y-%m-%d %H:%M:%S")
        logging.info(f'start-time {formatted_time} ({self.start_time})')
    
    def get_current_time(self):
        return time.time() - self.start_time
    
    def is_done(self):
        return self.tree.is_done()
    
    def get_result(self):
        return self.tree.get_result()
    
    def update_assign_coordinator(self, parent: DistributedNode, idle_coord: int):
        child = self.tree.split_node(parent, idle_coord)
        self.coordinators[idle_coord].assign_node(child)
    
    def update_split_coordinator(self, split_coord: int):
        self.coordinators[split_coord].split_node()
    
    def update_split_assign_info(self, split_coord: int, idle_coord: int):
        parent = self.coordinators[split_coord].assigned
        assert(parent != None)
        self.update_assign_coordinator(parent, idle_coord)
        self.update_split_coordinator(split_coord)
        logging.info(f'split: (node-{self.coordinators[idle_coord].assigned.id} coordinator-{idle_coord}) '
                     f'from (node-{self.coordinators[split_coord].assigned.id} coordinator-{split_coord})')
    
    def set_coordinator_idle(self, coord_rank):
        self.coordinators[coord_rank].node_solved()
        self.idle_coordinators.append(coord_rank)
    
    def process_notified_result(self, src: int):
        result, model = MPI.COMM_WORLD.recv(source=src, tag=2)
        if self.get_model_flag and result.is_sat():
            assert(model is not None)
            self.model = model
        else:
            assert(model is None)
        if src == self.isolated_rank:
            logging.info(f'solved-by-isolated: {result}')
            self.tree.original_solved(result)
            return True
        else:
            node = self.coordinators[src].assigned
            logging.info(f'solved: node-{node.id} is {result}')
            self.tree.node_partial_solved(node, result)
            # self.tree.log_display()
            if self.is_done():
                return True
            self.set_coordinator_idle(src)
            return False
    
    def check_coordinators(self):
        msg_status = MPI.Status()
        while MPI.COMM_WORLD.Iprobe(source=MPI.ANY_SOURCE, tag=1, status=msg_status):
            src = msg_status.Get_source()
            msg_type: ControlMessage.C2L = MPI.COMM_WORLD.recv(source=src, tag=1)
            if msg_type.is_split_succeed():
                # split node {path} to coordinator {target_rank}
                logging.debug(f'receive {msg_type} message from coordinator-{src}')
                target_rank = MPI.COMM_WORLD.recv(source=src, tag=2)
                self.send_assign_message(src, target_rank)
                self.send_transfer_message(src, target_rank)
                self.update_split_assign_info(src, target_rank)
            elif msg_type.is_split_failed():
                target_rank = MPI.COMM_WORLD.recv(source=src, tag=2)
                self.idle_coordinators.append(target_rank)
                coord: CoordinatorInfo = self.coordinators[src]
                if coord.status.is_splitting():
                    coord.status = CoordinatorStatus.solving
            elif msg_type.is_notify_result():
                # coordinator {src} solved the assigned task
                logging.debug(f'receive {msg_type} message from coordinator-{src}')
                if self.process_notified_result(src):
                    return True
            elif msg_type.is_notify_memory_out():
                detail = MPI.COMM_WORLD.recv(source=src, tag=2)
                self.memory_out_reason = detail
                logging.error(f'memory out from coordinator-{src}: {detail}')
                raise MemoryOutMessage()
            elif msg_type.is_notify_error():
                logging.error(f'receive {msg_type} message from coordinator-{src}')
                raise CoordinatorErrorMessage()
            else:
                assert(False)
        return False
    
    def init_coord_0(self):
        self.tree.assign_root_node()
        self.coordinators[0].assign_node(self.tree.root)
        src_pre_path = f'{self.temp_folder_path}/Coordinator-{self.isolated_rank}/tasks/round-0'
        src_done_path = f'{src_pre_path}/task-0.done'
        src_task_path = f'{src_pre_path}/task-0.smt2'
        while not os.path.exists(src_done_path):
            if MPI.COMM_WORLD.Iprobe(source=self.isolated_rank, tag=1):
                msg_type: ControlMessage.C2L = MPI.COMM_WORLD.recv(source=self.isolated_rank, tag=1)
                logging.debug(f'receive {msg_type} message from coordinator-isolated')
                if msg_type.is_notify_result():
                    # coordinator {src} solved the assigned task
                    if self.process_notified_result(self.isolated_rank):
                        logging.debug(f'init_coord_isolated solved')
                        return True
                elif msg_type.is_notify_memory_out():
                    detail = MPI.COMM_WORLD.recv(source=self.isolated_rank, tag=2)
                    self.memory_out_reason = detail
                    logging.error(f'memory out from coordinator-isolated: {detail}')
                    raise MemoryOutMessage()
                # elif msg_type.is_pre_process_done():
                #     return False
                elif msg_type.is_notify_error():
                    raise CoordinatorErrorMessage()
                else:
                    assert(False)
            time.sleep(0.01)
        
        dest_path = f'{self.temp_folder_path}/Coordinator-0/tasks/round-0'
        os.makedirs(dest_path, exist_ok=True)
        shutil.copyfile(src_task_path, 
                        f'{dest_path}/task-0.smt2')
        MPI.COMM_WORLD.send(ControlMessage.L2C.assign_node, 
                            dest=0, tag=1)
        return False
    
    def init_coord_isolated(self):
        src_coord = self.isolated_rank
        src_path  = self.input_file_path
        dest_pre_path = f'{self.temp_folder_path}/Coordinator-{src_coord}/tasks/round-0'
        dest_path = f'{dest_pre_path}/task-0.smt2'
        os.makedirs(dest_pre_path, exist_ok=True)
        shutil.copyfile(src_path, dest_path)
        MPI.COMM_WORLD.send(ControlMessage.L2C.assign_node,
                            dest=src_coord, tag=1)
        self.tree.assign_root_node(src_coord)
        self.coordinators[src_coord].assign_node(self.tree.root)
        while True:
            if self.time_limit != 0 and self.get_current_time() >= self.time_limit:
                raise TimeoutError()
            if not MPI.COMM_WORLD.Iprobe(source=src_coord, tag=1):
                continue
            msg_type: ControlMessage.C2L = MPI.COMM_WORLD.recv(source=src_coord, tag=1)
            logging.debug(f'receive {msg_type} message from coordinator-isolated')
            if msg_type.is_notify_result():
                # coordinator {src} solved the assigned task
                if self.process_notified_result(src_coord):
                    return True
            elif msg_type.is_notify_memory_out():
                detail = MPI.COMM_WORLD.recv(source=src_coord, tag=2)
                self.memory_out_reason = detail
                logging.error(f'memory out from coordinator-isolated: {detail}')
                raise MemoryOutMessage()
            elif msg_type.is_pre_partition_done():
                if self.num_dist_coords == 0:
                    _ = MPI.COMM_WORLD.recv(source=src_coord, tag=2)
                    logging.debug('pre-partition ignored: no distributed coordinators')
                    continue
                self.pre_partition()
                return False
            elif msg_type.is_notify_error():
                raise CoordinatorErrorMessage()
            else:
                assert(False)
    
    def get_next_idle_coordinator(self):
        # FIFO
        if len(self.idle_coordinators) == 0:
            return None
        return self.idle_coordinators.popleft()
    
    def select_coordinator_to_split(self):
        if self.num_dist_coords == 0:
            return None
        # FIFO
        rank = self.next_split_rank
        self.next_split_rank = (rank + 1) % self.num_dist_coords
        split_coord: CoordinatorInfo = self.coordinators[rank]
    
        if split_coord.status.is_solving() and \
           self.get_current_time() >= split_coord.last_split + self.split_tabu:
            return rank
        else:
            return None
    
    def send_split_message(self, split_coord, idle_coord):
        MPI.COMM_WORLD.send(ControlMessage.L2C.request_split,
                            dest=split_coord, tag=1)
        MPI.COMM_WORLD.send(idle_coord, 
                            dest=split_coord, tag=2)
        
    def send_assign_message(self, split_coord, idle_coord):
        MPI.COMM_WORLD.send(ControlMessage.L2C.assign_node,
                            dest=idle_coord, tag=1)
        MPI.COMM_WORLD.send(split_coord, 
                            dest=idle_coord, tag=2)
    
    def send_transfer_message(self, split_coord, idle_coord):
        MPI.COMM_WORLD.send(ControlMessage.L2C.transfer_node,
                            dest=split_coord, tag=1)
        MPI.COMM_WORLD.send(idle_coord, 
                            dest=split_coord, tag=2)
    
    def assign_node_to_idle_coordinator(self):
        idle_coord = self.get_next_idle_coordinator()
        if idle_coord == None:
            return
        split_coord = self.select_coordinator_to_split()
        if split_coord == None:
            self.idle_coordinators.append(idle_coord)
            return
        # logging.info(f'assign node from coordinator-{split_coord} to coordinator-{idle_coord}')
        self.coordinators[split_coord].status = CoordinatorStatus.splitting
        self.send_split_message(split_coord, idle_coord)
    
    def pre_partition(self):
        src_coord = self.isolated_rank
        pp_num_nodes: int = MPI.COMM_WORLD.recv(source=src_coord, tag=2)
        MPI.COMM_WORLD.send(ControlMessage.L2C.assign_node, 
                    dest=src_coord, tag=1)
        
        if pp_num_nodes == 0:
            logging.debug('pre-partitioning failed.')
            pp_num_nodes = 1
        
        for i in range(self.num_dist_coords):
            if i < pp_num_nodes:
                self.coordinators[src_coord].status = CoordinatorStatus.splitting
                self.send_assign_message(src_coord, i)
                self.update_split_assign_info(src_coord, i)
            else:
                self.idle_coordinators.append(i)
            
        self.tree.node_partial_solved(self.tree.root, NodeStatus.unsat)
    
    def solve(self):
        if self.init_coord_isolated():
            return
        # communicate with coordinators
        while True:
            if self.check_coordinators():
                return
            self.assign_node_to_idle_coordinator()
            if self.time_limit != 0 and self.get_current_time() >= self.time_limit:
                raise TimeoutError()
            # time.sleep(0.01)
    
    def terminate_coordinators(self):
        for i in range(self.num_dist_coords):
            MPI.COMM_WORLD.send(ControlMessage.L2C.terminate_coordinator,
                                dest=i, tag=1)
        MPI.COMM_WORLD.send(ControlMessage.L2C.terminate_coordinator,
                                dest=self.isolated_rank, tag=1)
    
    def clean_up(self):
        self.terminate_coordinators()
    
    def __call__(self):
        try:
            self.solve()
        except TimeoutError:
            result = 'timeout'
            self.tree.log_display()
        except MemoryOutMessage:
            result = 'memoryout'
            if self.memory_out_reason:
                logging.error(f'memory out detail: {self.memory_out_reason}')
        # except AssertionError as ae:
        #     result = 'AssertionError'
        #     # print(f'AssertionError: {ae}')
        #     # logging.info(f'AssertionError: {ae}')
        except CoordinatorErrorMessage:
            result = 'Coordinator-Error'
            self.tree.log_display()
        except Exception as e:
            result = 'Leader-Error'
            logging.error(f'Leader Error: {e}')
            logging.error(f'{traceback.format_exc()}')
            # MPI.COMM_WORLD.Abort()
        else:
            status: NodeStatus = self.get_result()
            if status.is_sat():
                result = 'sat'
            elif status.is_unsat():
                result = 'unsat'
            else:
                assert(False)
            self.tree.log_display()
        
        end_time = time.time()
        execution_time = end_time - self.start_time
        
        print(result)
        if self.get_model_flag and result == 'sat':
            assert(self.model is not None)
            # print('model: ')
            print(self.model)
        logging.info(f'result: {result}, time: {execution_time}')
        
        if self.output_folder_path != None:
            with open(f'{self.output_folder_path}/result.txt', 'w') as f:
                f.write(f'{result}\n{execution_time}\n')
        
        self.clean_up()
        MPI.COMM_WORLD.Barrier()
        # MPI.COMM_WORLD.Abort()
