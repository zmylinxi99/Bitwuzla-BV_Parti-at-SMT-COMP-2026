import os
import re
import sys
import json
import time
import argparse
import shlex
import shutil
import string
import random
import logging
import subprocess

def generate_random_string(length=16):
    """Generate a random alphanumeric string."""
    return ''.join(random.choices(string.ascii_letters + string.digits, k=length))

def get_logic(file_path):
    """Extract logic from SMT-LIB file."""
    try:
        with open(file_path, 'r') as f:
            content = f.read()
            match = re.search(r'set-logic ([A-Z_]+)', content)
            return match.group(1) if match else None
    except Exception as e:
        logging.error(f"Failed to read formula file: {e}")
        return None

def check_get_model_flag(file_path):
    """Check if (get-model) exists uncommented in SMT2 file."""
    try:
        with open(file_path, 'r') as f:
            for line in f:
                if '(get-model)' in line.split(';')[0]:
                    return 1
        return 0
    except Exception as e:
        logging.error(f"Error checking get-model flag: {e}")
        return 0

def init_logging(log_dir, debug=False):
    """Initialize logging."""
    if os.path.exists(log_dir):
        if not debug:
            shutil.rmtree(log_dir)
    os.makedirs(log_dir, exist_ok=True)
    logging.basicConfig(
        format='%(asctime)s [%(levelname)s] %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S',
        filename=os.path.join(log_dir, 'launcher.log'),
        level=logging.DEBUG
    )
    if debug:
        logging.info("Debug mode enabled: keep existing log files.")
    logging.info("=== BV-Parti Launcher Started ===")

def load_config(config_path):
    """Load JSON config and validate fields."""
    try:
        with open(config_path, 'r') as f:
            config = json.load(f)
    except Exception as e:
        sys.exit(f"Failed to load config: {e}")
    
    required_fields = ['formula_file', 'timeout_seconds', 'base_solver', 'mode']
    for field in required_fields:
        if field not in config:
            sys.exit(f"Missing required field: {field}")

    if config['mode'] == 'parallel':
        if 'parallel_core' not in config:
            sys.exit("'parallel_core' is required for mode=parallel")
        config.setdefault('network_subnet', '127.0.0.1/32')
        config['worker_node_ips'] = ['localhost']
        config['worker_node_cores'] = [config['parallel_core']]
    elif config['mode'] == 'distributed':
        for field in ['worker_node_ips', 'worker_node_cores', 'network_subnet']:
            if field not in config:
                sys.exit(f"'{field}' is required for mode=distributed")
    else:
        sys.exit(f"Unsupported mode: {config['mode']}")

    # if 'max_total_memory_mb' in config:
    #     sys.exit("'max_total_memory_mb' is deprecated; use 'max_worker_memory_mb'")
    config.setdefault('max_worker_memory_mb', 0)
    if not isinstance(config['max_worker_memory_mb'], int):
        sys.exit("'max_worker_memory_mb' must be an integer (MB)")
    if config['max_worker_memory_mb'] < 0:
        sys.exit("'max_worker_memory_mb' must be >= 0")
    config.setdefault('output_dir', './output')
    config.setdefault('output_total_time', False)
    os.makedirs(config['output_dir'], exist_ok=True)
    return config


def reset_output_dir_for_debug(output_dir):
    """In debug mode, always start from a clean output directory."""
    if os.path.exists(output_dir):
        if os.path.isdir(output_dir):
            shutil.rmtree(output_dir)
        else:
            os.remove(output_dir)
    os.makedirs(output_dir, exist_ok=True)

def prepare_rankfile(rankfile_path, worker_node_ips, isolated_only=False):
    """Write MPI rankfile."""
    try:
        with open(rankfile_path, 'w') as f:
            host = worker_node_ips[0] if worker_node_ips else 'localhost'
            if isolated_only:
                # Rank 0: isolated coordinator, Rank 1: leader.
                f.write(f"rank 0={host} slot=*\n")
                f.write(f"rank 1={host} slot=*\n")
            else:
                for idx, ip in enumerate(worker_node_ips):
                    f.write(f"rank {idx}={ip} slot=*\n")
                # Add extra ranks for isolated coordinator and leader.
                f.write(f"rank {len(worker_node_ips)}={host} slot=*\n")
                f.write(f"rank {len(worker_node_ips)+1}={host} slot=*\n")
        logging.info(f"Rankfile written to {rankfile_path}")
    except Exception as e:
        sys.exit(f"Failed to write rankfile: {e}")

def prepare_temp_folder(debug=False):
    """Create temporary folder."""
    base_path = '/tmp/bvp-files'
    os.makedirs(base_path, exist_ok=True)
    if debug:
        temp_path = os.path.join(base_path, 'DEBUG')
        if os.path.isdir(temp_path):
            try:
                existing_count = len(os.listdir(temp_path))
                if existing_count > 0:
                    logging.info(
                        f"Debug temp folder already has {existing_count} item(s): {temp_path}"
                    )
            except OSError as e:
                logging.warning(f"Failed to inspect debug temp folder {temp_path}: {e}")
    else:
        temp_path = os.path.join(base_path, generate_random_string())
    os.makedirs(temp_path, exist_ok=True)
    logging.info(f"Temporary folder created: {temp_path}")
    return temp_path

def build_mpi_command(config, temp_folder, rankfile_path, debug=False):
    """Build MPI execution command."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    dispatcher = os.path.join(script_dir, 'dispatcher.py')
    solver_bin = os.path.join(script_dir, 'binaries', config['base_solver'])
    partitioner_bin = os.path.join(script_dir, 'binaries', 'partitioner-bin')

    for binary in [solver_bin, partitioner_bin]:
        if not os.path.isfile(binary):
            sys.exit(f"Missing binary: {binary}")

    cmd = [
        'mpiexec',
        '--mca', 'oob_tcp_if_include', config['network_subnet'],
        '--mca', 'btl_tcp_if_include', config['network_subnet'],
        '--mca', 'btl', 'self,tcp',
        '--allow-run-as-root',
        '--use-hwthread-cpus',
        '--bind-to', 'none',
        '--rankfile', rankfile_path,
        
        'python3', dispatcher,
        '--temp-dir', temp_folder,
        '--output-dir', config['output_dir'],
        '--get-model-flag', str(check_get_model_flag(config['formula_file'])),
        '--file', config['formula_file'],
        '--time-limit', str(config['timeout_seconds']),
        '--max-worker-memory-mb', str(config['max_worker_memory_mb']),
        '--solver', solver_bin,
        '--available-cores-list', json.dumps(config['worker_node_cores']),
        '--partitioner', partitioner_bin
    ]
    if debug:
        cmd.append('--debug')
    return shlex.join(cmd)

def parse_args():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(description='BV-Parti launcher')
    parser.add_argument('config', help='Path to config JSON file.')
    parser.add_argument('--debug', action='store_true',
                        help='Enable debug mode: fixed temp dir and no cleanup.')
    return parser.parse_args()

def dump_debug_context(output_dir, config_path, temp_folder, rankfile_path, cmd):
    """Persist debug context for reproduction."""
    debug_context = {
        'config_path': os.path.abspath(config_path),
        'temp_folder': temp_folder,
        'rankfile_path': rankfile_path,
        'mpi_command': cmd
    }
    debug_context_path = os.path.join(output_dir, 'debug_context.json')
    with open(debug_context_path, 'w') as f:
        json.dump(debug_context, f, indent=2)
    logging.info(f"Debug context written to {debug_context_path}")

def adjust_cores_for_isolated_coordinator(config):
    """Adjust cores: reserve cores for leader and isolated coordinator."""
    server_0_cores = config['worker_node_cores'][0]
    if server_0_cores > 16:
        reserved_cores = 8
    elif server_0_cores > 8:
        reserved_cores = 4
    elif server_0_cores > 4:
        reserved_cores = 2
    else:
        sys.exit(f"Error: Not enough cores on first node to reserve cores for isolated coordinator.")
    
    # Reserve cores
    config['worker_node_cores'][0] -= reserved_cores  # reserved_cores for coordinator
    config['worker_node_cores'].append(reserved_cores)
    logging.info(f"Reserved {reserved_cores} cores for isolated coordinator on first node.")
    logging.info(f"Adjusted worker_node_cores to {config['worker_node_cores']}")

def should_use_isolated_only(config):
    """Use isolated coordinator only for small parallel core counts."""
    if config['mode'] != 'parallel':
        return False
    return config['parallel_core'] <= 8


if __name__ == '__main__':
    args = parse_args()
    config = load_config(args.config)
    if args.debug:
        reset_output_dir_for_debug(config['output_dir'])
    
    if not os.path.exists(config['formula_file']):
        sys.exit(f"Formula file does not exist: {config['formula_file']}")
    
    log_dir = os.path.join(config['output_dir'], 'logs')
    init_logging(log_dir, debug=args.debug)
    if args.debug:
        logging.info("Launcher running with --debug")

    isolated_only = should_use_isolated_only(config)
    if isolated_only:
        logging.info("parallel_core <= 8: run isolated coordinator only.")
    else:
        adjust_cores_for_isolated_coordinator(config)
    logging.info(f"Configuration: {json.dumps(config, indent=2)}")

    temp_folder = prepare_temp_folder(debug=args.debug)
    rankfile_path = os.path.join(config['output_dir'], 'rankfile')
    prepare_rankfile(rankfile_path, config['worker_node_ips'], isolated_only=isolated_only)

    cmd = build_mpi_command(config, temp_folder, rankfile_path, debug=args.debug)
    logging.info(f"MPI Command: {cmd}")
    if args.debug:
        dump_debug_context(config['output_dir'], args.config, temp_folder, rankfile_path, cmd)
        print(f"[DEBUG] temp_dir={temp_folder}")
        print(f"[DEBUG] rankfile={rankfile_path}")
        print(f"[DEBUG] launcher_log={os.path.join(log_dir, 'launcher.log')}")

    if config['output_total_time']:
        start_time = time.time()

    try:
        result = subprocess.run(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        logging.info("STDOUT:\n" + result.stdout.decode())
        logging.info("STDERR:\n" + result.stderr.decode())
        logging.info(f"Return code: {result.returncode}")
        sys.stdout.write(result.stdout.decode())
        if args.debug and result.stderr:
            sys.stderr.write(result.stderr.decode())
    except Exception as e:
        logging.error(f"Subprocess failed: {e}")
        sys.exit(1)
    
    if config['output_total_time']:
        elapsed = time.time() - start_time
        logging.info(f"Total execution time: {elapsed:.2f} seconds")
        print(f"\nTotal execution time: {elapsed:.2f} seconds")
