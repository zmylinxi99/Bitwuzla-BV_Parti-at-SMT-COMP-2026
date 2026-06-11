#!/usr/bin/env python3
import os
import sys
import json
import subprocess
import random
import string
import shutil

def generate_random_string(length):
    characters = string.ascii_letters + string.digits
    return ''.join(random.choice(characters) for _ in range(length))

def prepare_temp_folder():
    temp_folder_name = generate_random_string(16)
    temp_folder_path = os.path.join('/tmp/bvp-files', temp_folder_name)
    os.makedirs(temp_folder_path, exist_ok=True)
    return temp_folder_path

if __name__ == '__main__':
    if len(sys.argv) < 3:
        # ./Bitwuzla-BV_Parti-at-SMT-COMP-2026-build/solver/run_BV_Parti.py 128 ./Bitwuzla-BV_Parti-at-SMT-COMP-2026-build/test/instances/bv-unsat-3.05.smt2
        sys.exit('Usage: ./Bitwuzla-BV_Parti-at-SMT-COMP-2026-build/solver/run_BV_Parti.py <parallel_core_num> <instance_path>')
    parallel_core_num = int(sys.argv[1])
    instance_path = sys.argv[2]

    script_path = os.path.abspath(__file__)
    script_dir = os.path.dirname(script_path)

    bv_parti_path = os.path.join(script_dir, 'BV-Parti_launcher.py')

    # Create temporary folder for files
    temp_folder_path = prepare_temp_folder()
    # print(f'Output files are in: {temp_folder_path}')

    input_dict = {
        'formula_file': instance_path,
        'timeout_seconds': 1200,
        'base_solver': 'bitwuzla-0.9.1-bin',
        'mode': 'parallel',
        'parallel_core': parallel_core_num,
        'output_dir': temp_folder_path
    }

    config_path = os.path.join(temp_folder_path, 'input.json')
    with open(config_path, 'w') as file:
        json.dump(input_dict, file, indent=4)

    try:

        cmd = ' '.join([sys.executable, bv_parti_path,
                        config_path
                    ])
        # print(cmd)
        result = subprocess.run(cmd,
                                shell=True,
                                stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE
                            )
    except subprocess.CalledProcessError as e:
        # print(f'Error running BV_Parti: {e}')
        pass
    else:
        sys.stdout.write(result.stdout.decode('utf-8'))
        sys.stderr.write(result.stderr.decode('utf-8'))

    shutil.rmtree(temp_folder_path, ignore_errors=True)

