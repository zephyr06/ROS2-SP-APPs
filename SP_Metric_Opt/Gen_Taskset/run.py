import subprocess
import time
import argparse
import os

# Function to check if the program is running
def is_program_running(program_name):
    # Run the `ps -e | grep program_name` command
    result = subprocess.run(
        ["ps", "-e"], capture_output=True, text=True
    )
    # Check if the program_name is in the output
    return program_name in result.stdout

# Function to run the program and keep polling
def run_program(program_name, program_command):
    # Start the program in the background
    subprocess.Popen(program_command)

# Function to run the program and keep polling
def run_and_poll_program(program_name, program_command):
    # Start the program in the background
    subprocess.Popen(program_command)
    sleep_interval = 1

    while True:
        # Check if the program is still running
        if is_program_running(program_name):
            print(f"{program_name} is still running. Sleeping for {sleep_interval} seconds...", end="\r")
            time.sleep(sleep_interval)
        else:
            print(f"{program_name} has exited. Exiting the script.")
            break

# Example usage
if __name__ == "__main__":
    cpu_count = os.cpu_count()
    if cpu_count<1:
        cpu_count = 1
    print(f'{cpu_count} cpus')
    # quit()

    # Define your program name and the command to start it
    #  ./tests/CSPSimulation_2 --input_folder TaskData/taskset_cfg_6_1_gen_1 --simt 800000 --inst_idx 0 --scheduler BR
    program_name  = "CSPSimulation_2"
    command = "./tests/CSPSimulation_2"
    #data_dir = '--input_folder TaskData/taskset_cfg_10_1_gen_1'
    #scheduler = '--scheduler BR'
    simt0 = '--simt'
    simt1 = '990000'
    #n_inst = 8

    parser = argparse.ArgumentParser(description="Run the CSPSimulation program.")
    parser.add_argument('--input_folder', required=True, help="Input folder for task data")
    parser.add_argument('--scheduler', required=True, help="Scheduler type")
    parser.add_argument('--n_inst', type=int, required=True, help="Number of instances to simulate")

    # Parse arguments
    args = parser.parse_args()

    data_dir0 = '--input_folder'
    data_dir1 = args.input_folder
    sched0 = "--scheduler"
    sched1 = args.scheduler
    n_inst = args.n_inst

    idx = 0
    while True:
        n = n_inst - idx
        if n>cpu_count:
            n = cpu_count
        print(f'try to simulate instance {idx} to {idx+n} ...')

        if n == 1:
            inst0 = '--inst_idx'    
            inst1 = f'{idx}'
            program_command = [command, data_dir0, data_dir1, simt0, simt1, sched0, sched1, inst0,inst1]
            run_program(program_name, program_command)        
        else:     
            for i in range(n-1):
                inst0 = '--inst_idx'
                inst1 = f'{i+idx}'
                program_command = [command, data_dir0, data_dir1, simt0, simt1, sched0, sched1, inst0,inst1]
                #print(program_command)
                #quit()
                run_program(program_name, program_command)

            # Run the program and start polling
            inst1 = f'{idx+n-1}'
            #program_command = [command, data_dir, simt, scheduler, inst]  # Replace with the command and arguments
            program_command = [command, data_dir0, data_dir1, simt0, simt1, sched0, sched1, inst0,inst1]
            run_and_poll_program(program_name, program_command)

        idx += n
        if idx>=n_inst:
            break

    print('simulation done')
