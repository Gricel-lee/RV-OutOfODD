import os
import pandas as pd
import numpy as np
import sys
import subprocess
from typing import List

from config import config

def create_folder(fpath, name):
    # Get the directory from the file path
    dir_path = os.path.join(os.path.dirname(fpath), name)

    # Create the directory if it doesn't exist
    os.makedirs(dir_path, exist_ok=True)  # `exist_ok=True` avoids errors if the directory already exists
    
    return dir_path

def convert_xlsx_to_csv(xlsx_file):
    try:
        # Extract the directory and file name (without extension)
        directory, filename = os.path.split(xlsx_file)
        basename, _ = os.path.splitext(filename)
        
        # Construct the output file path with the same name but .csv extension
        csv_file = os.path.join(directory, f"{basename}.csv")
        
        # Read/write file
        df = pd.read_excel(xlsx_file)
        df.to_csv(csv_file, index=False)
        
        print(f"Successfully converted {xlsx_file} to {csv_file}")
    except Exception as e:
        print(f"Error occurred: {e}")
        sys.exit(1)
    
    return csv_file

def read_csv_or_xlsx(fpath):
    if fpath.endswith('.xlsx'):
        fpath = convert_xlsx_to_csv(fpath)
        # df = pd.read_csv(fpath, index_col=False)
        # os.remove(fpath)
        df = pd.read_excel(fpath)
        return df
    elif not fpath.endswith('.csv'):
        raise ValueError("Input file must be a CSV or Excel file (.csv or .xlsx)")
    df = pd.read_csv(fpath, index_col=False)
    return df

def check_file_extension(fpath):
    if fpath.endswith('.xlsx'):
        fpath = convert_xlsx_to_csv(fpath)
    elif not fpath.endswith('.csv'):
        raise ValueError("Input file must be a CSV or Excel file (.csv or .xlsx)")
    return fpath


def get_dtmc_model(problem_instance, ignore_states: List[str]=[]) -> str:
    '''Generates the DTMC model for the given problem instance.
    If ignore_states is given, transitions from those states will be removed.'''
    # get all situations and failures from the problem
    all_situations_n_failures = problem_instance.states

    s = ""
    s+= 'dtmc\n\n'
    for i_state, situation in enumerate(all_situations_n_failures):
        s+= f'  const int {situation} = {i_state}; \n'
    s+= f'\nconst int init_situation;\n'
    s+= '\nmodule System\n'
    s+= f'  s : [0..{len(all_situations_n_failures)-1}] init init_situation;\n\n'
    for i in sorted(problem_instance.situations.keys()):
        p_sum=np.sum([p for next_situation, p in problem_instance.situations[i].transitions])
        # p_sum=np.sum(problem_instance.situations[i].transitions[:,1])
        if i not in ignore_states and p_sum>0:    
            s+= f'  // Situation: {i}\n'
            s+= f'  [check_situation] s={i} & t=0 & time_close<time_MAX -> '
            for next_situation, prob in problem_instance.situations[i].transitions:
                s+= f' {prob}:(s\'={next_situation}) + '
            s = s.rstrip(' + ') + ';\n\n'
    s+= 'endmodule\n'
    lines="""
const int time_MAX;

module timeClose
  time_close : [0..time_MAX] init 0;
"""
    close_states=[]
    for i in sorted(problem_instance.situations.keys()):
        check_close_mod=(int(i[1:])-2)%problem_instance.close_state_mod==0
        check_val=int(i[1:])==2
        if int(i[1:])>1 and (check_close_mod or check_val):
            close_states.append(i)
    close_states_str="("
    for i in close_states[:-1]:
        close_states_str+=f"s={i} | "
    close_states_str+=f"s={close_states[-1]})"
    far_state=close_states_str.replace("|", "&")
    far_state=far_state.replace("=", "!=")
    lines+=f"""
  [monitor_time] t=1 & {close_states_str} & time_close<time_MAX -> 1:(time_close'=time_close+1);
  [monitor_time] t=1 & {far_state} -> 1:(time_close'=0);
endmodule

module Turn
  // 0: checking situation
  // 1: check number of situations spent at close distance
  t : [0..1] init 0;
  [check_situation] true -> 1:(t'=1);
  [monitor_time] true -> 1:(t'=0);
endmodule
"""  
    s+=lines
    return s

# def get_dtmc_model(problem_instance, ignore_states: List[str]=[]) -> str:
#     '''Generates the DTMC model for the given problem instance.
#     If ignore_states is given, transitions from those states will be removed.'''
#     # get all situations and failures from the problem
#     all_situations_n_failures = problem_instance.states

#     s = ""
#     s+= 'dtmc\n\n'
#     for i_state, situation in enumerate(all_situations_n_failures):
#         s+= f'  const int {situation} = {i_state}; \n'
#     s+= f'\nconst int init_situation;\n'
#     s+= '\nmodule System\n'
#     s+= f'  s : [0..{len(all_situations_n_failures)-1}] init init_situation;\n\n'
#     for i in sorted(problem_instance.situations.keys()):
#         if i not in ignore_states:    
#             s+= f'  // Situation: {i}\n'
#             s+= f'  [ ] s={i} -> '
#             for next_situation, prob in problem_instance.situations[i].transitions:
#                 s+= f' {prob}:(s\'={next_situation}) + '
#             s = s.rstrip(' + ') + ';\n\n'
#     s+= 'endmodule\n'
#     return s


def run_prism_command(model_file: str, properties_file: str, init_situation_int: int):
    # Check if the required files exist before trying to run the command
    if not os.path.exists(model_file):
        print(f"Error: Model file not found at '{model_file}'")
        return
    # Command Execution  (see https://www.prismmodelchecker.org/manual/RunningPRISM/Experiments)
    command = [config.PRISM_PATH, model_file, "-pf", properties_file, "-const", f"init_situation={init_situation_int},time_MAX={config.TIME_MAX}"]
    # print(f"Executing command: {' '.join(command)}")

    try:
        result = subprocess.run(
            command,
            capture_output=True,
            text=True,
            check=True
        )
    except FileNotFoundError:
        print("\n--- ERROR ---")
        print("Command 'prism' not found.")
        print("Please ensure the PRISM model checker is installed and that its")
        print("executable is in your system's PATH.")
    except subprocess.CalledProcessError as e:
        # This block runs if the command executes but returns an error code.
        print("\n--- PRISM Execution Failed ---")
        print(f"Return Code: {e.returncode}")
        print("--- Standard Output (stdout) ---")
        print(e.stdout)
        print("\n--- Standard Error (stderr) ---")
        print(e.stderr)
        print("--------------------------------")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        
    # Parse only value
    parsed_result = parse_prism_output(result.stdout)
    
    # # --- Display Results ---
    # print("\n--- PRISM Execution Successful ---")
    # print("--- Standard Output (stdout) ---")
    # print(result.stdout)
    # print("---------------------------------")


    # # --- Display Results ---
    # print("\n--- PRISM Execution Successful. Result: ---")
    # print(parsed_result)
    # print("---------------------------------")

    return parsed_result




def parse_prism_output(output_string: str) -> float:
    """
    Parses the stdout from PRISM to find and return the numerical result.
    """
    for line in output_string.splitlines():
        # Strip leading/trailing whitespace to handle different formatting
        clean_line = line.strip()
        if clean_line.startswith("Result:"):
            parts = clean_line.split()
            # Expected format: "Result:" "0.1234" ...
            if len(parts) > 1:
                try:
                    # The number is the second part
                    return float(parts[1])
                except ValueError:
                    # Could not convert the part to a float, so continue
                    print(f"Warning: Found 'Result:' line but could not parse float from '{parts[1]}'")
                    continue
    # If the loop completes without finding the result
    return None