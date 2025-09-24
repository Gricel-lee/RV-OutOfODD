
import os
import pandas as pd
import sys
import subprocess

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




def run_prism_command(model_file: str, properties_file: str):
    # --- Pre-computation Checks ---
    # # Check if the required files exist before trying to run the command
    if not os.path.exists(model_file):
        print(f"Error: Model file not found at '{model_file}'")
        return
    # if not os.path.exists(properties_file):
    #     print(f"Error: Properties file not found at '{properties_file}'")
    #     return

    # --- Command Execution ---
    command = [config.PRISM_PATH, model_file, "-pf", properties_file]
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




def parse_prism_output(output_string: str) -> float | None:
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