
import os
import pandas as pd
import sys

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
        # fpath = convert_xlsx_to_csv(fpath)
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


