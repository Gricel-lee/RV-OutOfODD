import pandas as pd
import os
import sys
import utility.auxiliary as aux
from pathlib import Path


"""
Generate a PRISM model file from an augmented situation coverage grid (CSV/XLSX) file.
"""
def generate_prism_file(fpath, init_state=1):
    # --- Prepare paths ---    
    # Create "output" folder
    output_folder = aux.create_folder(str(Path(fpath).parent), "output")
    print("Output folder: "+output_folder)
    
    
    # --- Read CSV file ---
    df = aux.read_csv_or_xlsx(fpath)
    
    
    # print(df.head())
    
    # Get transitions
    transitions = {}
    for _, row in df.iterrows():
        situations = row['Situation']
        next_situations = row['NextSituation']
        prob = row['Probability']
        if situations not in transitions.keys():
            transitions[situations] = []
        transitions[situations].append((next_situations, prob))
    # print("Transitions: ", transitions)
    
    # Get all model states
    situations = df['Situation'].unique()
    next_situations = df['NextSituation'].unique()
    all_situations_n_failures = sorted(list(set(situations).union(set(next_situations))))
    print("All model states: ", all_situations_n_failures)

    
    # --- Write PRISM model files ---
    for sit in transitions.keys():
        # Get initial state
        init_state = f"const int init_situation = state_{sit};"
        
        # Define PRISM file path
        fname = os.path.basename(fpath)
        prism_file = os.path.join(output_folder, f"{sit}"+f"{fname.split('.')[0]}.prism")

        
        with open(prism_file, 'w') as f:
            f.write('dtmc\n\n')
            
            # Add comment with mapping of situations
            for i_state, situation in enumerate(all_situations_n_failures):
                f.write(f'const int state_{situation} = {i_state}; \n')
            f.write(f'\n{init_state}\n')
            f.write('\nmodule System\n')
            f.write(f'  s : [0..{len(all_situations_n_failures)-1}] init init_situation;\n\n')
            
            # Add transitions
            for situation in transitions.keys():
                f.write(f'  // Situation: {situation} \n')
                f.write(f'  [ ] s=state_{situation} -> ')
                for next_situation, prob in transitions[situation]:
                    f.write(f' {prob}:(s\'=state_{next_situation}) + ')
                f.seek(f.tell() - 3, os.SEEK_SET)
                f.write(';\n\n')
            f.write('endmodule\n')
        
        
    return prism_file

def get_state(all_situations, situation):
    try:
        return all_situations.index(situation)
    except ValueError:
        raise ValueError(f"Situation {situation} not found in the list of situations.")
    



if __name__ == "__main__":

    if len(sys.argv) > 1:
        fpath = sys.argv[1]
    else:
        print("Usage: python run_dtmc.py <path_to_augmented_grid_csv_file>")
        print("Example: python3 run_dtmc.py ../example_maritime/input/coverageGrid.xlsx")
        sys.exit(1)

    # Generate PRISM file
    prism_file = generate_prism_file(fpath, init_state=1)
    
    print("\nPRISM model saved in: "+prism_file)
