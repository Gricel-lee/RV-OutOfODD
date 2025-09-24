import pandas as pd
import os
import sys
import utility.auxiliary as aux
from pathlib import Path
import augmented_scg.problem as problem
import augmented_scg.Planning.controllerAssess as controllerAssess
import config.config as config

"""
Generate a PRISM model file from an augmented situation coverage grid (CSV/XLSX) file.
"""
def generate_prism_file():
    # ---- ANALYSE ------------------------
    # --- Create problem from data in CSV
    output_folder="t0"
    csv_file = config.CSV_PATH
    problem_t0 = problem.Problem(output_folder=output_folder, csv_path=csv_file)
    # ---- Print and save dtmcs
    problem_t0.display()
    if config.SAVE_DTMC_FILES:
        problem_t0.save_dtmc_files()
    # --- Get verification results
    problem_t0.get_pmc_results()
    print(problem_t0.verification_results)
    
    # ---- PLAN ---------------------------
    i = 0
    max_iterations = config.MAX_PLANNING_ITERATIONS
    
    problem_tN = problem_t0
    situation_to_avoid = []
    
    # if violations exist
    while problem_tN.verification_results['Violation'].any() and i <= max_iterations:
        print(f"--- Planning iteration {i}: Violations found. Planning required.---")
        i += 1
        
        # create new output folder
        output_folder="t"+str(i)
        
        # create planning object
        controllerAssessment = controllerAssess.ControllerAssessment(problem_t0)
        
        # get situation,property with "worst violation"
        w_sit, w_prop = controllerAssessment.get_worst_violation()
        print("[Planning] Worst violation:\n", w_sit, w_prop)
        situation_to_avoid.append(w_sit)
        print("[Planning] Situations to avoid:", situation_to_avoid)
        #TODO: Choose between sum of errors across situation or single worst violation (situation + property)
    
        # generate new problem with updated transitions (0 for situation_to_avoid)
        csv_file = controllerAssessment.create_new_csv_data(situation_to_avoid, output_folder) # with prob. 0.0 for situation_to_avoid
        # create new problem instance
        problem_tN = problem.Problem(output_folder=output_folder, csv_path=csv_file, ignore_states=situation_to_avoid)

        # ---- Print and save dtmcs
        problem_tN.display()
        if config.SAVE_DTMC_FILES:
            problem_tN.save_dtmc_files()
        # --- Get verification results
        problem_tN.get_pmc_results()
        print(problem_tN.verification_results)

        print("Ignore states for next iteration:", controllerAssessment.situation_to_avoid)
        
        # update last problem
        problem_last = problem_tN

    if i == max_iterations and problem_t0.verification_results['Violation'].any():
        print("Max iterations reached.")
        return 
    
    # ---- Execute ---------------------------
    if not problem_t0.verification_results['Violation'].any():
        print("No violations found. No need to plan.")
        return 
        
    # Note: Check DeepDecs for ideas:
    # https://github.com/ccimrie/DeepDECS/blob/master/case_studies/mobile_robot_collision_limitation/DTMC_model_files/models/model_no_verif.pm
    
    
    
    
    
    
    # generate new dtmcs
    # get new verification results
    # compare results
    
    #repeat until no violations or max iterations reached
    
    
    

    return

def get_state(all_situations, situation):
    try:
        return all_situations.index(situation)
    except ValueError:
        raise ValueError(f"Situation {situation} not found in the list of situations.")
    


if __name__ == "__main__":

    # if len(sys.argv) > 1:
    #     fpath = sys.argv[1]
    # else:
    #     print("Usage: python run_dtmc.py <path_to_augmented_grid_csv_file>")
    #     print("Example: python3 run_dtmc.py ../example_maritime/t_0/input/coverageGrid.csv")
    #     sys.exit(1)

    # Generate PRISM file
    prism_file = generate_prism_file()

    
