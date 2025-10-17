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
def run_save():
    # ---- ANALYSE ------------------------
    print(f"[main] --- MAPE-K Analysis stage ---")
    # Create problem from data in CSV
    output_folder="t0"
    csv_file = config.CSV_PATH
    problem_t0 = problem.SAVEProblem(output_folder=output_folder, csv_path=csv_file)
    print(f"[main] Problem created from CSV: {csv_file}")
    
    # Print and save dtmcs
    if config.VERBOSE: problem_t0.display()
    if config.SAVE_DTMC_FILES:
        problem_t0.save_dtmc_file()
    print(f"[main] DTMC files saved in: {problem_t0.output_folder}")
    
    # Get verification results
    print("[main] Starting verification process (pre-deployment)...")
    problem_t0.get_pmc_results()
    if config.VERBOSE: print(f"[main] Verification results:\n {problem_t0.verification_results}")

    # ---- PLAN ---------------------------
    i = 0
    max_iterations = config.MAX_PLANNING_ITERATIONS
    
    problem_tN = problem_t0
    situation_to_avoid = []
    
    # if violations exist
    while problem_tN.verification_results['Violation'].any() and i <= max_iterations:
        print(f"[main] --- MAPE-K Planning stage. Violations found. Planning required, iteration: {i+1}/{config.MAX_PLANNING_ITERATIONS}")
        i += 1
        
        # create new output folder
        output_folder="t"+str(i)
        
        # create planning object
        controllerAssessment = controllerAssess.ControllerAssessment(problem_tN)
        
        # get situation,property with "worst violation"
        w_sit, w_prop = controllerAssessment.get_worst_violation()
        print(f"[main] Worst violation: {w_sit}, {w_prop}")
        situation_to_avoid.append(w_sit)
        print(f"[main] Situations to avoid: {situation_to_avoid}")
        #TODO: Choose between sum of errors across situation or single worst violation (situation + property)
    
        # generate new problem with updated transitions (0 for situation_to_avoid)
        csv_file = controllerAssessment.create_new_csv_data(situation_to_avoid, output_folder) # with prob. 0.0 for situation_to_avoid
        # create new problem instance
        problem_tN = problem.SAVEProblem(output_folder=output_folder, csv_path=csv_file, ignore_states=situation_to_avoid)

        # ---- Print and save dtmcs
        if config.VERBOSE: problem_tN.display()
        if config.SAVE_DTMC_FILES:
            problem_tN.save_dtmc_file()
        # --- Get verification results
        problem_tN.get_pmc_results()
        if config.VERBOSE: print(problem_tN.verification_results)
        print("[main] Ignore states for next iteration:", situation_to_avoid)
        
        # update last problem
        problem_last = problem_tN

    if i == max_iterations and problem_tN.verification_results['Violation'].any():
        print("Max iterations reached.")
        return 
    
    # ---- Execute ---------------------------
    if not problem_tN.verification_results['Violation'].any():
        print("[main] No violations found. No need to plan.")
        return problem_t0
        
    # Note: Check DeepDecs for ideas:
    # https://github.com/ccimrie/DeepDECS/blob/master/case_studies/mobile_robot_collision_limitation/DTMC_model_files/models/model_no_verif.pm
    
    
    
    
    
    
    # generate new dtmcs
    # get new verification results
    # compare results
    
    #repeat until no violations or max iterations reached
    
    
    

    return problem_t0

def get_state(all_situations, situation):
    try:
        return all_situations.index(situation)
    except ValueError:
        raise ValueError(f"Situation {situation} not found in the list of situations.")    
