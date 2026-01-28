import pandas as pd
from save import *
import csv
import numpy as np
import os
from pathlib import Path
import time
import matplotlib.pyplot as plt

def createProblemInstance(transition_csv_file, file_id="", unsafe_config=[]):
  save_eval_dir_env="SAVE_EVAL_PATH"
  transition_matrix=readTransitionMatrix(transition_csv_file, unsafe_config)
  saveTransitionProbs(transition_matrix)
  casestudy_dir=os.getenv(save_eval_dir_env)
  output_folder=os.path.join(casestudy_dir,f"save_output/{file_id}/t0")
  csv_file = config.CSV_PATH
  problem_t0 = problem.SAVEProblem(output_folder=output_folder, csv_path=csv_file)
  
  # Print and save dtmcs
  if config.SAVE_DTMC_FILES:
    problem_t0.save_dtmc_file()
  return problem_t0


def readTransitionMatrix(transition_csv_file, unsafe_config=[]):
  with open(transition_csv_file, newline='') as csvfile:
      csv_reader=csv.reader(csvfile, delimiter=',')
      rows=[rows[1:] for rows in csv_reader]
  transition_matrix=np.array([[float(val) for val in row] for row in rows[1:]])
  for s in unsafe_config:
    transition_matrix[s+1][:]=0
  transition_sum=np.sum(transition_matrix,axis=1)
  transition_sum_aug=transition_sum+1*(transition_sum<1)
  transition_matrix/=(transition_sum_aug[:,None]);
  return transition_matrix


def saveTransitionProbs(transition_matrix):
  save_eval_dir_env="SAVE_EVAL_PATH"
  sim_dir_path=os.getenv(save_eval_dir_env)
  f=open(f"{sim_dir_path}/save_setup/input/coverageGrid.csv",'w')
  first_row=f"Situation,Next,Probability\n"
  f.write(first_row)
  for i in np.arange(2,len(transition_matrix)):
    for j in np.arange(len(transition_matrix)):
      p=transition_matrix[i][j]
      next_state=f"s{j-1}" if j>1 else f"f{j+1}"
      line=f"s{i-1},{next_state},{p}\n"
      f.write(line)
  f.close()
  return


def verifyModel(problem_t0, situation):
  for i_prop in range(len(problem_t0.properties)):
    prop=problem_t0.properties[i_prop]
    bound=problem_t0.prop_bounds[i_prop]

    # Get pmc result
    result=aux.run_prism_command(problem_t0.dtmc_file, prop, situation.i_state)

    # Determine if the bound is satisfied
    violation=not( eval(str(result) + bound) ) 

    # Calculate error to bound
    error=None
    if violation:
      # Simplified and safer error calculation
      if '<=' in bound:
          error = result - float(bound.replace('<=', '').strip())
      elif '>=' in bound:
          error = float(bound.replace('>=', '').strip()) - result
      elif '<' in bound:
          # Add a small epsilon for strict inequalities
          error = result - float(bound.replace('<', '').strip()) + 1e-9
      elif '>' in bound:
          error = float(bound.replace('>', '').strip()) - result + 1e-9
      return False
  return True


def generateRandomTM(situation_no, count_min, count_max, transition_csv_file_path):
  lines=",f1,f2,"
  lines+=",".join([f"s{s}" for s in np.arange(situation_no)])
  lines+="\n"
  lines+="f1,"+",".join(["0" for s in np.arange(situation_no+2)])+"\n"
  lines+="f2,"+",".join(["0" for s in np.arange(situation_no+2)])+"\n"
  for s in np.arange(situation_no):
    lines+=(f"s{s},"+",".join([str(val) for val in np.random.randint(count_min, count_max, situation_no+2)])+"\n")
  with open(transition_csv_file_path,'w') as f:
    f.write(lines)
    f.close()
  return