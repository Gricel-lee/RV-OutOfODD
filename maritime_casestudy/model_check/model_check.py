 # import numpy as np
from itertools import combinations
import pandas as pd
from save import *
import csv
import numpy as np
import os
from pathlib import Path
from copy import deepcopy
import ast
import curses 
from curses import wrapper
import time

def strKey(key):
  str_key="["+",".join(key)+"]"
  return str_key

def createProblemInstance(transition_csv_file, unsafe_config=[]):
  transition_matrix=readTransitionMatrix(transition_csv_file, unsafe_config)
  saveTransitionProbs(transition_matrix)
  casestudy_dir=os.getenv("SIM_DIR_PATH")
  output_folder=f"{casestudy_dir}/build/data/save_output"
  csv_file = config.CSV_PATH
  problem_t0 = problem.SAVEProblem(output_folder=f"{output_folder}/t0", csv_path=csv_file)
  # Print and save dtmcs
  # if config.SAVE_DTMC_FILES:
  #   problem_t0.save_dtmc_file()
  return problem_t0


def readTransitionMatrix(transition_csv_file, unsafe_config=[]):
  with open(transition_csv_file, newline='') as csvfile:
      csv_reader=csv.reader(csvfile, delimiter=',')
      rows=[rows[1:] for rows in csv_reader]
  transition_matrix=np.array([[float(val) for val in row] for row in rows[1:]])
  # print(transition_matrix)
  for s in unsafe_config:
    transition_matrix[s+1][:]=0
  transition_sum=np.sum(transition_matrix,axis=1)
  transition_sum_aug=transition_sum+1*(transition_sum<1)
  transition_matrix/=(transition_sum_aug[:,None]);
  return transition_matrix


def saveTransitionProbs(transition_matrix):
  sim_dir_path=os.getenv("SIM_DIR_PATH")
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


def checkCombos(stdscr, transition_csv_file, situations, weights, viable_controllers):
  """
  Runs PRISM for each situation and property, evaluates the results, and returns them as a single DataFrame.
  """
  situations=situations.values()
  N=len(situations)
  res=[[]]
  [res.append(comb) for i in range(1, N+1) for comb in combinations(situations, i)]
  
  def checkCombo(problem_t0, situation, combo):
    ## TODO: update CSV file

    ## Update properties files
    combo_name=[c.name for c in combo]
    properties=deepcopy(problem_t0.properties)
    prop_bounds=deepcopy(problem_t0.prop_bounds)
    if len(combo_name)>0:
      prop_template="P=?[F<50 {0}]"
      unsafe_situation_str=[]
      for c in combo:
        unsafe_situation_str.append(f"s!={c.name}")
      properties.append(prop_template.format("|".join(unsafe_situation_str)))
      prop_bounds.append('>0.9')
    
    for i_prop in range(len(properties)):
      prop=properties[i_prop]
      bound=prop_bounds[i_prop]

      # Get pmc result
      result=aux.run_prism_command(problem_t0.dtmc_file, prop, situation.i_state)
  
      # Determine if the bound is satisfied
      violation=not( eval(str(result) + bound) ) 
      # stdscr.addstr(6, 0, f"{situation.name}+{combo_name}+{prop}->{violation}:  {result}{bound}")
      # stdscr.clrtoeol()

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
      
  def updateViableControllers(problem_t0, combo):
    combo_name=[c.name for c in combo]
    vals=viable_controllers.iloc[0][strKey(combo_name)]
    if (isinstance(vals, str)):
      vals=ast.literal_eval(vals)
    try:
      vals.append(tuple([float(w) for w in weights]))
    except:
      stdscr.addstr(7,0,f"\t\t PROBLEM  {vals}  {weights}")
      stdscr.refresh()
      time.sleep(100)
      sys.exit()
    viable_controllers.iat[0, viable_controllers.columns.get_loc(strKey(combo_name))]=vals
    
    casestudy_dir=os.getenv("SIM_DIR_PATH")
    viable_controllers.to_csv(path_or_buf=f"{casestudy_dir}/build/data/viable_controllers/viable_controllers.csv")

 # For each situation, for each property, run prism and store result
  update_progress="-Unsafe configuration: {0}  {1}%"
  situation_progress="\t-Situation progress: {0}  {1}%"
  
  count_combo=0
  vc_list=[]
  stdscr.addstr(0, 0, f"--Checking for weights {weights}--")
  for combo in res:
    count_situation=0
    vc=True
    problem_t0=createProblemInstance(transition_csv_file, [int(c.name[1:]) for c in combo])
    situations_sub_list=[s for s in situations if s not in combo]
    stdscr.clrtoeol()
    combo_name=strKey([c.name for c in combo])
    stdscr.clrtoeol()
    for situation in situations_sub_list:
      # stdscr.clear()
      stdscr.addstr(2, 0, update_progress.format(combo_name, int(100*count_combo/len(res))))
      stdscr.addstr(3, 0, situation_progress.format(situation.name, int(100*count_situation/len(situations_sub_list))))
      stdscr.clrtoeol()
      stdscr.refresh()
      if not checkCombo(problem_t0, situation, combo):
        vc=False
        break
      count_situation+=1
    if vc:
      # vc_list.append(combo_name)
      updateViableControllers(problem_t0, combo)
    count_combo+=1
  curses.reset_shell_mode()
  return viable_controllers


def createViableControllersCSV(problem_t0, casestudy_dir):
  situations=[s.name for s in problem_t0.situations.values()]
  N=len(situations)
  res=[list(comb) for i in range(1, N+1) for comb in combinations(situations, i)]
  combo_dict={"[]" : [[]]}
  for r in res:
    combo_dict[strKey(r)]=[[]]
  viable_controllers=pd.DataFrame(combo_dict, index=["viable controllers"])
  viable_controllers.to_csv(path_or_buf=f"{casestudy_dir}/build/data/viable_controllers/viable_controllers.csv")
  return viable_controllers


def testFunc(transition_csv_file):
  casestudy_dir=os.getenv("SIM_DIR_PATH")
  weights=((transition_csv_file.split('/')[-1]).split('.csv')[0]).split('_')

  ''' For pre-deployment '''
  if True:
    # Create problem from data in CSV
    viable_controllers_filename=f"{casestudy_dir}/build/data/viable_controllers/viable_controllers.csv"
    problem_t0=createProblemInstance(transition_csv_file)
    if not (os.path.exists(viable_controllers_filename)):
      Path(f"{casestudy_dir}/build/data/viable_controllers").mkdir(parents=True, exist_ok=True)
      viable_controllers=createViableControllersCSV(problem_t0,casestudy_dir)
    else:
      viable_controllers=pd.read_csv(viable_controllers_filename, index_col=0)

    # # Print and save dtmcs
    # if config.SAVE_DTMC_FILES:
    #     problem_t0.save_dtmc_file()
    viable_controllers=wrapper(checkCombos, transition_csv_file, problem_t0.situations, weights, viable_controllers)
    viable_controllers.to_csv(path_or_buf=f"{casestudy_dir}/build/data/viable_controllers/viable_controllers.csv")

  ''' For deployment stage '''
  if False:
    program=run_save()
    test=program.verification_results["Violation"]
    if not(test.any()):
      return True
  return False