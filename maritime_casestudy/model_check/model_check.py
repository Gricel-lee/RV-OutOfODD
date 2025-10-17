# import numpy as np
import pandas as pd
from save import *
import csv
import numpy as np
import os
from copy import deepcopy

def readTransitionMatrix(transition_csv_file):
  with open(transition_csv_file, newline='') as csvfile:
      csv_reader=csv.reader(csvfile, delimiter=',')
      rows=[rows[1:] for rows in csv_reader]
  transition_matrix=np.array([[float(val) for val in row] for row in rows[1:]])
  print(transition_matrix)
  transition_sum=np.sum(transition_matrix,axis=1)
  transition_sum_aug=transition_sum+1*(transition_sum<1)
  transition_matrix/=(transition_sum_aug[:,None]);
  return transition_matrix


def saveTransitionProbs(transition_matrix):
  # transition_matrix=np.round(transition_matrix,decimals=4)
  save_dir_path=os.getenv("SAVE_DIR_PATH")
  f=open(f"{save_dir_path}/example_maritime/input/coverageGrid.csv",'w')
  first_row=f"Situation,Next,Probability\n"
  f.write(first_row)
  for i in np.arange(1,len(transition_matrix)):
    for j in np.arange(len(transition_matrix)):
      p=transition_matrix[i][j]
      next_state=f"s{j}" if j>0 else f"f1"
      line=f"s{i},{next_state},{p}\n"
      f.write(line)
  f.close()
  return


def testFunc(transition_csv_file):
  transition_matrix=readTransitionMatrix(transition_csv_file)
  saveTransitionProbs(transition_matrix)
  program=run_save()
  test=program.verification_results["Violation"]
  print(test)
  if not(test.any()):
    return True
  return False #len(program.failures)