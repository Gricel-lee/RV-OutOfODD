import pandas as pd
from save import *
import csv
import numpy as np
import os
from pathlib import Path
import time
import matplotlib.pyplot as plt
from save_utils import *

file_path=os.path.join(os.getenv("SIM_DIR_PATH"),"short_paper_results/transition_matrix.csv")

avg_times_indvd={}
avg_times_total={}

trials=20

for i in np.arange(5,51,5):
  print(f"Analysing situation length {i}")
  avg_times_indv_temp=[]
  avg_times_total_temp=[]
  for j in np.arange(trials):
    print(f"\t\t\t\t\t\t\t\t",end='\r')
    print(f"\t-Complete: {int(100*(j/trials))}%",end='\r')
    generateRandomTM(i, 0, 10000, file_path)
    problem_t0=createProblemInstance(file_path)
    s_times=[]
    for s in problem_t0.situations.values():
      start_time=time.time()
      verifyModel(problem_t0, s)
      s_times.append(time.time()-start_time)
    avg_times_indv_temp.append(np.mean(s_times))
    avg_times_total_temp.append(np.sum(s_times))
  # avg_times_indvd[i]=np.mean(avg_times_indv_temp)
  # avg_times_total[i]=np.mean(avg_times_total_temp)
  times_array=np.array([avg_times_indv_temp, avg_times_total_temp])
  np.savez(f'times_{i}',times=times_array)