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

i=int(sys.argv[1])
trials=20
generateRandomTM(i, 0, 10000, file_path)
problem_t0=createProblemInstance(file_path)