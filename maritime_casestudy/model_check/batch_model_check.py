import numpy as np
import os
import sys
from model_check import *

csv_file_path=os.getenv("SIM_DIR_PATH")+"/build/data/controller"

weights=os.listdir(f"{csv_file_path}")

for weight in weights[:5]:
	weight_file="_".join([w[:-1] for w in weight.split('_')])
	print(f"Analysing weights  {weights}")
	testFunc(f"{csv_file_path}/{weight}/{weight_file}.csv")
