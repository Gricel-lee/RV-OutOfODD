import numpy as np
import sys
from pathlib import Path
import os

def updateLines(lines, lines_temp, vals):
	weights=[]
	for val in vals:
		ind=lines_temp.index(val)
		ind_weight=lines_temp[ind:].index("weight:")+ind

		weight=f"{np.round(np.random.uniform(0,3),1):.2f}"
		print(f"{weight}")
		## TO REMOVE; only to test pipeline
		# weight="0.5"
		lines[ind_weight]=f"{lines[ind_weight][:-1]} {weight}\n"
		weights.append(weight)
	return lines, weights

with open("mass_agent_template.txt", 'r') as f:
	lines_master=f.readlines()
f.close()

lines=lines_master.copy()
lines_temp=[line.strip() for line in lines]

attributes=["agent 0:", "agent 1:", "goal:"]

lines, weights=updateLines(lines, lines_temp, attributes)

weight_dir="".join([str(weight)+"_" for weight in weights])
weight_dir=weight_dir[:-1] # Remove floating '_'

line_start="${SIM_DIR_PATH}"
sim_dir_path=os.environ[line_start[2:-1]]
controller_dir_path=f"{sim_dir_path}/build/data/controller"

Path(f"{controller_dir_path}/{weight_dir}").mkdir(parents=True, exist_ok=True)
results_dir=f"{line_start}/build/data/controller/{weight_dir}"

ind_dir=lines_temp.index("results directory:")
lines[ind_dir]=f"{lines[ind_dir][:-1]} {results_dir}\n"

Path(f"{controller_dir_path}/{weight_dir}").mkdir(parents=True, exist_ok=True)
with open(f"{controller_dir_path}/{weight_dir}/mass_agent_{weight_dir}.yaml", 'w') as f:
	for line in lines:
		f.write(line)
f.close()

# Create setup file
with open("setup_design_template.txt", 'r') as f:
	lines_master=f.readlines()
f.close()

lines_temp=lines_master.copy()
lines_temp=[line.strip() for line in lines_temp]

ind_mass_agent=lines_temp.index("mass agent:")
ind_yaml_file=lines_temp[ind_mass_agent:].index("yaml file:")+ind_mass_agent
filename_start=f"{controller_dir_path}/"+weight_dir+"/mass_agent_"
lines_master[ind_yaml_file]=f"{lines_master[ind_yaml_file][:-1]} {filename_start}{weight_dir}.yaml\n"
Path(f"{sim_dir_path}/build/yaml_files").mkdir(parents=True, exist_ok=True)
with open(f"{sim_dir_path}/build/yaml_files/setup_design.yaml", 'w') as f:
	for line in lines_master:
		f.write(line)