from save_utils import *
from copy import deepcopy
import sys

save_eval_dir_env="SAVE_EVAL_PATH"

predeploy_file_path=os.path.join(os.getenv(save_eval_dir_env),"transition_probs/transition_matrix_feasibility_pre-deploy_{0}.csv")
deploy_file_path=os.path.join(os.getenv(save_eval_dir_env),"short_paper_results/transition_matrix_feasibility_deploy_{0}.csv")

situation_no=5
# odd_situation=1
count_min=0
count_max=100000
failure_min=5000000
failure_max=10000000
failure_inc=1.5

output=[]
output_str="{0}  {1}  {2}  {3}\n"
for i in np.arange(5,51,5):
  ## Load pre-deploy TM
  predeploy_TM=readTransitionMatrix(predeploy_file_path.format(i))
  nvc=False
  deploy_TM=deepcopy(predeploy_TM)
  print(f"Getting unsafe TM for number {i}...")
  while not nvc:
    number_states=np.random.randint(1,situation_no+1) ## Decide number of unsafe states
    available_states=np.arange(1, situation_no+1)
    dangerous_states=np.random.choice(available_states, size=number_states, replace=False) ## Select states which are unsafe
    # print(dangerous_states)
    for ds in dangerous_states:
      new_vals=np.random.randint(count_min, count_max, situation_no+2)
      if np.random.random()<0.5:
        new_vals[0]=np.random.randint(failure_min, failure_max)
      else:
        new_vals[1]=np.random.randint(failure_min, failure_max)
      deploy_TM[ds+1]=new_vals/np.sum(new_vals)
    saveTransitionProbs(deploy_TM)
    casestudy_dir=os.getenv(save_eval_dir_env)
    output_folder=os.path.join(casestudy_dir,f"save_output/{i}/t0")
    csv_file = config.CSV_PATH
    problem_t0 = problem.SAVEProblem(output_folder=output_folder, csv_path=csv_file)
    # Print and save dtmcs
    if config.SAVE_DTMC_FILES:
        problem_t0.save_dtmc_file()
    for s in problem_t0.situations.values():
      situation_check=verifyModel(problem_t0, s)
      if not situation_check:
        nvc=True
        break
    failure_min*=failure_inc
    failure_max*=failure_inc
  verif_results=problem_t0.get_pmc_results()
  
  violation=verif_results.loc[verif_results['Violation']==True]
  violation_list_str=",".join(list(set([v.split('s=')[-1][:-1] for v in violation['Property'].values])))
  fail_prop=f"[{violation_list_str}]"

  error=np.max(violation['Error'].values)

  problem_t_out=run_save(output_folder=f"{output_folder}/t0")
  save_success=False if any(problem_t_out.verification_results['Violation'].values) else True

  output.append(output_str.format(fail_prop, error, save_success, problem_t_out.ignore_states))

with open("results.txt",'w') as f:
  for o in output:
    f.write(o)