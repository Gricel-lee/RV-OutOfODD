from save_utils import *
from copy import deepcopy

predeploy_file_path=os.path.join(os.getenv("SIM_DIR_PATH"),"short_paper_results/transition_matrix_feasibility_pre-deploy_{0}.csv")
deploy_file_path=os.path.join(os.getenv("SIM_DIR_PATH"),"short_paper_results/transition_matrix_feasibility_deploy_{0}.csv")
situation_no=5
odd_situation=1
count_min=0
count_max=100000

for i in np.arange(20):
  ## Load pre-deploy TM
  predeploy_TM=readTransitionMatrix(predeploy_file_path.format(i))
  nvc=False
  deploy_TM=deepcopy(predeploy_TM)
  print(f"Getting unsafe TM for number {i}...")
  while not nvc:
    new_vals=np.random.randint(count_min, count_max, situation_no+2)
    deploy_TM[odd_situation+1]=new_vals/np.sum(new_vals)
    saveTransitionProbs(deploy_TM)
    casestudy_dir=os.getenv("SIM_DIR_PATH")
    output_folder=f"{casestudy_dir}/short_paper_results/save_output"
    csv_file = config.CSV_PATH
    problem_t0 = problem.SAVEProblem(output_folder=f"{output_folder}/t0", csv_path=csv_file)
    # Print and save dtmcs
    if config.SAVE_DTMC_FILES:
        problem_t0.save_dtmc_file()
    for s in problem_t0.situations.values():
      situation_check=verifyModel(problem_t0, s)
      if not situation_check:
        nvc=True
        break
  run_save(output_folder=f"{output_folder}/t0")