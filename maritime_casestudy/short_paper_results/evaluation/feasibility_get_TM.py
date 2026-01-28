from save_utils import *

predeploy_file_path=os.path.join(os.getenv("SIM_DIR_PATH"),"short_paper_results/transition_matrix_feasibility_pre-deploy_{0}.csv")
deploy_file_path=os.path.join(os.getenv("SIM_DIR_PATH"),"short_paper_results/transition_matrix_feasibility_deploy.csv")
situation_no=5

for i in np.arange(20):
  print(f"Getting TM number {i}...")
  ## Get viable controller
  vc=False
  while not vc:
    generateRandomTM(situation_no, 0, 10000, predeploy_file_path.format(i))
    problem_t0=createProblemInstance(predeploy_file_path.format(i))
    for s in problem_t0.situations.values():
      situation_check=verifyModel(problem_t0, s)
      if not situation_check:
        break
    vc=True