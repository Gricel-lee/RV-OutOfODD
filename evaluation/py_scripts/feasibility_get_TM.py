from save_utils import *

save_eval_dir_env="SAVE_EVAL_PATH"

predeploy_file_path=os.path.join(os.getenv(save_eval_dir_env),"transition_probs/transition_matrix_feasibility_pre-deploy_{0}.csv")

for i in np.arange(5,51,5):
  print(f"Getting TM number {i}...")
  ## Get viable controller
  vc=False
  while not vc:
    generateRandomTM(i, 0, 10000, predeploy_file_path.format(i))
    problem_t0=createProblemInstance(predeploy_file_path.format(i), file_id=i)
    for s in problem_t0.situations.values():
      situation_check=verifyModel(problem_t0, s)
      if not situation_check:
        break
    vc=True