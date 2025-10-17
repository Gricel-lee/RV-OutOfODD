import csv
# import numpy as np

def generateFiles(situation_no, transition_csv, close_state_mod):
  dtmc_file="dtmc\n\n"
  dtmc_file+="const int f1=0;\n"
  for i in range(situation_no):
    dtmc_file+=f"const int s{i+1}={i+1};\n"

  dtmc_file+="\nconst int init_situation;\n"
  dtmc_file+=f"\nconst int max_situations={situation_no};\n\n"

  dtmc_file+="module System\n"
  dtmc_file+="\ts : [0..max_situations] init init_situation;\n\n"

  print("Initial DTMC complete")

  with open(transition_csv, newline='') as csvfile:
    csv_reader=csv.reader(csvfile, delimiter=',')
    count=1
    rows=[row for row in csv_reader]
    for i in range(1,len(rows)):
    # for i in range(1,len(csv_reader)):
        counts=[float(val) for val in rows[i][1:]]
        transition_sum=0
        for count in counts:
          transition_sum+=count
        if transition_sum>0:
          transition_comment=f"\t// Situation: s{i}\n"
          transition_line=f"\t[check_situation] s=s{i} & t=0 & time_close<time_MAX -> "
          for j in range(len(counts)):
            if counts[j]>0:
              prob=counts[j]/transition_sum;
              if j<(len(counts)-1):
                transition_line+=f" {prob}:(s'=s{j+1}) + "
              else:
                transition_line+=f" {prob}:(s'=f1);"
          dtmc_file+=f"{transition_comment}{transition_line}\n\n"
        count+=1
  dtmc_file+="endmodule"

  lines="""
  const int time_MAX;

  module timeClose
      time_close : [0..time_MAX] init 0;
  """
  print("GETTING CLOSE STATES")
  close_states=[]
  for s in range(2,situation_no):
    if s%close_state_mod==2:
      close_states.append(s)
  close_states_str="("
  for s in close_states[:-1]:
    close_states_str+=f"s=s{s} | "
  close_states_str+=f"s=s{close_states[-1]})"
  far_state=close_states_str.replace("|", "&")
  far_state=far_state.replace("=", "!=")
  lines+=f"""
      [monitor_time] t=1 & {close_states_str} & time_close<time_MAX -> 1:(time_close'=time_close+1);
      [monitor_time] t=1 & {far_state} -> 1:(time_close'=0);
  endmodule

  module Turn
      // 0: checking situation
      // 1: check number of situations spent at close distance
      t : [0..1] init 0;
      [check_situation] true -> 1:(t'=1);
      [monitor_time] true -> 1:(t'=0);
  endmodule
"""


  print(dtmc_file+lines)
  return 0