import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
# import matplotlib.animation as animation
from multiprocessing import Pool
import tqdm
import istarmap
from visualise_utils import *
import distinctipy

import os
import sys

# TT=len(output[0])
start_TT=int(sys.argv[1])
end_TT=int(sys.argv[2])
speed=int(sys.argv[3])

frames=np.arange(start_TT,end_TT,speed)

workers=5
step_size=int((end_TT-start_TT)/workers)
# print(len(frames), len(frames)%workers)
final_t=-(len(frames)%workers) if len(frames)%workers>0 else len(frames)
frames=frames[0:final_t]
# print(len(frames))

bound=250

bounds=np.array([[-bound,bound], [-bound,bound]])

def splitFrames(tt):
    fig, ax, agent_deployed_info, agent_waiting_info, TT=setupAxes(bounds=bounds)
    ax.plot(np.arange(-1000,1000), np.zeros(2000), linestyle='--', c=[0,0,0])
    def makeFrame(t):
      ## Check if new agents needs to be added
        for key in agent_waiting_info.copy():
            if agent_waiting_info[key][0,11]<=t:
                new_ax_agent=plotOnAx(agent_waiting_info[key], int(agent_waiting_info[key][0,11]), ax)
                agent_deployed_info[key]=[new_ax_agent, agent_waiting_info[key]]
                agent_waiting_info.pop(key)

        for key in agent_deployed_info.copy():
            # print(f"KEY {key}")
            if agent_deployed_info[key][1][-1,11]<t:
                # print("  Removing")
                def removeAgent(agent_obj):
                    agent_obj[0].remove()
                    agent_obj[2].remove()
                    agent_obj[4].remove()
                removeAgent(agent_deployed_info[key][0])
                agent_deployed_info.pop(key)
            else:
                updateAgent(agent_deployed_info[key][0], agent_deployed_info[key][1], t)
        str_t=str(t)
        no_zeros=10
        char_len=no_zeros-len(str_t)
        # for c in np.arange(char_len):
        #     str_t='0'+str_t
        str_t="0"*char_len+str_t
        fig.savefig(f'output_video/temp_image_{str_t}.png', bbox_inches='tight', dpi=500)
    
    for i in tt:
        makeFrame(i)
    plt.close(fig)
    return 0

with Pool(workers) as pool:
    x=[(f,) for f in np.reshape(frames, [workers, int(len(frames)/workers)])]
    print(x)
    for _ in tqdm.tqdm(pool.istarmap(splitFrames, x),
                       total=len(x)):
        pass

# makeFrame(100)

# for t in frames:
#     makeFrame(int(t))
#     print(f"Completion:   {t/TT}")

# speed=int(sys.argv[1])
# zoom=float(sys.argv[2])

# agent_colours=distinctipy.get_colors(20)

# # fig, ax, ax_zoom, ax_lines, agent_deployed_info, agent_waiting_info, TT=setupAxes(zoom)

# agent_info, max_vals, goals, TT=getAllAgentInfo()
# # [rect_info, goal_type]=goals[goal]
# goal_coords=np.array([[goals[key][0][0]+goals[key][0][1], goals[key][0][0]-goals[key][0][1], 
#                 goals[key][0][2]+goals[key][0][3], 
#                 goals[key][0][2]-goals[key][0][3]] for key in goals.keys()])
# print(goal_coords)
# max_vals=max_vals[:len(goal_coords)]
# for i in np.arange(len(goal_coords)):
#     max_vals[i][:4]=goal_coords[i]
# # print(max_vals)
# # sys.exit()
# # start_x=rect_info[0]-rect_info[1]
# # end_x=rect_info[0]+rect_info[1]
# # start_y=rect_info[2]-rect_info[3]
# # end_y=rect_info[2]+rect_info[3]


# def makeFrame(t):
#   ## Check if new agents needs to be added
#     fig, ax, ax_zoom=setUpSimAxesOnly(zoom, goals, max_vals, colour=agent_colours)
#     for key in agent_info.copy():
#         # print(key)
#         agent=agent_info[key]
#         time_step=np.where(agent[:,9]==t*speed)
#         if np.size(time_step)!=0:
#             new_ax_agent=plotOnAx(agent, int(agent[time_step[0][0],9]), ax, colour=agent_colours)
#             new_ax_zoom_agent=plotOnAx(agent, int(agent[time_step[0][0],9]), ax_zoom, colour=agent_colours)
#     ax.plot(np.arange(-200,200), np.zeros(400), linestyle='--', c=[0,0,0])
#     ax_zoom.plot(np.arange(-200,200), np.zeros(400), linestyle='--', c=[0,0,0])
#     str_t=str(t)
#     no_zeros=10
#     char_len=no_zeros-len(str_t)
#     for c in np.arange(char_len):
#         str_t='0'+str_t
#     plt.savefig(f'output_video/temp_image_{str_t}.png', bbox_inches='tight', dpi=500)
#     plt.close()
#     return 0


# fig, ax, ax_lines, agent_deployed_info, agent_waiting_info, TT=setupAxes(follow=False)

# def makeFrame(t):
#     # if t==0:
#     #     plt.waitforbuttonpress()
#     # if t%10==0:
#     #     print(f"{t*speed}/{int(TT)}")
#     # updates=[]
#     fig, ax, ax_zoom=setUpSimAxesOnly(zoom, goals, max_vals, colour=agent_colours)
#     for key in agent_info.copy():
#         # print(key)
#         agent=agent_info[key]
#         time_step=np.where(agent[:,9]==t*speed)
#         if np.size(time_step)!=0:
#             new_ax_agent=plotOnAx(agent, int(agent[time_step[0][0],9]), ax, colour=agent_colours)
#             new_ax_zoom_agent=plotOnAx(agent, int(agent[time_step[0][0],9]), ax_zoom, colour=agent_colours)
#     ax.plot(np.arange(-200,200), np.zeros(400), linestyle='--', c=[0,0,0])
#     ax_zoom.plot(np.arange(-200,200), np.zeros(400), linestyle='--', c=[0,0,0])
#     str_t=str(t)
#     no_zeros=10
#     char_len=no_zeros-len(str_t)
#     for c in np.arange(char_len):
#         str_t='0'+str_t
#     plt.savefig(f'output_video/temp_image_{str_t}.png', bbox_inches='tight', dpi=500)
#     plt.close()
  # ## Check if new agents needs to be added
  #   for key in agent_waiting_info.copy():
  #       if agent_waiting_info[key][0,11]<=t*speed:
  #           new_ax_agent=plotOnAx(agent_waiting_info[key], int(agent_waiting_info[key][0,11]), ax)
  #           agent_deployed_info[key]=[new_ax_agent, agent_waiting_info[key]]
  #           agent_waiting_info.pop(key)

  #   for key in agent_deployed_info.copy():
  #       # print(f"KEY {key}")
  #       if agent_deployed_info[key][1][-1,11]<t*speed:
  #           # print("  Removing")
  #           def removeAgent(agent_obj):
  #               agent_obj[0].remove()
  #               agent_obj[2].remove()
  #               agent_obj[4].remove()
  #           removeAgent(agent_deployed_info[key][0])
  #           agent_deployed_info.pop(key)
  #       else:
  #           updateAgent(agent_deployed_info[key][0], agent_deployed_info[key][1], t*speed)
# line_ani=animation.FuncAnimation(fig, animate, frames=int(TT/speed), interval=16, blit=False, repeat=False)
# plt.show()