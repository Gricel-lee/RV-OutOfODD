import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import matplotlib.animation as animation
import os
import sys
from visualise_utils import *

speed=int(sys.argv[1])
fig, ax, ax_lines, agent_deployed_info, agent_waiting_info, TT=setupAxes(axis_lines=True)

def animate(t):
    if t==0:
        plt.waitforbuttonpress()
    if t%10==0:
        print(f"{t*speed}/{int(TT)}")
    updates=[]

  ## Check if new agents needs to be added
    for key in agent_waiting_info.copy():
        if agent_waiting_info[key][0,11]<=t*speed:
            new_ax_agent=plotOnAx(agent_waiting_info[key], int(agent_waiting_info[key][0,11]), ax)
            agent_deployed_info[key]=[new_ax_agent, agent_waiting_info[key]]
            agent_waiting_info.pop(key)

    for key in agent_deployed_info.copy():
        # print(f"KEY {key}")
        if agent_deployed_info[key][1][-1,11]<t*speed:
            # print("  Removing")
            def removeAgent(agent_obj):
                agent_obj[0].remove()
                agent_obj[2].remove()
                agent_obj[4].remove()
            removeAgent(agent_deployed_info[key][0])
            agent_deployed_info.pop(key)
        else:
            updateAgent(agent_deployed_info[key][0], agent_deployed_info[key][1], t*speed)
line_ani=animation.FuncAnimation(fig, animate, frames=int(TT/speed), interval=16, blit=False, repeat=False)
plt.show()