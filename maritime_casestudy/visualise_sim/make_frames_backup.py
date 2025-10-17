import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
# import matplotlib.animation as animation
from multiprocessing import Pool
import tqdm
import istarmap

import distinctipy

import os
import sys

agent_colors=distinctipy.get_colors(20)

def getInd(filename):
    fn=filename.split('.')[0]
    fn=fn.split('_')[1]
    return int(fn)

def getColour(x_type):
    # blue_agent=np.array([0,0,1])
    # green_agent=np.array([0.6,0.6,0])
    # pink_agent=np.array([0.4,0.0,0.5])
    # x_colour=np.array([0,0,0])
    # if x_type==0:
    #     x_colour=blue_agent
    # elif x_type==1:
    #     x_colour=green_agent
    # else:
    #     x_colour=pink_agent
    return agent_colors[int(x_type)]

speed=int(sys.argv[1])

files=os.listdir('results')
# agent_type_thresh=len(files)/2

def plotOnAx(values, t, ax_x):
    r_radius=values[t,3]
    rng_radius_small=values[t,4]

    ## Setting up robot
    # r_colour=np.array([0,0,0]) 
    r_colour=getColour(values[t,-1])

    rbt=plt.Circle(values[t,0:2], radius=r_radius, fc=r_colour, alpha=1.0, zorder=10)
    rbt=ax_x.add_patch(rbt)
    sns_colour=0.7*np.ones(3)
    rbt_sns=plt.Circle(values[t,0:2], radius=rng_radius_small, fc=sns_colour, alpha=0.2, zorder=3)
    rbt_sns=ax_x.add_patch(rbt_sns)
    rbt_angle=(values[t,2]*np.pi/180.0)
    rbt_lnx=(1.2*r_radius)*np.cos(values[t,2]*np.pi/180.0)
    rbt_lny=(1.2*r_radius)*np.sin(values[t,2]*np.pi/180.0)

    ## Local line
    rbt_ln,=ax_x.plot([results[0,0],results[0,0]+rbt_lnx], [results[0,1], results[0,1]+rbt_lny], linewidth=r_radius*3, c=[0,0,0], zorder=20)
    agent=[rbt, r_radius, rbt_sns, rbt_angle, rbt_lnx, rbt_lny, rbt_ln]
    return agent

no_zeros=10

def makeFrame(t, agent_outs, goal_areas):
    fig=plt.figure()
    axs=[]
    gs=fig.add_gridspec(2, 3)
    ax=fig.add_subplot(gs[0,0:2])
    ax_lines=fig.add_subplot(gs[:,2])
    ax_zoom=fig.add_subplot(gs[1,0:2])

    output=[]
    for agent_out in agent_outs:
        agent=plotOnAx(agent_out, t, ax)
        plotOnAx(agent_out, t, ax_zoom)
        output.append(agent_out)
        ax_lines.plot(agent_out[:,0], agent_out[:,1], c=getColour(agent_out[t,-1]))

    buffer=1.5
    x_max=buffer+np.max(np.array([np.max(arr[:,0]) for arr in output]))
    x_min=-buffer+np.min(np.array([np.min(arr[:,0]) for arr in output])) 
    y_max=buffer+np.max(np.array([np.max(arr[:,1]) for arr in output]))
    y_min=-buffer+np.min(np.array([np.min(arr[:,1]) for arr in output]))

    ## Plot agents's goal locations
    for goal in goal_areas:
        [rect_info, goal_type]=goal_areas[goal]
        start_x=rect_info[0]-rect_info[1]
        end_x=rect_info[0]+rect_info[1]
        start_y=rect_info[2]-rect_info[3]
        end_y=rect_info[2]+rect_info[3]

        rect_start_xy=[start_x,start_y]
        width=2*rect_info[1]
        height=2*rect_info[3]
        temp_goal_area_reg=plt.Rectangle(rect_start_xy,width,height,fc=getColour(goal_type), alpha=0.3, zorder=1)
        temp_goal_area_zoom=plt.Rectangle(rect_start_xy,width,height,fc=getColour(goal_type), alpha=0.3, zorder=1)
        temp_goal_area_lines=plt.Rectangle(rect_start_xy,width,height,fc=getColour(goal_type), alpha=0.3, zorder=1)
        ax.add_patch(temp_goal_area_reg)
        ax_zoom.add_patch(temp_goal_area_zoom)
        ax_lines.add_patch(temp_goal_area_lines)

      ## Track max/min positions for plotting
        if start_x<x_min:
            x_min=start_x 
        if end_x>x_max:
            x_max=end_x 
        if start_y<y_min:
            y_min=start_y 
        if end_y>y_max:
            y_max=end_y 

    water_background_colour=np.array([214,239,255])/255.0
    ax.set_facecolor(water_background_colour)
    ax_zoom.set_facecolor(water_background_colour)

    #add rectangle to plot
    ax.set_xlim([x_min,x_max])
    ax.set_ylim([y_min,y_max])
    ax.set_aspect('equal')

    zoom_mag=1.0/float(sys.argv[2])
    ax_zoom.set_xlim([zoom_mag*x_min,zoom_mag*x_max])
    ax_zoom.set_ylim([zoom_mag*y_min,zoom_mag*y_max])
    ax_zoom.set_aspect('equal')

    str_t=str(t)
    char_len=no_zeros-len(str_t)
    for c in np.arange(char_len):
        str_t='0'+str_t

    plt.savefig(f'output_video/temp_image_{str_t}.png', bbox_inches='tight', dpi=500)
    plt.close()

goals={}
goal_ind=0
output=[]
for file in files:
    results=np.loadtxt(f'results/{file}')
    # agent_no=getInd(file)
    # agent_no_col=np.ones((1,len(results)))*agent_no
    # results=np.hstack((results, agent_no_col.T))
    output.append(results)

TT=len(output[0])

frames=np.arange(0,TT,speed)

with Pool(5) as pool:
    x=[(t, output, goals) for t in frames]
    for _ in tqdm.tqdm(pool.istarmap(makeFrame, x),
                       total=len(x)):
        pass