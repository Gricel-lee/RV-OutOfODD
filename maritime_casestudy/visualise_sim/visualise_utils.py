import distinctipy
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

current_colour_ind=0
agent_types={}
agent_colours=distinctipy.get_colors(20)
# availble_colours=distinctipy.get_colors(20)
# agent_colours={}



def getInd(filename):
    fn=filename.split('.')[0]
    fn=fn.split('_')[1]
    return int(fn)


def getColour(x_type):
    # global current_colour_ind
    # if x_type not in agent_colours:
    #     agent_colours[x_type]=availble_colours[current_colour_ind]
    #     current_colour_ind=current_colour_ind+1
    # print(x_type, agent_colours)
    # print(x_type)
    return agent_colours[int(x_type)]


def updateAgent(agent_ax, agent, t):
    start_time=int(agent[0,11])
    t=t-start_time
    r_radius=agent_ax[1]
    agent_ax[0].center=agent[t,0],agent[t,1]
    agent_ax[2].center=agent[t,0],agent[t,1]
    # pt_x=(1.2*r_radius)*np.cos(agent[t,2]*np.pi/180.0)
    # pt_y=(1.2*r_radius)*np.sin(agent[t,2]*np.pi/180.0)
    heading_x=agent[t,0]
    heading_y=agent[t,1]-(0.1*r_radius)*0.5
    agent_ax[4].set_xy([heading_x, heading_y])
    agent_ax[4].set_angle(agent[t,2]*180.0/np.pi)
    return agent_ax[0], agent_ax[2], agent_ax[4]

def plotOnAx(values, t, ax_x, colour=None):
    start_time=int(values[0,11])
    t=t-start_time
    r_radius=values[t,3]
    rng_radius_small=values[t,4]

    ## Setting up robot
    r_type=int(values[t,-1])
    r_colour=getColour(r_type) if colour is None else colour[r_type]

    rbt=plt.Circle(values[t,0:2], radius=r_radius, fc=r_colour, alpha=1.0, zorder=10)
    rbt=ax_x.add_patch(rbt)
    sns_colour=0.7*np.ones(3)
    rbt_sns=plt.Circle(values[t,0:2], radius=rng_radius_small, fc=sns_colour, alpha=0.2, zorder=3)
    rbt_sns=ax_x.add_patch(rbt_sns)
    rbt_angle=(values[t,2])
    rbt_lnx=(1.2*r_radius)*np.cos(values[t,2])
    rbt_lny=(1.2*r_radius)*np.sin(values[t,2])

    ## Local line (heading direction)
    # rbt_ln,=ax_x.plot([values[t,0],values[t,0]+rbt_lnx], [values[t,1], values[t,1]+rbt_lny], linewidth=r_radius*3, c=[0,0,0], zorder=20)
    # agent=[rbt, r_radius, rbt_sns, rbt_angle, rbt_lnx, rbt_lny, rbt_ln]
    heading_length=r_radius*1.2
    width_scale=0.2 # cover 10% of agent
    heading_width=r_radius*width_scale
    bottom_left_corner=values[t,0:2]
    bottom_left_corner[1]-=width_scale*0.5
    rbt_heading=plt.Rectangle(bottom_left_corner,r_radius*1.2,r_radius*0.1,angle=rbt_angle,fc=[0,0,0], zorder=15)
    ax_x.add_patch(rbt_heading)
    agent=[rbt, r_radius, rbt_sns, rbt_angle, rbt_heading]
    return agent

def getAllAgentInfo():
    results_dir='../results'
    files=os.listdir(results_dir)
    files=[file for file in files if file[-4:]=='.txt']
    goals={}
    goal_ind=0

    max_vals=[]

    agent_info={}

    TT=0

    for file in files:
        results=np.loadtxt(f'{results_dir}/{file}')
        goal=results[0,7:11]
        found=False
        for key in goals:
            if (goals[key][0]==goal).all():
                found=True
                break
        if not found:
            goals[goal_ind]=[goal, results[0,-1]]
            goal_ind+=1
        agent_info[file]=results
        max_vals.append([np.max(results[:,0]), np.min(results[:,0]), np.max(results[:,1]), np.min(results[:,1]), int(results[-1,9])])
        if results[-1,11]>TT:
            TT=results[-1,11]
    return agent_info, max_vals, goals, TT

def setUpSimAxesOnly(zoom_mag, goals, max_vals, colour=None):
    fig=plt.figure()
    axs=[]
    gs=fig.add_gridspec(2, 3)
    ax=fig.add_subplot(gs[0,:])
    ax_zoom=fig.add_subplot(gs[1,:])

    buffer=1.5
    x_max=buffer+np.max(np.array(max_vals)[:,0])
    x_min=-buffer+np.min(np.array(max_vals)[:,1])
    y_max=buffer+np.max(np.array(max_vals)[:,2])
    y_min=-buffer+np.min(np.array(max_vals)[:,3])
    TT=np.max(np.array(max_vals)[:,4])

    ## Plot agents's goal locations
    for goal in goals:
        [rect_info, goal_type]=goals[goal]
        goal_type=int(goal_type)
        start_x=rect_info[0]-rect_info[1]
        end_x=rect_info[0]+rect_info[1]
        start_y=rect_info[2]-rect_info[3]
        end_y=rect_info[2]+rect_info[3]

        rect_start_xy=[start_x,start_y]
        width=2*rect_info[1]
        height=2*rect_info[3]
        goal_colour=getColour(goal_type) if colour is None else colour[goal_type]
        temp_goal_area_reg=plt.Rectangle(rect_start_xy,width,height,fc=goal_colour, alpha=0.3, zorder=1)
        temp_goal_area_zoom=plt.Rectangle(rect_start_xy,width,height,fc=goal_colour, alpha=0.3, zorder=1)
        temp_goal_area_lines=plt.Rectangle(rect_start_xy,width,height,fc=goal_colour, alpha=0.3, zorder=1)
        ax.add_patch(temp_goal_area_reg)
        ax_zoom.add_patch(temp_goal_area_zoom)

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

    zoom_mag=1.0/zoom_mag
    ax_zoom.set_xlim([zoom_mag*x_min,zoom_mag*x_max])
    ax_zoom.set_ylim([zoom_mag*y_min,zoom_mag*y_max])
    ax_zoom.set_aspect('equal')

    return fig, ax, ax_zoom

def setupAxes(zoom=False, axis_lines=False, zoom_mag=0.0, bounds=None):
    results_dir='../results'
    files=os.listdir(results_dir)
    files=[file for file in files if file[-4:]=='.txt']
    fig=plt.figure()
    axs=[]
    # print("Value of zoom: ", zoom)
    if zoom:
        if axis_lines:
            gs=fig.add_gridspec(2, 3)
            ax=fig.add_subplot(gs[0,0:2])
            ax_lines=fig.add_subplot(gs[:,2])
            ax_zoom=fig.add_subplot(gs[1,0:2])
        else: 
            gs=fig.add_gridspec(2, 2)
            ax=fig.add_subplot(gs[0,:])
            ax_zoom=fig.add_subplot(gs[1,:])
    else:
        gs=fig.add_gridspec(2, 2)
        if axis_lines:
            ax=fig.add_subplot(gs[:,0])
            ax_lines=fig.add_subplot(gs[:,1])
        else: 
            ax=fig.add_subplot(gs[:,:])

    # output=[]
    # agents=[]
    # agents_zoom=[]

    agent_deployed_info={}
    agent_waiting_info={}

    # new_agents=len(files)/3
    goals={}
    goal_ind=0

    max_vals=[]

    def convertRow(val_in):
        global current_colour_ind
        val=val_in.decode()
        try:
            float(val)
            return float(val)
        except ValueError:
            if val not in agent_types:
                val_new=current_colour_ind
                current_colour_ind+=1
                agent_types[val]=int(val_new)
            return agent_types[val]

    for file in files:
        # with open(f'{results_dir}/{file}') as temp_file:
        #     temp_line=temp_file.readlines()[0].strip().split(' ')
        # no_attribute=len(temp_line)
        # cols=tuple(np.arange(no_attribute))
        # types="f,"*(no_attribute-1)+"U100"
        results=np.loadtxt(f'{results_dir}/{file}', converters=convertRow)
        # results=np.reshape(results, (len(results), len(results[0])))
        goal=results[0,7:11]
        found=False
        for key in goals:
            if (goals[key][0]==goal).all():
                found=True
                break
        if not found:
            goals[goal_ind]=[goal, results[0,-1]]
            goal_ind+=1

        if results[0,11]==0:
            agent=plotOnAx(results, 0, ax)
            # agents.append(agent)

            if zoom:
                agent_zoom=plotOnAx(results,0,ax_zoom)
                # agents_zoom.append(agent_zoom)
                agent_deployed_info[file]=[agent, agent_zoom, results]
            else:
                agent_deployed_info[file]=[agent, results]
        else:
            agent_waiting_info[file]=results
        # output.append(results)
        max_vals.append([np.max(results[:,0]), np.min(results[:,0]), np.max(results[:,1]), np.min(results[:,1]), int(results[-1,11])])
        if axis_lines:
            ax_lines.plot(results[:,0], results[:,1], c=getColour(results[0,-1]), zorder=10)

    if bounds is None:
        buffer=1.5
        # x_max=buffer+np.max(np.array([np.max(arr[:,0]) for arr in output]))
        # x_min=-buffer+np.min(np.array([np.min(arr[:,0]) for arr in output])) 
        # y_max=buffer+np.max(np.array([np.max(arr[:,1]) for arr in output]))
        # y_min=-buffer+np.min(np.array([np.min(arr[:,1]) for arr in output]))
        x_max=buffer+np.max(np.array(max_vals)[:,0])
        x_min=-buffer+np.min(np.array(max_vals)[:,1])
        y_max=buffer+np.max(np.array(max_vals)[:,2])
        y_min=-buffer+np.min(np.array(max_vals)[:,3])
    else:
        x_min=bounds[0,0]
        x_max=bounds[0,1]
        y_min=bounds[1,0]
        y_max=bounds[1,1]
    # print(x_min, x_max)
    TT=np.max(np.array(max_vals)[:,4])

    ## Plot agents's goal locations
    for goal in goals:
        [rect_info, goal_type]=goals[goal]
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
        if axis_lines:
            ax_lines.add_patch(temp_goal_area_lines)
        if zoom:
            ax_zoom.add_patch(temp_goal_area_zoom)

      ## Track max/min positions for plotting
        if bounds is None:
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

    if zoom:
        ax_zoom.set_facecolor(water_background_colour)

    #add rectangle to plot
    ax.set_xlim([x_min,x_max])
    ax.set_ylim([y_min,y_max])
    ax.set_aspect('equal')

    if zoom:
        print(zoom)
        zoom_mag=1.0/zoom_mag
        ax_zoom.set_xlim([zoom_mag*x_min,zoom_mag*x_max])
        ax_zoom.set_ylim([zoom_mag*y_min,zoom_mag*y_max])
        ax_zoom.set_aspect('equal')
        if axis_lines:
            return fig, ax, ax_zoom, ax_lines, agent_deployed_info, agent_waiting_info, TT 
        else:
            return fig, ax, ax_zoom, agent_deployed_info, agent_waiting_info, TT
    elif axis_lines:
        return fig, ax, ax_lines, agent_deployed_info, agent_waiting_info, TT
    else:
        return fig, ax, agent_deployed_info, agent_waiting_info, TT
    # return fig, ax, ax_zoom, ax_lines, agents, agents_zoom, output