## QUICK NOTES

### Install and setup
Clone box2d repo anywhere you want
`git clone https://github.com/erincatto/box2d`

Set environment variables:
`export BOX2D_ROOT_DIR=/path/to/box2d/`
`export SIM_DIR_PATH=/path/to/RV-OutOfODD/maritime_casestudy`
`export SAVE_DIR_PATH=/path/to/RV-OutOfODD/SAVE`
`export PRISM_DIR_PATH=/path/to/prism_dir` (this should point to parent of `bin`)
You'll need these set in terminals that you build/run simulations. Add these to bottom of .bashrc file for ease.

Build simulator
`cd $SIM_DIR_PATH`
`mkkdir build && cd build`
`cmake -DBOX2D_ROOT_DIR=$BOX2D_ROOT_DIR ../`
`make`

### Quick notes about the agents
There are two distinct types of agents; COLREGs agents, and MASS agents. There can be different subtypes of these agents, as defined by specific configuration files (see below). All agents have the same location of travelling to a goal location, and once reached will be randomly given a new goal to travel to. 

The COLREGs agents will follow COLREGs for:
1. overtaking
2. oncoming
3. crossing

And will then drive towards centre of goal locations. 


MASS agents are controlled by potential fields (PF). In the config file, you can assign a weight to an agent type (repel), and a weigh to the goal location (attract).

### Config files of interest
The simulator uses a config YAML file for simulation setup, and for each agent included, will point to the respective YAML files for the agent types configuration. 

For interest I recommend looking at template files which is used by a script to execute simulations with different potential field (PF) weights:
- `maritime_casestudy/data_collection/generate_yaml/setup_design_template.txt`
This is for the simulation in general (number of timesteps, agent types invovled, where to find their config files, etc). 
- `maritime_casestudy/data_collection/generate_yaml/mass_agent_template.txt`
The parameters have been set except for the PF weights. These will be generated randomly in the script (see below). 

I'd also look at:
- `maritime_casestudy/yaml_files/agents/agent_a_design.yaml` and `maritime_casestudy/yaml_files/agents/agent_b_design.yaml`
These are the COLREGs agents to be used at pre-deployment. Then for deployment:
- `maritime_casestudy/yaml_files/agents/agent_a_deployment.yaml` and `maritime_casestudy/yaml_files/agents/agent_b_deployment.yaml`
These are currently identical to the design versions, but these should be changed to represent ODD. I recommend increasing min and max sizes, I suspect this should cause the property regarding the MASS agent being too spent being close to the COLREGs vessel to fail. The properties that can be changed to represent ODD are:
- min/max sizes: when an agent of this type is spawned a size is randomly selected with uniform probability within this range
- vel-size ratio:
	- linear: the max linear velocity for this agent type is this constant multiplied by the size
	_"larger vessels of the same type can go faster"_
	- angular: the max angular velocity for this agent type is this constant divided by the size
	_"larger vessels of the same type turns slower"_
- range-size ratio: the perecption range of the agent type is this constant multiplied by the size
_"larger vessels of the same type can see further away"_

Notes: There are some parts of the config files that are not used by the simulator and can be removed (forbidden zones), but there are also parts which while not used needs to be kept in (goal pose). I'd recommend leaving it all in for now, and I can tidy up when back for submission. 

### Running simulator

To run pre-deployment stage
`cd $SIM_DIR_PATH`
`source scripts/data_collection.sh`

This will generate random weights for the PF, and record at each timestep the state transition in `build/data/X_Y_Z/X_Y_Z.csv`

There are six goal locations, organised as a hexagon.

The MASS agent will start at (0,0), and the COLREGs agents' positions will be at random goal locations (not neccessarily one they've initially been tasked of reaching). 

If the MASS agent has a near miss/collision (the threshold determined in MASS YAML template file, see above). If this happens, the MASS agent's position will reset to the origin, and the COLREGs agents' positions will reset to random goal locations. This is to ensure that we do not keep collecting useless collision->collision transitions. 

Once complete, the simulator will then call the SAVE code to verify the controller with every combination of valid and invalid situation. The MASS agent will have two bins for the number of neighbours of each agent type, and two bins for the the distance of the closet agent. In total, this wil be 7 situations, and therefore $2^7$ combinations.

The output will append to a results file in `build/data/viable_controllers`; each combination of valid/invalid situations will now have a list of controllers that satisfy the properties.  

Deployment execution...

### Still to be done
In theory, all pieces are implemented (except for maybe one, see below), but requires a small bit of editing in `maritime_casestudy/src/main.cpp` to call different functions for deployment. List of tasks are:
- Edit `main.cpp` to have two different sim loops, one for pre-deployment (implemented), and the other for deployment (copy-and-past pre-deployment, tweak, and update setup YAML file to call this loop instead)
- Implement function which loads the viable controllers list. (Can copy and paste CSV loading code in `include/mass.h`, lines 112-139 (should be last batch of code in constructor function))
- Implement function which updates transition matrix and calls SAVE. The transition matrix already updates in pre-deployment, but a slight update to favour new data points might be needed (and if so, only requires a slight tweaking in a copied version of the function). function modelCheck() in mass_agent.cpp already calls SAVE, but you might need to update slightly in a copied function how to manipulate the returned results. This function currently imports a python module, which interacts with SAVE, and then stores the returned result. So, for ease, I recommend to adapt the python module which performs slightly different depending whether it's pre-deployment or deployment, and then update the modelCheck() to pass this as an argument to the module. The module can be found
`maritime_casestudy/model_check/model_check.py` 