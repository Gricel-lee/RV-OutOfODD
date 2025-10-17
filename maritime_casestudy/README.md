## QUICK NOTES

Install and setup
- Clone box2d repo
- Set BOX2D_ROOT_DIR environment variable
- mkkdir build && cd build && cmake -DBOX2D_ROOT_DIR=$BOX2D_ROOT_DIR ../
- make
- export SIM_DIR_PATH=/path/to/colregs_sim/
- run scripts/run_simulation.sh as test
- YAML info

Developing code
- places to look
	- mass_agent.cpp...
	- normal_agent.cpp...

Analysing simulation
- to visualise results run scripts/visualise X Y
- to create video script/...
- to get analysis results...
