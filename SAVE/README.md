<<<<<<< HEAD
# RV-OutOfODD

The repo is divided into two directories:

1. ```SAVE``` contains the code required for executing the process, and instructions for setup, including guidance for your own application. These can be found in ```SAVE/README.md```

<<<<<<<< HEAD:SAVE/README.md
=======
# Run SAVE

>>>>>>> 2cb06d873ee7a8dc65b9c1585ebf488ee46e0209
First, set up virtual environment with requirements in ```src/requirements.txt``` in venv.

Second, to run SAVE do:

1. Set paths in src/config.ini (no longer needed, see `../maritime_casestudy/README.md`)

<<<<<<< HEAD
2. Run by activating the virtual environment, 
```
source venv/bin/activate && python --version
=======
2. Run by activating the virtual environment, and setting the environment variables 
```
source venv/bin/activate && python --version
export SAVE_DIR_PATH=/path/to/SAVE/src
export SAVE_INPUT_PATH=/path/to/input_dir_containing_required_files
export PRISM_DIR_PATH=/path/to/prism_directory
>>>>>>> 2cb06d873ee7a8dc65b9c1585ebf488ee46e0209
cd src/
python3 main.py
```

<<<<<<< HEAD

=======
>>>>>>> 2cb06d873ee7a8dc65b9c1585ebf488ee46e0209
# Notes:

- Situations name in the .csv file must start with "s".

<<<<<<< HEAD
- Failures name in the .csv file must start with "f".
========
2. ```evaluation``` contains scripts for generating usable data as described in the paper. This includes pre-deployment and deployment transition matrices, along with generating PRISM models and a plot for execution times. 
>>>>>>>> 2cb06d873ee7a8dc65b9c1585ebf488ee46e0209:README.md
=======
- Failures name in the .csv file must start with "f".
>>>>>>> 2cb06d873ee7a8dc65b9c1585ebf488ee46e0209
