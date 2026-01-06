# Run SAVE

First, set up virtual environment with requirements in ```src/requirements.txt``` in venv.

Second, to run SAVE do:

1. Set paths in src/config.ini (no longer needed, see `../maritime_casestudy/README.md`)

2. Run by activating the virtual environment, and setting the environment variables 
```
source venv/bin/activate && python --version
export SAVE_DIR_PATH=/path/to/SAVE/src
export SAVE_INPUT_PATH=/path/to/input_dir_containing_required_files
export PRISM_DIR_PATH=/path/to/prism_directory
cd src/
python3 main.py
```

# Notes:

- Situations name in the .csv file must start with "s".

- Failures name in the .csv file must start with "f".