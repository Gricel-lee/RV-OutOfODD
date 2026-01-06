# Case study preliminary evaluation

This directory contains the scripts required to reproduce the evaluation process used in the paper. 

## Setup

Assuming you have set up SAVE, first set up the environment variables:

```
export SAVE_INPUT_PATH=/path/to/evaluation/save_setup
export PRISM_DIR_PATH=/path/to/prism_directory
export SAVE_EVAL_PATH=/path/to/evaluation
export PYTHONPATH=$PYTHONPATH:/path/to/SAVE/src
```

## Running SAVE

The following will reproduce the steps for generating dummy controllers and their corresponding transition matrices for scenarios where the number of situations vary from 5 to 50 (increments of 5).

1. ```cd $SAVE_EVAL_PATH/py_scripts```
2. ```python3 feasibility_get_TM.py``` This generates pre-deployment data
3. ```python3 feasibility_get_unsafe_TM.py``` This generates noise for deployment data which is then passed through SAVE to synthesise new controllers



## Scalability