# Case study preliminary evaluation

This directory contains the scripts required to reproduce the evaluation process used in the paper. 

## Setup

Assuming you have set up SAVE, first set up the environment variables:

```
export SAVE_INPUT_PATH=/path/to/evaluation/save_setup
export SAVE_EVAL_PATH=/path/to/evaluation
export PYTHONPATH=$PYTHONPATH:/path/to/SAVE/src
```

## Running SAVE

The following will reproduce the steps for generating dummy controllers and their corresponding transition matrices for scenarios where the number of situations vary from 5 to 50 (increments of 5).

```cd $SAVE_EVAL_PATH/py_scripts```

```python3 feasibility_get_TM.py``` This generates pre-deployment data

```python3 feasibility_get_unsafe_TM.py``` This generates noise for deployment data which is then passed through SAVE to synthesise new controllers

The output consists of:

1. The output of SAVE saved as text file: ```evaluation/py_scripts/results.txt```

2. PRISM models stored in ```evaluation/save_output/no_situations/dtmc.prism```

## Scalability

To acquire execution times:

```cd $SAVE_EVAL_PATH/scalability```

```python3 scalability.py``` which will run 20 trials for each number of situations, and store the execution times in ```evaluation/times``` as npz files.


To plot the results:
```python3 plot_scalability.py```


To observe the number of states and transitions:
```source model_size_analysis.sh```