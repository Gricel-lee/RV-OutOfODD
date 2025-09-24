# RV-OutOfODD


## Run SAVE

First, set up virtual environment with requirements in ```src/requirements.txt``` in venv.

Second, to run SAVE do:

1. Set paths in src/config.ini

2. Run by activating the virtual environment, 
```
source venv/bin/activate && python --version
cd src/
python3 run_dtmc.py
```


# Notes:

- Situations name in the .csv file must start with "s".

- Failures name in the .csv file must start with "f".