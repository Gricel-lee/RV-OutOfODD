import configparser
import os

# Create a parser instance
config = configparser.ConfigParser()
config_file = 'config.ini'
if not os.path.exists(config_file):
	raise FileNotFoundError(f"Configuration file '{config_file}' not found.")
config.read(config_file)

# Note: in the config.ini file
# - the section [PATHS] contains the path to the project
# - the section [PARAMS] contains required parameters

# ----- Read the configuration file config.ini------
try:
	CSV_PATH = os.path.abspath(config['PATHS']['CSV_PATH'])
	PRISM_PATH = os.path.abspath(config['PATHS']['PRISM_PATH'])
	PROPERTIES_PATH = os.path.abspath(config['PATHS']['PROPERTIES_PATH'])
	PROP_BOUND_PATH = os.path.abspath(config['PATHS']['PROP_BOUND_PATH'])
	MAX_PLANNING_ITERATIONS = config.getint('PARAMS', 'MAX_PLANNING_ITERATIONS', fallback=5)
	SAVE_DTMC_FILES = True # Always save DTMC files, needed for PRISM cmd line
	TIME_MAX = config.getint('PARAMS', 'TIME_MAX', fallback=10)
	# Verbose output
	VERBOSE = config.getboolean('PARAMS', 'VERBOSE', fallback=False)
except KeyError as e:
	raise KeyError(f"Missing section/key in config.ini: {e}")



