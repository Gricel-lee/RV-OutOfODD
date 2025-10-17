export PYTHONPATH=":$SEAMS_DIR_PATH/model_check/:$SAVE_DIR_PATH/src/:"
# for i in $(seq 1 10);
# do

  # PRE-DEPLOYMENT PHASE
  cd $SEAMS_DIR_PATH/data_collection/generate_yaml
  python3 generate_yaml.py

  ## LOOP THROUGH ALL SETUPS
  cd $SEAMS_DIR_PATH/build

  # Run simulation
  ./test $SEAMS_DIR_PATH/build/yaml_files/setup_design.yaml
  cd $SEAMS_DIR_PATH
  ## UPDATE CSV FILES
    ##  TODO: function to update csv files

# done
# python3 INTERPRET_AND_ASSIGN_CONTROLLER.py ## A collection of controllers and which combination of situations they are valid for  
# cd ../

# # DEPLOYMENT PHASE
# ./test ../data_collection/yaml_files/setup_design.yaml ## Where mass controller is chosen from list of controllers from design phase
# 													   ## Now utilises SAVE, i.e. updates transitions on fly, performs model checking, update controller as needed
