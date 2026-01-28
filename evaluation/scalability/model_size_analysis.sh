for i in $(seq 5 5 50);
do
    echo "Results for $i situations:"
    touch $SAVE_EVAL_PATH/save_output/$i/model_size.txt
    prism $SAVE_EVAL_PATH/save_output/$i/output_t0/dtmc.prism -const init_situation=1,time_max=50 | grep "States\|Transitions" > $SAVE_EVAL_PATH/save_output/$i/model_size.txt
    echo -e "$(cat $SAVE_EVAL_PATH/save_output/$i/model_size.txt)\n"
done