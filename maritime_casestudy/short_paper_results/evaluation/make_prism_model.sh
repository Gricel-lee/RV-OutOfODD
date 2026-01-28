for i in $(seq 50 50);
do
    # echo $i
	python3 make_prism_model.py $i
	xprism output_t0/dtmc.prism
	# python3 make_prism_model.py 5
done