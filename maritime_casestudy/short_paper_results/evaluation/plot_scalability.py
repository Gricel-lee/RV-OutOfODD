import numpy as np
import matplotlib.pyplot as plt
plt.rcParams.update({'font.size': 30})

avg_times_indvd={}
avg_times_total={}

for i in np.arange(5,51,5):
	times=np.load(f"times_{i}.npz")["times"]
	plt.boxplot(times[0], positions=[i], widths=1.7)
	# plt.boxplot(times[1], positions=[i])
plt.ylabel("Number of situations")
plt.xlabel("Model checking time")
plt.show()