import numpy as np
import matplotlib.pyplot as plt

avg_times_indvd={}
avg_times_total={}

for i in np.arange(1,10):
	times=np.load(f"times_{i}.npz")["times"]
	plt.boxplot(times[0], positions=[i])
	# plt.boxplot(times[1], positions=[i])
plt.show()