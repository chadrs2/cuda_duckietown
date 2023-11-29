import numpy as np
import matplotlib.pyplot as plt

nthreads = np.array([1,2,4,8,12,16,20])
compute_runtimes = []
start_reading = False
with open("run_log.txt") as f:
    for line in f:
        compute_line = False
        grab_next_word = False
        for w in line.split():
            if w == 'Max':
                start_reading = True
            elif start_reading:
                if w == "[Compute]":
                    compute_line = True            
                elif w == "time:" and compute_line:
                    grab_next_word = True
                elif compute_line and grab_next_word:
                    compute_runtimes.append(float(w))
                    grab_next_word = False

# print(compute_runtimes)
compute_runtimes = np.array(compute_runtimes)


fig, axs = plt.subplots(1,2)
fig.suptitle("HotPlate OpenMP")

axs[0].plot(nthreads, compute_runtimes ,'-^',color='r')
axs[0].set_xlabel("Number of Threads")
axs[0].set_ylabel("Compute Time (ms)")

speed_up = compute_runtimes[0] / compute_runtimes
axs[1].plot(nthreads, speed_up ,'-^',color='r')
axs[1].set_xlabel("Number of Threads")
axs[1].set_ylabel("Speed Up")

plt.show()