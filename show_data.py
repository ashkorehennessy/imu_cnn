import matplotlib.pyplot as plt
import numpy as np

plt.ion()
fig, ax = plt.subplots()
processed_data = np.load("dataset/5/5_20250329_060217_38.npy")
while True:
    ax.clear()
    ax.plot(processed_data)
    plt.pause(0.01)