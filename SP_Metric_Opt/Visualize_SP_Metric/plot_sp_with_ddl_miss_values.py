import numpy as np
import matplotlib.pyplot as plt

# Define the Python equivalents of the functions

def penalty_func(violate_probability, threshold):
    return -0.01 * np.exp(10 * abs(threshold - violate_probability))

def reward_func(violate_probability, threshold):
    return np.log((threshold - violate_probability) + 1)

def interpolate(x, x1, y1, x2, y2):
    return y1 + (x - x1) * (y2 - y1) / (x2 - x1)

def sp_func(violate_probability, threshold):
    min_val = penalty_func(1, threshold)
    max_val = reward_func(0, threshold)
    if threshold >= violate_probability:
        val = reward_func(violate_probability, threshold)
    else:
        val = penalty_func(violate_probability, threshold)
    return interpolate(val, min_val, 0, max_val, 1)

# Define parameters
threshold = 0.5
violate_probabilities = np.linspace(0, 1, 500)

# Compute SP values
sp_values = [sp_func(vp, threshold) for vp in violate_probabilities]

# Plot the SP function
plt.figure(figsize=(8, 6))
plt.plot(violate_probabilities, sp_values, label="SP Function", color="blue")
plt.title("SP Function vs Violate Probability")
plt.xlabel("Violate Probability")
plt.ylabel("SP Function Value")
plt.axvline(x=threshold, color="red", linestyle="--", label="Threshold")
plt.legend()
plt.grid(linestyle='dashed')
plt.savefig("SP_threshold_plot.pdf")
plt.show()
