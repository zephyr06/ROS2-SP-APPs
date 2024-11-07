import copy

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import LinearRegression
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import MinMaxScaler
from sklearn.linear_model import LogisticRegression
from scipy.optimize import curve_fit
from collect_tsp_time_perf_data import *


# Folder containing the text files
data_folder = "."
output_csv = "/home/zephyr/Programming/ROS2-SP-APPs/SP_Metric_Opt/applications/tsp_solver_osm/experiments/average_performance.csv"
average_performance = calculate_average_performance(data_folder, output_csv)
print(average_performance)
# Read the CSV file
data = pd.read_csv(output_csv, sep=';')
# Replace 'inf' values with NaN
data.replace([np.inf, -np.inf], np.nan, inplace=True)
# Drop rows that contain NaN (which were 'inf' originally)
data.dropna(inplace=True)
data_org = copy.deepcopy(data)

# Extract CPU time and path cost
data[['Time']] = data[['Time']] * 1000.0
data['path_cost'] = max(data['path_cost']) - data['path_cost']
X = data[['Time']]
y = data['path_cost']
print("CPU time:")
print(np.array(X['Time']))

# Normalize path cost
scaler = MinMaxScaler(feature_range=(0.1, 1))
y_normalized = scaler.fit_transform(data['path_cost'].values.reshape(-1, 1))
print("Normalized performance:")
print(np.array(y_normalized).flatten())

data['path_cost'] = y_normalized

# Define polynomial regression model
degree = 3  # You can adjust the degree of the polynomial
model = make_pipeline(PolynomialFeatures(degree), LinearRegression())
#
# # Fit polynomial regression model
model.fit(X, y_normalized)
# Predict path cost for original data points
y_predicted_normalized = model.predict(X)
print(model)
print(model[1].coef_)



# Inverse transform to get original scale
# y_predicted = scaler.inverse_transform(y_predicted_normalized)

# Plot the original data points
plt.scatter(data['Time'], data['path_cost'], color='blue', label='Original Data')

# Sort the values for plotting the regression curve
sort_axis = np.argsort(X.values.flatten())
plt.plot(X.values[sort_axis], y_predicted_normalized[sort_axis], color='red', label='Polynomial Regression Curve')

# Add labels and title
plt.xlabel('CPU Time')
plt.ylabel('Path Cost')
plt.title('Polynomial Regression Curve with Original Data Points')

# Add legend
plt.legend()

# Show plot
# plt.show()


for i in range(len(y_normalized)):
    print(i, ":", X['Time'][2+i], data_org['path_cost'][2+i], y_normalized[i])