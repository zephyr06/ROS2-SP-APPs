import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import LinearRegression
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import MinMaxScaler
from sklearn.linear_model import LogisticRegression
from scipy.optimize import curve_fit

# Read the CSV file
data = pd.read_csv("experiments/2024-06-05-11-48-21_command_history.csv", sep=';')

# Extract CPU time and path cost
X = data[['CPU_time']]
y = data['path_cost']
data['path_cost'] = max(data['path_cost']) - data['path_cost']

# Normalize path cost
scaler = MinMaxScaler(feature_range=(0, 1))
y_normalized = scaler.fit_transform(data['path_cost'].values.reshape(-1, 1))
data['path_cost'] = y_normalized

# Define polynomial regression model
degree = 10  # You can adjust the degree of the polynomial
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
plt.scatter(data['CPU_time'], data['path_cost'], color='blue', label='Original Data')

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
plt.show()