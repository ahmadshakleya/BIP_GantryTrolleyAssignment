import os
from animation import readMat
import pandas as pd
import numpy as np

# Set the root directory and change to the required directory
root_dir = "/home/ahmadshakleya/Documents/Modelica/Model"
os.chdir(root_dir)
os.system("ls")

os.chdir("BIP_CandC_PACKAGE.GantryTrolley_TrolleyLockSimulation")

# Function to run a single simulation
def single_simulation(dp):
    simulation_command = f"./GantryTrolley_TrolleyLockSimulation -override gantryTrolley_TrolleyLock.dp={dp}"
    os.system(simulation_command)
    [names, data] = readMat("GantryTrolley_TrolleyLockSimulation_res.mat")
    return [names, data]

# Run a test simulation and print results
[names, data] = single_simulation(1)
print(names)
print(data)

x_index = names.index("gantryTrolley_TrolleyLock.theta")
x_array = data[x_index]
print(x_array)
print(type(x_array))

# Load calibration data
df = pd.read_csv("../calibration_data_d_p.csv", header=0, names=['theta_array (angular position)'])
print(df.iloc[:, 0])
measured_values = df.iloc[:, 0].to_numpy()

# Function to calculate the sum of squared errors (SSE)
def calculate_sse(x_array, measured_values):
    sse = np.sum((x_array - measured_values) ** 2)
    return sse

# Calculate SSE for a range of dc values
sse_dp_list = []

for i in np.arange(0, 5, 0.01):
    print(i)
    [names, data] = single_simulation(i)
    x_index = names.index("gantryTrolley_TrolleyLock.theta")
    x_array = data[x_index]
    sse = calculate_sse(x_array, measured_values)
    sse_dp_list.append(sse)

# Output results
print(sse_dp_list)
min_val = np.min(sse_dp_list)
optimal_index = sse_dp_list.index(min_val)
optimal_dp = 0 + optimal_index * 0.01

print(f"Minimum SSE: {min_val}")
print(f"Optimal Dp index: {optimal_index}")
print(f"Optimal Dp value: {optimal_dp}")
print(f"SSE for Dp=4.79: {sse_dp_list[optimal_index]}")  # Assuming 479 corresponds to DC=4.79
