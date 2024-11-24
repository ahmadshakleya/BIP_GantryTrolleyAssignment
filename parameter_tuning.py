import os
from animation import readMat
import pandas as pd
import numpy as np

# Set the root directory and change to the required directory
root_dir = "/home/ahmadshakleya/Documents/Modelica/Model"
os.chdir(root_dir)
os.system("ls")

os.chdir("BIP_CandC_PACKAGE.GantryTrolley_PendulumLockSimulation")

# Function to run a single simulation
def single_simulation(dc):
    simulation_command = f"./GantryTrolley_PendulumLockSimulation -override gantryTrolley_PendulumLock.dc={dc}"
    os.system(simulation_command)
    [names, data] = readMat("GantryTrolley_PendulumLockSimulation_res.mat")
    return [names, data]

# Run a test simulation and print results
[names, data] = single_simulation(1)
print(names)
print(data)

x_index = names.index("gantryTrolley_PendulumLock.x")
x_array = data[x_index]
print(x_array)
print(type(x_array))

# Load calibration data
df = pd.read_csv("../calibration_data_d_c.csv", header=0, names=['x_array (position)'])
print(df.iloc[:, 0])
measured_values = df.iloc[:, 0].to_numpy()

# Function to calculate the sum of squared errors (SSE)
def calculate_sse(x_array, measured_values):
    sse = np.sum((x_array - measured_values) ** 2)
    return sse

# Calculate SSE for a range of dc values
sse_dc_list = []

for i in np.arange(0, 5, 0.01):
    print(i)
    [names, data] = single_simulation(i)
    x_index = names.index("gantryTrolley_PendulumLock.x")
    x_array = data[x_index]
    sse = calculate_sse(x_array, measured_values)
    sse_dc_list.append(sse)

# Output results
print(sse_dc_list)
min_val = np.min(sse_dc_list)
optimal_index = sse_dc_list.index(min_val)
optimal_dc = 0 + optimal_index * 0.01

print(f"Minimum SSE: {min_val}")
print(f"Optimal DC index: {optimal_index}")
print(f"Optimal DC value: {optimal_dc}")
print(f"SSE for DC=4.79: {sse_dc_list[479]}")  # Assuming 479 corresponds to DC=4.79
