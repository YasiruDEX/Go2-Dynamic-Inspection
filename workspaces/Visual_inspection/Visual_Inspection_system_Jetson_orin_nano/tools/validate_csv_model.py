import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# ==================================================================
# 1. USER'S FORMULAS (The ones you want to test)
# ==================================================================
def calculate_pan(x, y):
    val = 377.745210
    val += -1.054312534 * (x)
    val += 0.203070967 * (y)
    val += 0.000708651 * (x**2)
    val += -0.000012210 * (x * y)
    val += -0.000874403 * (y**2)
    return val

def calculate_tilt(x, y):
    val = 172.799904
    val += -0.076125862 * (x)
    val += -0.297366972 * (y)
    return val

# ==================================================================
# 2. LOAD DATASET
# ==================================================================
csv_path = "/home/dinethra/Jetson_orin_nano/data/calibration_points_20260210_121251.csv"
print(f"Loading: {csv_path}")

try:
    df = pd.read_csv(csv_path, names=["camera_x", "camera_y", "servo_pan", "servo_tilt"])
    print(f"Loaded {len(df)} points.")
except Exception as e:
    print(f"Error loading file: {e}")
    exit()

# ==================================================================
# 3. VALIDATE (Check Errors)
# ==================================================================
df['pred_pan'] = df.apply(lambda row: calculate_pan(row['camera_x'], row['camera_y']), axis=1)
df['pred_tilt'] = df.apply(lambda row: calculate_tilt(row['camera_x'], row['camera_y']), axis=1)

df['error_pan'] = df['pred_pan'] - df['servo_pan']
df['error_tilt'] = df['pred_tilt'] - df['servo_tilt']

rmse_pan = np.sqrt((df['error_pan'] ** 2).mean())
rmse_tilt = np.sqrt((df['error_tilt'] ** 2).mean())

print("-" * 40)
print(f"VALIDATION RESULTS (Degree 2/1 Formulas)")
print("-" * 40)
print(f"Pan RMSE Error:  {rmse_pan:.2f} degrees")
print(f"Tilt RMSE Error: {rmse_tilt:.2f} degrees")
print("-" * 40)

# ==================================================================
# 4. PLOT (RESTRICTED AREA)
# ==================================================================
def plot_surface_restricted(ax, func, X_data, Y_data, Z_data, label):
    # Find min/max of ACTUAL DATA
    x_min, x_max = X_data.min(), X_data.max()
    y_min, y_max = Y_data.min(), Y_data.max()
    
    # Add a tiny buffer (10 pixels) just to see the edges
    buffer = 10
    xr = np.linspace(x_min - buffer, x_max + buffer, 20)
    yr = np.linspace(y_min - buffer, y_max + buffer, 20)
    XX, YY = np.meshgrid(xr, yr)
    
    # Compute Surface
    ZZ = np.zeros_like(XX)
    for i in range(XX.shape[0]):
        for j in range(XX.shape[1]):
            ZZ[i,j] = func(XX[i,j], YY[i,j])
            
    # Plot Surface
    ax.plot_surface(XX, YY, ZZ, cmap='viridis', alpha=0.6, edgecolor='gray', linewidth=0.2)
    # Plot Scatter
    ax.scatter(X_data, Y_data, Z_data, c='red', s=40, label='Measured', depthshade=False)
    
    ax.set_xlabel('Camera X')
    ax.set_ylabel('Camera Y')
    ax.set_zlabel(label)
    ax.set_title(f"{label} Model (Restricted to Data Area)")

fig = plt.figure(figsize=(14, 6))

# Pan Plot
ax1 = fig.add_subplot(121, projection='3d')
plot_surface_restricted(ax1, calculate_pan, df['camera_x'], df['camera_y'], df['servo_pan'], "Pan")

# Tilt Plot
ax2 = fig.add_subplot(122, projection='3d')
plot_surface_restricted(ax2, calculate_tilt, df['camera_x'], df['camera_y'], df['servo_tilt'], "Tilt")

plt.tight_layout()
print("Displaying Restricted Surface Plot...")
plt.show()
