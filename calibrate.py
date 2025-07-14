import numpy as np
from sklearn.linear_model import LinearRegression

# Marker A: [0, 200] - Raw ArUco X vs real-world Y
raw_markerA = np.array([271.96, 270.6, 267.85, 266.1, 264.13, 261.9, 260.05, 257.5, 254.43, 253.43, 251.2, 248.5, 246.7]).reshape(-1, 1)
real_Y_A = np.array([0, 10.5, 22.2, 34.9, 47.9, 60.1, 73.0, 85.7, 98.4, 111.1, 123.8, 136.5, 149.2])

model_A = LinearRegression().fit(raw_markerA, real_Y_A)
alpha_A = model_A.coef_[0]
beta_A = model_A.intercept_

# Marker B: [200, 0] - Raw ArUco Y vs real-world X
raw_markerB = np.array([-292.5, -291.7, -291.3, -288.66, -287.12, -284.1, -282.6, -280.01, -277.4, -275.11, -272.5, -270.1, -266.75]).reshape(-1, 1)
real_X_B = np.array([0, 10.5, 22.2, 34.9, 47.9, 60.1, 73.0, 85.7, 98.4, 111.1, 123.8, 136.5, 149.2])

model_B = LinearRegression().fit(raw_markerB, real_X_B)
alpha_B = model_B.coef_[0]
beta_B = model_B.intercept_

# Marker C: [-200, 0] - Raw ArUco Y vs real-world X
raw_markerC = np.array([298.6, 297.6, 297.2, 295.0, 292.36, 291.0, 288.5, 286.45, 286.05, 283.26, 291.1, 279.4, 277.35]).reshape(-1, 1)
real_X_C = np.array([0, -10.5, -22.2, -34.9, -47.9, -60.1, -73.0, -85.7, -98.4, -111.1, -123.8, -136.5, -149.2])

model_C = LinearRegression().fit(raw_markerC, real_X_C)
alpha_C = model_C.coef_[0]
beta_C = model_C.intercept_

print("Marker A [0, 200] → real_Y = {:.4f} * raw_X + {:.4f}".format(alpha_A, beta_A))
print("Marker B [200, 0] → real_X = {:.4f} * raw_Y + {:.4f}".format(alpha_B, beta_B))
print("Marker C [-200, 0] → real_X = {:.4f} * raw_Y + {:.4f}".format(alpha_C, beta_C))
