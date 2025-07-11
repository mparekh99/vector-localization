import numpy as np
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression

# Your data
circle_marker = {
    (0,0) : [277,1.3,27],
    (0,50) : [264.3, 0.75, 28.5], 
    (0,100) : [252.7, 0.77, 28],
    (0,136.5): [246.1, 0.74, 29]
}

diamond_marker = {
    (0,0) : [274, 15, 32],
    (-50,0) : [265.7, 15.2, 31],
    (-100,0) : [256, 15, 31],
    (-136.5, 0) : [249.2, 15.3,32.7]
}

def prepare_data(marker_dict, axis='y'):
    # Extract true positions and raw measurements
    true_positions = []
    raw_measurements = []
    
    for pos, raw in marker_dict.items():
        # pos is (x, y) coordinate
        if axis == 'y':
            true_positions.append(pos[1])  # y coordinate (for circles)
        else:
            true_positions.append(pos[0])  # x coordinate (for diamonds)
        
        raw_measurements.append(raw[0])  # raw distance measurement (first element)

    return np.array(raw_measurements).reshape(-1, 1), np.array(true_positions)

def plot_and_fit(raw, true, title):
    # Fit linear regression
    model = LinearRegression()
    model.fit(raw, true)
    
    # Predict on raw data range
    xs = np.linspace(raw.min(), raw.max(), 100).reshape(-1,1)
    preds = model.predict(xs)
    
    plt.scatter(raw, true, color='red', label='Raw Data')
    plt.plot(xs, preds, color='blue', label='Linear Fit')
    plt.xlabel('Raw Measured Distance')
    plt.ylabel('True Position (mm)')
    plt.title(title)
    plt.legend()
    plt.grid(True)
    plt.show()
    
    print(f"{title} Linear Model: True_pos = {model.coef_[0]:.4f} * Raw + {model.intercept_:.4f}")

# Circles (y axis)
circle_raw, circle_true = prepare_data(circle_marker, axis='y')
plot_and_fit(circle_raw, circle_true, "Circle Marker Calibration (Y axis)")

# Diamonds (x axis)
diamond_raw, diamond_true = prepare_data(diamond_marker, axis='x')
plot_and_fit(diamond_raw, diamond_true, "Diamond Marker Calibration (X axis)")
