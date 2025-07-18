import pandas as pd
import numpy as np

#CHATGPT

# List of file paths
file_paths = ['data/Front_Transform.csv', 'data/Left_Transform.csv', 'data/Right_Transform.csv']  # Replace with actual filenames

# Dictionary to hold scale factors for each file
scale_factors = {}

def compute_scale_factor(df):
    # Compute mean of raw X and Y
    mean_x = df['RAW_X'].mean()
    mean_y = df['RAW_Y'].mean()
    
    # Compute distance from origin
    distance = np.sqrt(mean_x**2 + mean_y**2)
    
    # Compute scale factor (inverse of mean distance to bring mean closer to origin)
    return 1.0 / distance if distance != 0 else 1.0

# Loop through files and compute scale factor per file
for path in file_paths:
    df = pd.read_csv(path)
    scale_factor = compute_scale_factor(df)
    scale_factors[path] = scale_factor
    print(f"{path}: scale factor = {scale_factor:.6f}")

# Function to scale a raw point using a given file's scale factor
def scale_raw_point(x, y, file_name):
    s = scale_factors.get(file_name, 1.0)
    return x * s, y * s


print(f'THE SCALE FACTORS: {scale_factors}\n')

# Example: scale a new raw point from file2.csv
new_x, new_y = -174.0, -276.0
scaled_x, scaled_y = scale_raw_point(new_x, new_y, 'file2.csv')
print(f"Scaled point: ({scaled_x}, {scaled_y})")
