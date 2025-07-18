import pandas as pd

# Load your CSV
file_path = 'data/Right_Transform.csv'
df = pd.read_csv(file_path)

# Step 1: Compute means
mean_raw_x = df['RAW_X'].mean()
mean_raw_y = df['RAW_Y'].mean()
mean_true_x = df['TRUE_X'].mean()
mean_true_y = df['TRUE_Y'].mean()

# Step 2: Compute translation vector
translation_x = mean_true_x - mean_raw_x
translation_y = mean_true_y - mean_raw_y

print(f"Translation vector: ({translation_x:.4f}, {translation_y:.4f})")

# Step 3: Apply translation to all raw points
df['TRANSLATED_X'] = df['RAW_X'] + translation_x
df['TRANSLATED_Y'] = df['RAW_Y'] + translation_y

# Step 4 (Optional): Save result
df.to_csv('data/Right_Translated.csv', index=False)
