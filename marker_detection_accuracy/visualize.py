import os
import glob
import csv
import matplotlib.pyplot as plt

def plot_marker_file(csv_filename):
    raw_x, raw_y = [], []
    true_x, true_y = [], []

    with open(csv_filename, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            raw_x.append(float(row['RAW_X']))
            raw_y.append(float(row['RAW_Y']))
            true_x.append(float(row['TRUE_X']))
            true_y.append(float(row['TRUE_Y']))

    plt.figure(figsize=(8,6))
    plt.scatter(raw_x, raw_y, c='red', label='Raw Points', alpha=0.6)
    plt.scatter(true_x, true_y, c='blue', label='True Points', alpha=0.6)

    plt.title(f'Data Plot for {os.path.basename(csv_filename)}')
    plt.xlabel('X position')
    plt.ylabel('Y position')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')  # keep aspect ratio square
    plt.show()


# If you want to plot all CSV files in current folder:
# csv_files = glob.glob("raw_data_readings\*.csv")
csv_files = glob.glob("data/*.csv")

for file in csv_files:
    plot_marker_file(file)
