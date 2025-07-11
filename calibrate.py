import pandas as pd
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression

# Load the CSV with no header
df = pd.read_csv('circle_marker_data.csv', header=None, names=['x', 'y'])

# Print the first few rows to verify
print(df.head())

# Extract features and target
X = df[['x']]
y = df['y']

# Fit linear regression model
model = LinearRegression()
model.fit(X, y)

# Get the slope and intercept
slope = model.coef_[0]
intercept = model.intercept_
print(f"Best-fit line: y = {slope:.4f}x + {intercept:.4f}")

# Predict for plotting
y_pred = model.predict(X)

# Plot
plt.figure(figsize=(10, 6))
plt.scatter(X, y, color='blue', label='Data points')
plt.plot(X, y_pred, color='red', linewidth=2, label='Best-fit line')
plt.xlabel('x')
plt.ylabel('y')
plt.title('Linear Regression Fit')
plt.legend()
plt.grid(True)
plt.show()
