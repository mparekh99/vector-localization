# GOTTEN FROM CHATPGT!!!

from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import LinearRegression
import numpy as np
import joblib  
import matplotlib.pyplot as plt


def plot_fit(measured, true, model, title):
    xs = np.linspace(min(measured), max(measured), 100)
    preds = model.predict(xs.reshape(-1, 1))

    plt.plot(xs, preds, label="Model", color='blue')
    plt.scatter(measured, true, color='red', label="Data")
    plt.title(title)
    plt.xlabel("Measured")
    plt.ylabel("True")
    plt.legend()
    plt.grid(True)
    plt.show()



# Fit a degree-2 polynomial model
def train_model(X, y, degree=2):
    model = make_pipeline(PolynomialFeatures(degree), LinearRegression())
    model.fit(np.array(X).reshape(-1, 1), y)
    return model


# DATA 

# True Mesaured Dist
true_dist = [63.5, 100.0, 150.0, 200.0]


# Circle Marker ~ Y positions
circle_y_measured = [93.39, 50, -8.6, -69]

# Diamond Marker - x axis - negative is closer
diamond_x_measured = [-135.9, -93, -34, 31]

# Hexagon Marker - x axis - positive is closer
hexagon_x_measured = [94, 52, -9, -64]

# Train models
models = {
    'Circles2': train_model(circle_y_measured, true_dist),
    'Diamonds2': train_model(diamond_x_measured, true_dist),
    'Hexagons2': train_model(hexagon_x_measured, true_dist),
}

# Save all models
joblib.dump(models, "pose_calibration_models.pkl")



plot_fit(circle_y_measured, true_dist, models['Circles2'], "Circles2 Y Calibration")
plot_fit(diamond_x_measured, true_dist, models['Diamonds2'], "Diamonds2 X Calibration")
plot_fit(hexagon_x_measured, true_dist, models['Hexagons2'], "Hexagons2 X Calibration")