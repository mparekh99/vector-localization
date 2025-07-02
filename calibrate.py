# GOTTEN FROM CHATPGT!!!


from sklearn.linear_model import LinearRegression
import numpy as np
import joblib  


# Training part----- Manually got these y values from testing distances 
# for calibration 
reported_Y = np.array([[93.39], [50], [-8.6], [-69]])
real_Y = np.array([63.5, 100, 150, 200])
model = LinearRegression().fit(reported_Y, real_Y)

# Save the model 
joblib.dump(model, 'dist_calibrated.pkl')
print("Model saved!")
