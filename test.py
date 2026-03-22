import numpy as np
from sklearn.linear_model import LinearRegression
from sklearn.tree import DecisionTreeRegressor
from sklearn.metrics import mean_squared_error, mean_absolute_error
from tensorflow import keras
from tensorflow.keras import layers
import matplotlib.pyplot as plt

# Generate random x and y data
np.random.seed(42)
X = np.random.rand(100, 1) * 10  # 100 random x values between 0 and 10
y = 2.5 * X.flatten() + np.random.normal(0, 2, 100)  # y = 2.5x + noise

# ============= Linear Regression Model =============
print("=" * 50)
print("LINEAR REGRESSION MODEL")
print("=" * 50)
lr_model = LinearRegression()
lr_model.fit(X, y)

lr_slope = lr_model.coef_[0]
lr_intercept = lr_model.intercept_
lr_predictions = lr_model.predict(X)
lr_r2 = lr_model.score(X, y)
lr_mse = mean_squared_error(y, lr_predictions)
lr_rmse = np.sqrt(lr_mse)
lr_mae = mean_absolute_error(y, lr_predictions)

print(f"Slope: {lr_slope:.4f}")
print(f"Intercept: {lr_intercept:.4f}")
print(f"R^2 Score: {lr_r2:.4f}")
print(f"MSE: {lr_mse:.4f}")
print(f"RMSE: {lr_rmse:.4f}")
print(f"MAE: {lr_mae:.4f}")

# ============= Decision Tree Model =============
print("\n" + "=" * 50)
print("DECISION TREE MODEL")
print("=" * 50)
dt_model = DecisionTreeRegressor(max_depth=4, random_state=42)
dt_model.fit(X, y)

dt_predictions = dt_model.predict(X)
dt_r2 = dt_model.score(X, y)
dt_mse = mean_squared_error(y, dt_predictions)
dt_rmse = np.sqrt(dt_mse)
dt_mae = mean_absolute_error(y, dt_predictions)

print(f"Max Depth: {dt_model.get_depth()}")
print(f"Leaf Count: {dt_model.get_n_leaves()}")
print(f"R^2 Score: {dt_r2:.4f}")
print(f"MSE: {dt_mse:.4f}")
print(f"RMSE: {dt_rmse:.4f}")
print(f"MAE: {dt_mae:.4f}")

# ============= Neural Network Model =============
print("\n" + "=" * 50)
print("NEURAL NETWORK MODEL")
print("=" * 50)

nn_model = keras.Sequential([
    layers.Input(shape=(1,)),
    layers.Dense(16, activation="relu"),
    layers.Dense(8, activation="relu"),
    layers.Dense(1),
])

nn_model.compile(optimizer="adam", loss="mse", metrics=["mae"])

history = nn_model.fit(X, y, epochs=100, batch_size=8, verbose=0)

nn_predictions = nn_model.predict(X, verbose=0).flatten()
nn_mse = mean_squared_error(y, nn_predictions)
nn_rmse = np.sqrt(nn_mse)
nn_mae = mean_absolute_error(y, nn_predictions)
nn_r2 = 1 - (np.sum((y - nn_predictions) ** 2) / np.sum((y - np.mean(y)) ** 2))

print(f"R^2 Score: {nn_r2:.4f}")
print(f"MSE: {nn_mse:.4f}")
print(f"RMSE: {nn_rmse:.4f}")
print(f"MAE: {nn_mae:.4f}")

# ============= Performance Comparison =============
print("\n" + "=" * 50)
print("PERFORMANCE COMPARISON")
print("=" * 50)

metric_rows = [
    ("R^2 Score", lr_r2, dt_r2, nn_r2, max),
    ("MSE", lr_mse, dt_mse, nn_mse, min),
    ("RMSE", lr_rmse, dt_rmse, nn_rmse, min),
    ("MAE", lr_mae, dt_mae, nn_mae, min),
]

print(f"{'Metric':<15} {'Linear Reg':<15} {'Decision Tree':<15} {'Neural Net':<15} {'Winner':<15}")
print("-" * 78)
for metric, lr_value, dt_value, nn_value, selector in metric_rows:
    best_value = selector(lr_value, dt_value, nn_value)
    winner = "LR" if best_value == lr_value else "DT" if best_value == dt_value else "NN"
    print(f"{metric:<15} {lr_value:<15.4f} {dt_value:<15.4f} {nn_value:<15.4f} {winner:<15}")

# Plot comparison
sort_idx = np.argsort(X.flatten())
X_sorted = X.flatten()[sort_idx]
lr_sorted = lr_predictions[sort_idx]
dt_sorted = dt_predictions[sort_idx]
nn_sorted = nn_predictions[sort_idx]

fig, axes = plt.subplots(1, 3, figsize=(18, 5))

axes[0].scatter(X, y, label="Data points", alpha=0.6)
axes[0].plot(X_sorted, lr_sorted, color="red", linewidth=2, label="Linear Regression")
axes[0].set_xlabel("X")
axes[0].set_ylabel("Y")
axes[0].set_title(f"Linear Regression (R^2 = {lr_r2:.4f})")
axes[0].legend()
axes[0].grid(True, alpha=0.3)

axes[1].scatter(X, y, label="Data points", alpha=0.6)
axes[1].plot(X_sorted, dt_sorted, color="orange", linewidth=2, label="Decision Tree")
axes[1].set_xlabel("X")
axes[1].set_ylabel("Y")
axes[1].set_title(f"Decision Tree (R^2 = {dt_r2:.4f})")
axes[1].legend()
axes[1].grid(True, alpha=0.3)

axes[2].scatter(X, y, label="Data points", alpha=0.6)
axes[2].plot(X_sorted, nn_sorted, color="green", linewidth=2, label="Neural Network")
axes[2].set_xlabel("X")
axes[2].set_ylabel("Y")
axes[2].set_title(f"Neural Network (R^2 = {nn_r2:.4f})")
axes[2].legend()
axes[2].grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig("model_comparison.png", dpi=100)
print("\nComparison plot saved as 'model_comparison.png'")
