import cvxpy as cp
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import os

data_file = os.path.join(
    os.path.expanduser("~"),
    "cvy_ws/src/convoy_ros/data_analysis/data/vel_io_data.csv",
)
data = pd.read_csv(data_file)
t = data["time"].to_numpy()
u = data["desired speed"].to_numpy()
y = data["speed"].to_numpy()

y_true = y[1:]
X = np.array([[y[k], y[k], u[k]] for k in range(len(t) - 1)])
n = X.shape[1]
theta = cp.Variable(n)
cost = cp.sum_squares(y_true - X @ theta)
constraints = [theta[0] == 1, theta[1] == -theta[2]]
prob = cp.Problem(cp.Minimize(cost), constraints)
prob.solve()

dt_over_tau = theta[2].value
dt = 0.02
tau = dt / dt_over_tau

a = 1 - dt / tau
b = dt / tau
print(f"tau = {tau}")
print(f"v_new = {a}v + {b}u")

# simulate the system using some dt
v0 = 0.0
dt_new = 0.02
t_new = t[:: round(dt_new / dt)]
print(t_new)
u_new = u[:: round(dt_new / dt)]
v_pred = np.zeros_like(t_new)
v_pred[0] = v0
for k in range(0, len(t_new) - 1):
    v_pred[k + 1] = (1 - dt_new / tau) * v_pred[k] + dt_new / tau * u_new[k]

plt.figure(figsize=(10, 8))
plt.plot(t, u, color="k", linewidth=2, label="desired speed", alpha=0.5)
plt.plot(t[1:], y_true, linewidth=0.75, label="measured speed")
plt.plot(t_new, v_pred, label=f"model prediction, dt={dt}")
plt.xlabel("time [s]")
plt.ylabel("speed [m/s]")
plt.grid()
plt.legend(bbox_to_anchor=(1.0, 0.5), loc="center left")
plt.title(
    rf"Desired speed vs measured speed vs model prediction ($\tau$ = {tau:.3f} s)"
)
plt.subplots_adjust(0.07, 0.08, 0.75, 0.96, 0.2, 0.2)
plt.show()
