import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

df = pd.read_csv("results.csv")
df_gps = df[df["px_gps"] != 0]

fig = plt.figure(figsize=(14, 5))

ax1 = fig.add_subplot(131, projection='3d')
ax1.plot(df["px_true"], df["py_true"], df["pz_true"], 'k-', label="True", linewidth=2)
ax1.plot(df["px_est"],  df["py_est"],  df["pz_est"],  'b-', label="EKF",  linewidth=1.5)
ax1.scatter(df_gps["px_gps"], df_gps["py_gps"], df_gps["pz_gps"], c='r', s=5, label="GPS")
ax1.set_title("3D trajectory")
ax1.legend()

ax2 = fig.add_subplot(132)
err = ((df["px_est"]-df["px_true"])**2 + (df["py_est"]-df["py_true"])**2 + (df["pz_est"]-df["pz_true"])**2)**0.5
ax2.plot(df["t"], err, 'b-')
ax2.set_title("Position error (m)")
ax2.set_xlabel("t (s)")
ax2.grid(True)

ax3 = fig.add_subplot(133)
ax3.plot(df["px_true"], df["py_true"], 'k-', label="True", linewidth=2)
ax3.plot(df["px_est"],  df["py_est"],  'b-', label="EKF",  linewidth=1.5)
ax3.scatter(df_gps["px_gps"], df_gps["py_gps"], c='r', s=5, label="GPS")
ax3.set_title("Top view (x-y)")
ax3.set_aspect('equal')
ax3.legend()
ax3.grid(True)

plt.tight_layout()
plt.savefig("results.png", dpi=150)
plt.show()
print("Plot saved in results.png")
