import h5py
import numpy as np
import matplotlib.pyplot as plt

"""
with h5py.File("trajectory.h5", "r") as f:
    gripper_positions = f["observation/robot_state/gripper_position"][:]

print("Shape:", gripper_positions.shape)
print("First 10 values:", gripper_positions[:10])
print("Minimum:", np.min(gripper_positions))
print("Maximum:", np.max(gripper_positions))

plt.plot(gripper_positions)
plt.xlabel("Recorded step")
plt.ylabel("Gripper position")
plt.title("Recorded Gripper Movement")
plt.grid(True)
plt.show()
"""
with h5py.File("trajectory.h5", "r") as f:
    gripper_positions = f["observation/robot_state/gripper_position"][:]
    timestamps = f["observation/timestamp/control/step_start"][:]

time_seconds = (timestamps - timestamps[0]) / 1000.0

plt.plot(time_seconds, gripper_positions)
plt.xlabel("Time (seconds)")
plt.ylabel("Gripper position (units unverified)")
plt.show()

