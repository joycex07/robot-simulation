import h5py

with h5py.File("trajectory.h5", "r") as f:
    joint_positions = f["observation/robot_state/joint_positions"][:]

print("Shape:", joint_positions.shape)
print("Starting position:", joint_positions[0])
print("Ending position:", joint_positions[-1])