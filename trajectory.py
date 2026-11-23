import h5py

with h5py.File("trajectory.h5", "r") as f:
    joint_positions = f["observation/robot_state/joint_positions"][:]

print("Shape:", joint_positions.shape)
print("Starting position:", joint_positions[0])
print("Ending position:", joint_positions[-1])

with h5py.File("trajectory.h5", "r") as f:
    timestamps = f["observation/timestamp/robot_state/read_start"][:]
    print(list(f["observation/timestamp/robot_state"].keys()))
    print("Start:", f["observation/timestamp/robot_state/read_start"][:])
    print("End:", f["observation/timestamp/robot_state/read_end"][:])

total_time = timestamps[-1] - timestamps[0]
total_time /= 1000

print(total_time)


joint_elapse = total_time/100 #101 joint positions = 100 intervals in between