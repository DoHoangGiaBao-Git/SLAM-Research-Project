from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
from scipy.spatial import cKDTree
import argparse

parser = argparse.ArgumentParser(
    description="Getting path data."
)
parser.add_argument("--bag", type=str, required=True, help="Path to bag folder")
parser.add_argument("--topic", type=str, required=True, help="Topic name")
parser.add_argument("--gt", type=str, required=True, help="slam_toolbox path")
parser.add_argument("--slam_custom", type=str, required=True, help="Custome SLAM path")

args = parser.parse_args()
bag_path = Path(args.bag)
odom_topic = args.topic
gt_traj_path = Path(args.gt)
slam_traj_path = Path(args.slam_custom)

odom_x, odom_y = [], []
gt_x, gt_y = [], []
slam_x, slam_y = [], []

print("Reading rosbag and extracting odometry...")
typestore = get_typestore(Stores.ROS2_JAZZY)

# Extract odometry from rosbag
with AnyReader([bag_path], default_typestore=typestore) as reader:
    connections = [x for x in reader.connections if x.topic == odom_topic]
    for connection, timestamp, rawdata in reader.messages(connections=connections):
        msg = reader.deserialize(rawdata, connection.msgtype)
        pos = msg.pose.pose.position
        odom_x.append(pos.x)
        odom_y.append(pos.y)

print(f"Extracted {len(odom_x)} odometry points.")

# Helper to load TUM format trajectories
def load_tum(filepath):
    data = []
    if not Path(filepath).exists():
        return np.array(data)
    with open(filepath, 'r') as f:
        for line in f:
            if line.startswith('#') or not line.strip():
                continue
            parts = [float(x) for x in line.strip().split()]
            if len(parts) >= 8:
                data.append(parts[:8])
    return np.array(data)

gt = load_tum(gt_traj_path)
slam = load_tum(slam_traj_path)

if len(gt) > 0:
    gt_x = gt[:, 1].tolist()
    gt_y = gt[:, 2].tolist()
    print(f"Loaded {len(gt)} ground truth poses.")
else:
    print("Warning: gt_trajectory.txt not found or empty.")

if len(slam) > 0:
    slam_x = slam[:, 1].tolist()
    slam_y = slam[:, 2].tolist()
    print(f"Loaded {len(slam)} custom SLAM keyframe poses.")
else:
    print(f"Warning: {slam_traj_path} not found.")

stats_text = ""

# Compute Spatial APE Metrics (Nearest Geometric Point Matching)
if len(gt) > 0 and len(slam) > 0:
    gt_poses_2d = gt[:, 1:3]   # (N, 2) [x, y]
    slam_poses_2d = slam[:, 1:3] # (M, 2) [x, y]

    # Build spatial KD-Tree from ground truth path
    gt_tree = cKDTree(gt_poses_2d)

    # Find closest ground truth distance for each SLAM pose
    distances, _ = gt_tree.query(slam_poses_2d, k=1)

    errors = np.array(distances)
    rmse = np.sqrt(np.mean(errors**2))
    mean_err = np.mean(errors)
    median_err = np.median(errors)
    max_err = np.max(errors)
    std_err = np.std(errors)

    print("\n--- Corrected SLAM Evaluation Results (Spatial APE) ---")
    print(f"Number of evaluated poses: {len(errors)}")
    print(f"RMSE:   {rmse:.4f} meters")
    print(f"Mean:   {mean_err:.4f} meters")
    print(f"Median: {median_err:.4f} meters")
    print(f"Max:    {max_err:.4f} meters")
    print(f"Std:    {std_err:.4f} meters")

    # Format metrics text box for the plot
    stats_text = (
        f"Spatial APE Metrics (2D):\n"
        f"--------------------------\n"
        f"Evaluated Poses: {len(errors)}\n"
        f"RMSE:   {rmse:.4f} m\n"
        f"Mean:   {mean_err:.4f} m\n"
        f"Median: {median_err:.4f} m\n"
        f"Max:    {max_err:.4f} m\n"
        f"Std:    {std_err:.4f} m"
    )

# Plotting the trajectories
fig, ax = plt.subplots(figsize=(10, 8))

if gt_x and gt_y:
    ax.plot(gt_x, gt_y, label='Ground Truth (slam_toolbox)', color='green', linewidth=2.5, alpha=0.8)
if odom_x and odom_y:
    ax.plot(odom_x, odom_y, label=f'Odometry ({odom_topic})', color='red', linestyle='--', linewidth=1.2, alpha=0.6)
if slam_x and slam_y:
    ax.plot(slam_x, slam_y, label='Custom SLAM Output', color='blue', marker='o', markersize=3, linewidth=1.5)

ax.set_title('TurtleBot 4: SLAM vs Odometry vs Ground Truth')
ax.set_xlabel('X Position (meters)')
ax.set_ylabel('Y Position (meters)')
ax.axhline(0, color='black', linewidth=0.5, linestyle=':')
ax.axvline(0, color='black', linewidth=0.5, linestyle=':')
ax.grid(True, linestyle='--', alpha=0.6)
ax.legend(loc='upper right')
ax.axis('equal')

# Render styled evaluation metrics box in top-left
if stats_text:
    bbox_props = dict(boxstyle='round,pad=0.5', facecolor='white', edgecolor='gray', alpha=0.9)
    ax.text(
        0.03, 0.97, stats_text,
        transform=ax.transAxes,
        fontsize=9,
        verticalalignment='top',
        fontfamily='monospace',
        bbox=bbox_props
    )

output_filename = 'trajectories_comparison_turtlebot4.png'
plt.savefig(output_filename, dpi=300, bbox_inches='tight')
print(f"Plot saved successfully as '{output_filename}'")
plt.show()
