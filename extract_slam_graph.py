#!/usr/bin/env python3
import argparse
from pathlib import Path
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

def extract_gt_trajectory(bag_path, output_file, topic_name):
    print(f"Reading bag directory: {bag_path}")

    # Map to store the latest optimized pose for each node ID:
    # node_id -> (timestamp, x, y, z, qx, qy, qz, qw)
    nodes = {}

    typestore = get_typestore(Stores.ROS2_JAZZY)

    with AnyReader([Path(bag_path)], default_typestore=typestore) as reader:
        connections = [x for x in reader.connections if x.topic == topic_name]
        if not connections:
            print(f"Error: Topic '{topic_name}' was not found in the bag file.")
            print(f"Available topics: {[c.topic for c in reader.connections]}")
            return

        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)

            for marker in msg.markers:
                # Filter specifically for slam_toolbox graph node markers (ns: 'slam_toolbox')
                if marker.ns == "slam_toolbox":
                    node_id = marker.id
                    t = marker.header.stamp.sec + marker.header.stamp.nanosec * 1e-9
                    pos = marker.pose.position
                    ori = marker.pose.orientation

                    # Store or overwrite with the latest optimized pose published for this node ID
                    nodes[node_id] = (
                        t,
                        pos.x,
                        pos.y,
                        pos.z,
                        ori.x,
                        ori.y,
                        ori.z,
                        ori.w,
                    )

    if not nodes:
        print(f"No node markers found under 'slam_toolbox' namespace on topic '{topic_name}'.")
        return

    # Sort nodes by ID to maintain keyframe insertion sequence
    sorted_nodes = [nodes[k] for k in sorted(nodes.keys())]

    print(f"Extracted {len(sorted_nodes)} optimized node poses from slam_toolbox.")

    # Write TUM format trajectory
    with open(output_file, "w") as f:
        for t, x, y, z, qx, qy, qz, qw in sorted_nodes:
            f.write(f"{t:.6f} {x:.4f} {y:.4f} {z:.4f} {qx:.4f} {qy:.4f} {qz:.4f} {qw:.4f}\n")

    print(f"Saved ground truth trajectory to: {output_file}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Extract slam_toolbox graph nodes from ROS 2 bag into TUM format."
    )
    parser.add_argument("--bag", type=str, required=True, help="Path to rosbag2 folder")
    parser.add_argument("--output", type=str, default="slam_toolbox_trajectory.txt", help="Output file path")
    parser.add_argument(
        "--topic",
        type=str,
        default="/slam_toolbox/graph_visualization",
        help="Topic name",
    )

    args = parser.parse_args()
    extract_gt_trajectory(args.bag, args.output, args.topic)
