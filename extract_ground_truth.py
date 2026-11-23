from pathlib import Path
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

bag_path = Path('slam_evaluation_dataset')
output_file = 'gt_trajectory.txt'

print("Extracting ground truth to TUM format...")
typestore = get_typestore(Stores.ROS2_JAZZY)

with AnyReader([bag_path], default_typestore=typestore) as reader:
    connections = [x for x in reader.connections if x.topic == '/sim_ground_truth_pose']
    
    with open(output_file, 'w') as f:
        count = 0
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            
            # TUM format: timestamp(seconds) x y z qx qy qz qw
            # msg.header.stamp has sec and nanosec
            t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            pos = msg.pose.pose.position
            ori = msg.pose.pose.orientation
            
            f.write(f"{t_sec:.6f} {pos.x:.4f} {pos.y:.4f} {pos.z:.4f} {ori.x:.4f} {ori.y:.4f} {ori.z:.4f} {ori.w:.4f}\n")
            count += 1

print(f"Successfully exported {count} ground truth poses to '{output_file}'")