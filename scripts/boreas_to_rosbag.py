import os
import numpy as np
import pandas as pd
import cv2
import rclpy
from rclpy.serialization import serialize_message
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions
from rosbag2_py import TopicMetadata
from builtin_interfaces.msg import Time
from sensor_msgs.msg import PointCloud2, PointField, Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion, Pose, PoseWithCovariance, TwistWithCovariance
from tf2_msgs.msg import TFMessage
import tf_transformations
from ament_index_python.packages import get_package_share_directory
from collections import defaultdict
import math
from datetime import datetime
from tqdm import tqdm

def euler_to_quaternion(roll, pitch, yaw):
    return tf_transformations.quaternion_from_euler(roll, pitch, yaw)

def timestamp_from_filename(filename):
    # Extract ROS time from filename (assume nanoseconds)
    # stamp_ns = int(os.path.splitext(os.path.basename(filename))[0])
    # sec = stamp_ns // 1_000_000_000
    # nsec = stamp_ns % 1_000_000_000
    stamp_us = int(os.path.splitext(os.path.basename(filename))[0])

    sec = stamp_us // 1_000_000
    nsec = (stamp_us % 1_000_000) * 1000
    return Time(sec=sec, nanosec=nsec)

def get_ros_time_from_timestamp(timestamp):
    sec = int(timestamp // 1e6)
    nanosec = int((timestamp % 1e6) * 1e3)
    return Time(sec=sec, nanosec=nanosec)

def load_lidar_bin(file_path):
    return np.fromfile(file_path, dtype=np.float32).reshape(-1, 6)

def create_pointcloud2(points, timestamp):
    msg = PointCloud2()
    msg.header.frame_id = 'lidar'
    msg.header.stamp = timestamp

    msg.height = 1
    msg.width = points.shape[0]
    msg.is_bigendian = False
    msg.is_dense = True
    msg.point_step = 24  # 6 floats × 4 bytes
    msg.row_step = msg.point_step * points.shape[0]

    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name='ring', offset=16, datatype=PointField.FLOAT32, count=1),
        PointField(name='time', offset=20, datatype=PointField.FLOAT32, count=1),
    ]
    msg.fields = fields
    msg.data = points.astype(np.float32).tobytes()
    return msg

def create_image_msg(image_path, frame_id, timestamp):
    img = cv2.imread(image_path)
    if img is None:
        raise ValueError(f"Failed to read image: {image_path}")
    msg = Image()
    msg.header.frame_id = frame_id
    msg.header.stamp = timestamp
    msg.height, msg.width = img.shape[:2]
    msg.encoding = 'bgr8'
    msg.step = msg.width * 3
    msg.data = img.tobytes()
    return msg

def create_odometry_msg(pose_row, parent_frame , child_frame, stamp):
    odom = Odometry()
    odom.header.stamp = stamp
    odom.header.frame_id = parent_frame  # e.g., 'world'
    odom.child_frame_id = child_frame  # e.g., 'base_link'

    # Position from ENU
    odom.pose.pose.position.x = pose_row['easting']
    odom.pose.pose.position.y = pose_row['northing']
    odom.pose.pose.position.z = pose_row['altitude']

    # Orientation from roll, pitch, heading (yaw)
    roll = pose_row['roll']
    pitch = pose_row['pitch']
    yaw = math.radians(pose_row['heading'])  # convert heading to radians

    q = euler_to_quaternion(roll, pitch, yaw)
    odom.pose.pose.orientation.x = q[0]
    odom.pose.pose.orientation.y = q[1]
    odom.pose.pose.orientation.z = q[2]
    odom.pose.pose.orientation.w = q[3]

    # Linear velocity
    odom.twist.twist.linear.x = pose_row['vel_east']
    odom.twist.twist.linear.y = pose_row['vel_north']
    odom.twist.twist.linear.z = pose_row['vel_up']

    # Angular velocity (already in ENU)
    odom.twist.twist.angular.x = pose_row['angvel_x']
    odom.twist.twist.angular.y = pose_row['angvel_y']
    odom.twist.twist.angular.z = pose_row['angvel_z']

    return odom

def create_tf(pose_row, parent_frame, child_frame, stamp):
    tf = TransformStamped()
    tf.header.stamp = stamp
    tf.header.frame_id = parent_frame  # e.g., 'world'
    tf.child_frame_id = child_frame    # e.g., 'base_link'

    # Translation (position)
    tf.transform.translation.x = pose_row['easting']
    tf.transform.translation.y = pose_row['northing']
    tf.transform.translation.z = pose_row['altitude']

    # Rotation (convert heading from degrees to radians)
    roll = pose_row['roll']
    pitch = pose_row['pitch']
    yaw = math.radians(pose_row['heading'])
    q = euler_to_quaternion(roll, pitch, yaw)

    tf.transform.rotation.x = q[0]
    tf.transform.rotation.y = q[1]
    tf.transform.rotation.z = q[2]
    tf.transform.rotation.w = q[3]

    return tf

def load_static_tf(transform_path, parent_frame, child_frame):
    T = np.loadtxt(transform_path)
    translation = T[:3, 3]
    rotation_matrix = T[:3, :3]
    quat = tf_transformations.quaternion_from_matrix(T)

    tf = TransformStamped()
    tf.header.stamp = Time(sec=0, nanosec=0)
    tf.header.frame_id = parent_frame
    tf.child_frame_id = child_frame
    tf.transform.translation.x = translation[0]
    tf.transform.translation.y = translation[1]
    tf.transform.translation.z = translation[2]
    tf.transform.rotation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
    return tf

def load_inverse_static_tf(transform_path, parent_frame, child_frame):
    # Load original transformation matrix
    T = np.loadtxt(transform_path)

    # Invert the transformation matrix
    T_inv = np.linalg.inv(T)

    # Extract translation and rotation
    translation = T_inv[:3, 3]
    quat = tf_transformations.quaternion_from_matrix(T_inv)

    # Populate TransformStamped message
    tf = TransformStamped()
    tf.header.stamp = Time(sec=0, nanosec=0)
    tf.header.frame_id = parent_frame
    tf.child_frame_id = child_frame
    tf.transform.translation.x = translation[0]
    tf.transform.translation.y = translation[1]
    tf.transform.translation.z = translation[2]
    tf.transform.rotation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
    return tf

def find_closest_timestamp(target_ts, candidates):
    """
    Given a target timestamp and a list/array of candidate timestamps (in float seconds),
    returns the closest candidate and its index.
    """
    candidates = np.array(candidates)
    idx = np.searchsorted(candidates, target_ts)
    if idx == 0:
        return candidates[0], 0
    if idx == len(candidates):
        return candidates[-1], len(candidates) - 1
    before = candidates[idx - 1]
    after = candidates[idx]
    if abs(target_ts - before) < abs(target_ts - after):
        return before, idx - 1
    else:
        return after, idx

def main(base_path, bag_length):
    rclpy.init()
    lidar_folder = os.path.join(base_path, 'lidar')
    radar_folder = os.path.join(base_path, 'radar')
    camera_folder = os.path.join(base_path, 'camera')
    calib_folder = os.path.join(base_path, 'calib')

    # Init rosbag2 writer
    writer = SequentialWriter()
    bag_path = f"bags/boreas_ros2_bag_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    os.makedirs(os.path.dirname(bag_path), exist_ok=True)
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    writer.open(storage_options, converter_options)

    topics = {
        '/lidar': 'sensor_msgs/msg/PointCloud2',
        '/camera/image_raw': 'sensor_msgs/msg/Image',
        '/radar/image_raw': 'sensor_msgs/msg/Image',
        '/odometry': 'nav_msgs/msg/Odometry',
        '/tf_static': 'tf2_msgs/msg/TFMessage',
        '/tf': 'tf2_msgs/msg/TFMessage',
    }

    for topic, msg_type_str in topics.items():
        metadata = TopicMetadata(
            name=topic,
            type=msg_type_str,
            serialization_format="cdr")
        writer.create_topic(metadata)

    # Gather all timestamps from sensor files
    all_stamps = set()
    timestamp_to_file = defaultdict(dict)

    # lidar
    for file in os.listdir(lidar_folder):
        if file.endswith('.bin'):
            ts = timestamp_from_filename(file)
            t_float = ts.sec + ts.nanosec / 1e9
            all_stamps.add(t_float)
            timestamp_to_file[t_float]['lidar'] = file

    # camera
    for file in os.listdir(camera_folder):
        if file.endswith('.png'):
            ts = timestamp_from_filename(file)
            t_float = ts.sec + ts.nanosec / 1e9
            all_stamps.add(t_float)
            timestamp_to_file[t_float]['camera'] = file

    # radar
    for file in os.listdir(radar_folder):
        if file.endswith('.png'):
            ts = timestamp_from_filename(file)
            t_float = ts.sec + ts.nanosec / 1e9
            all_stamps.add(t_float)
            timestamp_to_file[t_float]['radar'] = file

    # Sort all timestamps
    sorted_timestamps = sorted(all_stamps)

    start_sec = int(sorted_timestamps[0])
    start_nsec = int((sorted_timestamps[0] - start_sec) * 1e9)

    # Write static transforms
    tf_static = TFMessage()
    static_ros_time = Time(sec=start_sec, nanosec=start_nsec)
    static_time = static_ros_time.sec * int(1e9) + static_ros_time.nanosec

    # Load each static transform, then set its stamp properly:
    # T_A_B -> Transformation from frame A to frame B
    tf1 = load_static_tf(os.path.join(calib_folder, 'T_applanix_lidar.txt'), 'base_link', 'lidar')
    tf2 = load_inverse_static_tf(os.path.join(calib_folder, 'T_camera_lidar.txt'), 'lidar', 'camera')
    tf3 = load_inverse_static_tf(os.path.join(calib_folder, 'T_radar_lidar.txt'), 'lidar', 'radar')

    # Set their header.stamp to match static_ros_time:
    for tf in [tf1, tf2, tf3]:
        tf.header.stamp = static_ros_time
        tf_static.transforms.append(tf)

    try:
        writer.write('/tf_static', serialize_message(tf_static), static_time)
    except Exception as e:
        print(f"Failed to write static transforms: {e}")

    
    for t_float in tqdm(sorted_timestamps):
        sec = int(t_float)
        nsec = int((t_float - sec) * 1e9)
        ros_time = Time(sec=sec, nanosec=nsec)
        timestamp = ros_time.sec * int(1e9) + ros_time.nanosec

        print(f"processed time : {t_float - sorted_timestamps[0]:.2f} seconds")

        if bag_length != -1:
            if (t_float - sorted_timestamps[0] > bag_length):
                break
        
        try:
            # lidar
            if 'lidar' in timestamp_to_file[t_float]:
                lidar_file = os.path.join(lidar_folder, timestamp_to_file[t_float]['lidar'])
                lidar_data = load_lidar_bin(lidar_file)
                lidar_msg = create_pointcloud2(lidar_data, ros_time)
                writer.write('/lidar', serialize_message(lidar_msg), timestamp)

            # camera
            if 'camera' in timestamp_to_file[t_float]:
                camera_file = os.path.join(camera_folder, timestamp_to_file[t_float]['camera'])
                cam_msg = create_image_msg(camera_file, 'camera', ros_time)
                writer.write('/camera/image_raw', serialize_message(cam_msg), timestamp)

            # radar
            if 'radar' in timestamp_to_file[t_float]:
                radar_file = os.path.join(radar_folder, timestamp_to_file[t_float]['radar'])
                radar_msg = create_image_msg(radar_file, 'radar', ros_time)
                writer.write('/radar/image_raw', serialize_message(radar_msg), timestamp)

        except Exception as e:
            print(f"Failed at {timestamp}: {e}")

    poses = pd.read_csv(os.path.join(base_path, 'applanix/gps_post_process.csv'))
    poses = poses.dropna(subset=['GPSTime', 'easting', 'northing', 'altitude'])

    # Find closest timestamp in poses to the first sorted timestamp.
    first_timestamp, first_timestamp_idx = find_closest_timestamp(
        sorted_timestamps[0], poses['GPSTime'].values
    )

    for idx, row in tqdm(
        poses.iloc[first_timestamp_idx:].iterrows(),
        total=len(poses) - first_timestamp_idx
    ):
        stamp_sec = int(row['GPSTime'])
        stamp_nsec = int((row['GPSTime'] % 1) * 1e9)
        ros_time = Time(sec=stamp_sec, nanosec=stamp_nsec)
        timestamp = ros_time.sec * int(1e9) + ros_time.nanosec

        print(f"processed time : {(row['GPSTime'] - first_timestamp):.2f} seconds")

        if bag_length != -1 and (row['GPSTime'] - first_timestamp > bag_length):
            break

        # Odometry
        odom_msg = create_odometry_msg(row, 'world', 'base_link', ros_time)
        writer.write('/odometry', serialize_message(odom_msg), timestamp)

        # TF (dynamic)
        tf_msg = TFMessage(transforms=[create_tf(row, 'world', 'base_link', ros_time)])
        writer.write('/tf', serialize_message(tf_msg), timestamp)


if __name__ == '__main__':
    base_path = r"boreas-2021-01-26-11-22"
    bag_length = 60 # seconds
    main(base_path, bag_length)
    rclpy.shutdown()
