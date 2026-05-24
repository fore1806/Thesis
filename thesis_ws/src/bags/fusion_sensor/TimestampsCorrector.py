import rosbag2_py
from rclpy.serialization import serialize_message, deserialize_message
from sensor_msgs.msg import PointCloud2
import os
import shutil

# --- CONFIGURATION ---
INPUT_BAG = 'session2Real_decompressed'
OUTPUT_BAG = 'session2Real_fixed'
TARGET_TOPIC = '/rslidar_points'
# ---------------------

def fix_bag():
    reader = rosbag2_py.SequentialReader()

    storage_options = rosbag2_py.StorageOptions(
        uri=INPUT_BAG,
        storage_id='sqlite3'
    )

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )

    reader.open(storage_options, converter_options)

    
    writer = rosbag2_py.SequentialWriter()

    output_storage_options = rosbag2_py.StorageOptions(
        uri=OUTPUT_BAG,
        storage_id='sqlite3'
    )

    writer.open(output_storage_options, converter_options)  # writer handles dir creation

    topic_types = reader.get_all_topics_and_types()
    for topic in topic_types:
        writer.create_topic(topic)

    print(f"Processing bag... rewriting {TARGET_TOPIC} headers.")
    count = 0

    while reader.has_next():
        topic, data, timestamp = reader.read_next()

        if topic == TARGET_TOPIC:
            msg = deserialize_message(data, PointCloud2)
            msg.header.stamp.sec = int(timestamp // 1e9)
            msg.header.stamp.nanosec = int(timestamp % 1e9)
            data = serialize_message(msg)
            count += 1

            if count % 500 == 0:
                print(f"Fixed {count} LiDAR frames...")

        writer.write(topic, data, timestamp)

    print(f"\nDone! Fixed {count} LiDAR frames total.")
    print(f"Output bag: {OUTPUT_BAG}")


if __name__ == "__main__":
    fix_bag()