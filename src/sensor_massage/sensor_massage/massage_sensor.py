import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, AccelStamped
from sensor_msgs.msg import Imu, MagneticField
import message_filters

class SensorFuser(Node):
    def __init__(self):
        super().__init__('sensor_fuser')

        # 1. Setup Subscribers
        # Note: Using TwistStamped based on your data structure ('linear' and 'angular' fields)
        self.accel_sub = message_filters.Subscriber(self, AccelStamped, '/accelerometer')
        self.gyro_sub = message_filters.Subscriber(self, TwistStamped, '/gyroscope')
        self.mag_sub = message_filters.Subscriber(self, MagneticField, '/magnetometer')

        # 2. Synchronize the three topics
        # queue_size=10, slop=0.1 allows for 100ms difference in timestamps
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.accel_sub, self.gyro_sub, self.mag_sub], 
            queue_size=10, 
            slop=0.1
        )
        self.ts.registerCallback(self.common_callback)

        # 3. Publisher for Madgwick Filter
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        
        self.get_logger().info('Sensor Fuser Node started. Combining 3 topics into /imu/data_raw')

    def common_callback(self, accel_msg, gyro_msg, mag_msg):
        imu_msg = Imu()
        
        # Use current time if the incoming message has zero timestamp
        if accel_msg.header.stamp.sec == 0 and accel_msg.header.stamp.nanosec == 0:
            imu_msg.header.stamp = self.get_clock().now().to_msg()
        else:
            imu_msg.header.stamp = accel_msg.header.stamp
        imu_msg.header.frame_id = 'imu_link' # Ensure this matches your TF tree

        # Map Accelerometer 'linear' values
        imu_msg.linear_acceleration.x = accel_msg.accel.linear.x
        imu_msg.linear_acceleration.y = accel_msg.accel.linear.y
        imu_msg.linear_acceleration.z = accel_msg.accel.linear.z

        # Map Gyroscope 'angular' values
        imu_msg.angular_velocity.x = gyro_msg.twist.angular.x
        imu_msg.angular_velocity.y = gyro_msg.twist.angular.y
        imu_msg.angular_velocity.z = gyro_msg.twist.angular.z

        # Note: The Magnetometer data is usually handled by the filter 
        # on its own topic (/imu/mag), but we combine Accel/Gyro here for /imu/data_raw
        
        self.imu_pub.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SensorFuser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()