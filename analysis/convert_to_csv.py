import rclpy
import csv
from rclpy.node import Node
from imu_msg.msg import IMUmsg

class BagReader(Node):
    def __init__(self):
        super().__init__('bag_reader')
        self.subscription = self.create_subscription(
            IMUmsg,
            '/imu',
            self.listener_callback,
            10
        )
        
        self.csv_file = open('imu_data_driving.csv', mode='w', newline='')   
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['time_sec', 'time_nanosec',
                                    'q_x', 'q_y', 'q_z', 'q_w', 
                                    'av_x', 'av_y', 'av_z', 
                                    'la_x', 'la_y', 'la_z', 
                                    'mag_x', 'mag_y', 'mag_z'])

    def listener_callback(self, msg):
        self.csv_writer.writerow([msg.header.stamp.sec, msg.header.stamp.nanosec,
                                    msg.imu.orientation.x, msg.imu.orientation.y, msg.imu.orientation.z, msg.imu.orientation.w,
                                    msg.imu.angular_velocity.x, msg.imu.angular_velocity.y, msg.imu.angular_velocity.z,
                                    msg.imu.linear_acceleration.x, msg.imu.linear_acceleration.x, msg.imu.linear_acceleration.x,
                                    msg.mag_field.magnetic_field.x/1e-4, msg.mag_field.magnetic_field.y/1e-4, msg.mag_field.magnetic_field.z/1e-4])
        self.get_logger().info(f'time_sec: {msg.header.stamp.sec}, time_nanosec: {msg.header.stamp.nanosec}, q_x: {msg.imu.orientation.x}, q_y: {msg.imu.orientation.y}, q_z: {msg.imu.orientation.z}, q_w: {msg.imu.orientation.w}, av_x: {msg.imu.angular_velocity.x}, av_y: {msg.imu.angular_velocity.y}, av_z: {msg.imu.angular_velocity.z}, la_x: {msg.imu.linear_acceleration.x}, la_y: {msg.imu.linear_acceleration.y}, la_z: {msg.imu.linear_acceleration.z}, mag_x: {msg.mag_field.magnetic_field.x}, mag_y: {msg.mag_field.magnetic_field.y}, mag_z: {msg.mag_field.magnetic_field.z}')

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    bag_reader = BagReader()
    rclpy.spin(bag_reader)
    bag_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()