#!/usr/bin/env python
import rospy
from xarm.wrapper import XArmAPI
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

def main():
    # Initialize ROS node
    rospy.init_node('xarm_pose_recorder', anonymous=True)
    
    # Publisher for xArm pose
    pose_pub = rospy.Publisher('xarm/pose', PoseStamped, queue_size=10)
    
    # Initialize xArmAPI; adjust the IP address as needed.
    robot_ip = "192.168.1.209"
    arm = XArmAPI(robot_ip)
    arm.set_mode(1)
    
    # Set the loop rate (10 Hz in this example)
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        # Get current xArm position; get_position() returns [status, pose]
        result = arm.get_position()
        pose_data = result[1]
        
        if pose_data is not None and len(pose_data) >= 6:
            # Create a PoseStamped message and fill it with data
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "xarm_base"  # Change as appropriate
            
            # Set position from first three elements: [x, y, z]
            pose_msg.pose.position.x = pose_data[0]
            pose_msg.pose.position.y = pose_data[1]
            pose_msg.pose.position.z = pose_data[2]
            
            # Convert Euler angles (roll, pitch, yaw) to quaternion for orientation
            quat = quaternion_from_euler(pose_data[3], pose_data[4], pose_data[5])
            pose_msg.pose.orientation.x = quat[0]
            pose_msg.pose.orientation.y = quat[1]
            pose_msg.pose.orientation.z = quat[2]
            pose_msg.pose.orientation.w = quat[3]
            
            # Publish the pose message
            pose_pub.publish(pose_msg)
        else:
            rospy.logwarn("Invalid or incomplete pose data received from xArm")
            
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
