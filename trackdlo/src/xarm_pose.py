#!/usr/bin/env python3

import rospy
from xarm.wrapper import XArmAPI
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from tf.transformations import quaternion_from_euler
import message_filters

def callback():
    # Initialize xArmAPI; adjust the IP address as needed.
    robot_ip = "192.168.1.209"
    arm = XArmAPI(robot_ip)
    arm.set_mode(1)
    print('Save XARM6 pose')
    print(f'robot ip: {robot_ip}')
    
    # Set the loop rate (15 Hz to customize other topics)
    rate = rospy.Rate(15)
    
    while not rospy.is_shutdown():
        # Get current xArm position; get_position() returns [status, pose]
        pose_data = arm.get_position()[1]
        gripper_data = arm.get_gripper_position()[1]
        
        if pose_data is not None and gripper_data is not None:
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
            
            # Create a Float32 message for gripper openness and publish it
            gripper_msg = Float32()
            gripper_msg.data = gripper_data
            gripper_pub.publish(gripper_msg)
            
        else:
            rospy.logwarn("Invalid or incomplete pose data received from xArm")
            
        rate.sleep()

if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('xarm_pose_recorder', anonymous=True)
    
    rgb_topic = rospy.get_param('/init_tracker/rgb_topic')
    rgb_sub = message_filters.Subscriber(rgb_topic, Image)
    
    # Publisher for xArm pose
    pose_pub = rospy.Publisher('/xarm/pose', PoseStamped, queue_size=30)
    gripper_pub = rospy.Publisher('/xarm/gripper', Float32, queue_size=30)
    callback()
    
    rospy.spin()
