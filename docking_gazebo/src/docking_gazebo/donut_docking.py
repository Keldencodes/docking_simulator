#!usr/bin/env python

import roslib
roslib.load_manifest('docking_gazebo')
import rospy
import mavros
import cv2
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class donut_docking:

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber('/camera_test/camera1/image_raw', Image, self.detect_circle)

    self.cv_pose = np.array([np.nan, np.nan, np.nan])
    self.cv_pose_avg = np.array ([np.nan, np.nan, np.nan])
    self.cv_feed_pos_pub = rospy.Publisher('/donut/mavros/mocap/pose', PoseStamped, queue_size=10)

    self.state_sub = rospy.Subscriber('/donut/mavros/state', State, self.state_cb)
    self.local_pos_pub = rospy.Publisher('/donut/mavros/setpoint_position/local', PoseStamped, queue_size=10)

    self.arming_client = rospy.ServiceProxy('/donut/mavros/cmd/arming', CommandBool)
    self.set_mode_client = rospy.ServiceProxy('/donut/mavros/set_mode', SetMode) 

  def reject_outliers(self,data,m=2):
    return data[abs(data - np.mean(data)) < m * np.std(data)]

  def state_cb(self,data):
    global current_state
    current_state = data

  def detect_circle(self,data):
    global pose_avg_store, cv_avg_poses

    try:
      img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
    except CvBridgeError as e:
      print(e)

    # blur image edges
    img = cv2.medianBlur(img, 3)

    # convert from bgr to hsv
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # threshold the HSV image, keep only the red pixels
    lower_red_hue_range = cv2.inRange(hsv, (0, 100, 100), (10, 255, 255))
    upper_red_hue_range = cv2.inRange(hsv, (160, 100, 100), (179, 255, 255))

    # Combine the above two images
    red_hue_image = cv2.addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0)
    red_hue_image = cv2.GaussianBlur(red_hue_image, (9, 9), 2, 2)

    # detect circles
    circs = cv2.HoughCircles(red_hue_image, cv2.HOUGH_GRADIENT, 1, 20, 
                             param1=20, param2=10, minRadius=0, maxRadius=0)

    if circs is None:
      self.cv_pose = np.array([np.nan, np.nan, np.nan])
    else:
      # take the first circle vector
      circ_r = np.uint16(np.around(circs[0,:][0]))
      # draw detected circle and center point
      cv2.circle(img,(circ_r[0],circ_r[1]),circ_r[2],(0,255,0),2)
      cv2.circle(img,(circ_r[0],circ_r[1]),2,(255,0,0),3)

      circ_x = circs[0,:][0][0]
      circ_y = circs[0,:][0][1]
      circ_d = 2*circs[0,:][0][2]
      img_height, img_width, _ = img.shape
      img_center_x = img_width/2
      img_center_y = img_height/2
      diff_x = img_center_x - circ_x
      diff_y = img_center_y - circ_y
      fov = np.radians(80)
      foc_len = (img_width/2)/(np.tan(fov/2))

      marker_d = 0.1
      unit_dist = marker_d/circ_d
      self.cv_pose[0] = foc_len*unit_dist
      self.cv_pose[1] = diff_x*unit_dist
      self.cv_pose[2] = diff_y*unit_dist

      if pose_avg_store >= 0:
        pose_avg_store -= 1
        cv_avg_poses[pose_avg_store,:] = self.cv_pose
      elif pose_avg_store == -1:
        pose_avg_store = 5
        if pose_iter > 0:
          self.cv_pose_avg = np.mean(cv_avg_poses, axis=0)
        elif pose_iter == 0:
          cv_pose_avg = np.mean(cv_avg_poses, axis=0)
          compare = np.abs(np.subtract(cv_pose_avg, self.cv_pose_avg))
          if np.all(compare < 0.2):
            self.cv_pose_avg = cv_pose_avg

      cv2.imshow("Image Stream", img)
      cv2.waitKey(3)

  def position_control(self):
    global desired_pose, cv_poses, pose_store, pose_iter

    if np.isfinite(self.cv_pose).any():
      if pose_store >= 0 and pose_iter > 0:
        pose_store -= 1
        cv_poses[pose_store,:] = self.cv_pose
      elif pose_store == -1 and pose_iter > 0:
        pose_store = 200
        pose_iter -= 1

        cv_pose_x = np.mean(self.reject_outliers(cv_poses[:,0]))
        cv_pose_y = np.mean(self.reject_outliers(cv_poses[:,1]))
        cv_pose_z = np.mean(self.reject_outliers(cv_poses[:,2]))

        desired_pose[0] = desired_pose[0] + (1.5 - cv_pose_x)
        desired_pose[1] = desired_pose[1] + (0.0 - cv_pose_y)
        desired_pose[2] = desired_pose[2] + (0.0 - cv_pose_z)
      # elif pose_iter == 0:
      #   feed_pose = PoseStamped()
      #   feed_pose.pose.position.x = self.cv_pose_avg[0] - 3.0
      #   feed_pose.pose.position.y = self.cv_pose_avg[1] - 5.0
      #   feed_pose.pose.position.z = self.cv_pose_avg[2] + 0.825
  
      #   # publish pose 
      #   feed_pose.header.stamp = rospy.Time.now()
      #   self.cv_feed_pos_pub.publish(feed_pose)

    pose = PoseStamped()
    pose.pose.position.x = desired_pose[0]
    pose.pose.position.y = desired_pose[1]
    pose.pose.position.z = desired_pose[2]
  
    # Update timestamp and publish pose 
    pose.header.stamp = rospy.Time.now()
    self.local_pos_pub.publish(pose)

offb_set_mode = SetMode
current_state = State()

desired_pose = np.array([0.0, -5.0, 6.0])
pose_store = 200
pose_iter = 2
cv_poses = np.empty((pose_store,3))

pose_avg_store = 5
cv_avg_poses = np.empty((pose_avg_store,3))

def main():
  dd = donut_docking()
  rospy.init_node('donut_docking')
  
  global current_state
  prev_state = current_state

  rate = rospy.Rate(20.0) # MUST be more then 2Hz

  # send a few setpoints before starting
  for i in range(100):
    dd.position_control()
    rate.sleep()
    
  # wait for FCU connection
  while not current_state.connected:
    rate.sleep()

  last_request = rospy.get_rostime()
  while not rospy.is_shutdown():
    now = rospy.get_rostime()
    if current_state.mode != "OFFBOARD" and (now - last_request > rospy.Duration(5.)):
      dd.set_mode_client(base_mode=0, custom_mode="OFFBOARD")
      last_request = now 
    else:
      if not current_state.armed and (now - last_request > rospy.Duration(5.)):
        dd.arming_client(True)
        last_request = now 

    # older versions of PX4 always return success==True, so better to check Status instead
    if prev_state.armed != current_state.armed:
      rospy.loginfo("Vehicle armed: %r" % current_state.armed)
    if prev_state.mode != current_state.mode: 
      rospy.loginfo("Current mode: %s" % current_state.mode)
    prev_state = current_state

    # Update timestamp and publish pose 
    dd.position_control()
    rate.sleep()

if __name__ == '__main__':
  main()