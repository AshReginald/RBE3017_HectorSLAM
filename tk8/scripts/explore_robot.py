#!/usr/bin/env python3

import rospy
import actionlib
import numpy as np
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from scipy.ndimage import label
from tf import TransformListener

class RobotExplorer:
    def __init__(self):
        rospy.init_node('robot_explorer', anonymous=True, log_level=rospy.DEBUG)
        rospy.sleep(1.0)
        
        # Parameters
        self.robot_frame = rospy.get_param('~robot_base_frame', 'base_link')
        self.costmap_topic = rospy.get_param('~costmap_topic', 'move_base/global_costmap/costmap')
        self.min_frontier_size = rospy.get_param('~min_frontier_size', 0.5)  # meters
        self.planner_frequency = rospy.get_param('~planner_frequency', 0.2)  # Hz
        self.transform_tolerance = rospy.get_param('~transform_tolerance', 1.5)
        self.min_distance_to_obstacle = rospy.get_param('~min_distance_to_obstacle', 0.5)  # meters
        
        # Subscribers
        self.costmap_sub = rospy.Subscriber(self.costmap_topic, OccupancyGrid, self.costmap_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Initialize TF listener
        self.tf_listener = TransformListener()
        
        # Move base action client
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        if not self.move_base_client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Could not connect to move_base action server")
            rospy.signal_shutdown("Action server not available")
            return
        
        # State variables
        self.costmap = None
        self.map_metadata = None
        self.min_range = float('inf')
        self.rate = rospy.Rate(self.planner_frequency)
        self.goal_active = False
        rospy.loginfo("Robot Explorer initialized successfully")

    def scan_callback(self, msg):
        """Update minimum distance to obstacles from laser scan."""
        valid_ranges = [r for r in msg.ranges if not np.isnan(r) and not np.isinf(r)]
        self.min_range = min(valid_ranges) if valid_ranges else float('inf')
        
    def costmap_callback(self, msg):
        """Store the latest costmap and its metadata."""
        self.costmap = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        self.map_metadata = msg.info
        rospy.logdebug("Received new costmap")
        
    def get_frontiers(self):
        """Identify frontiers (boundaries between known free and unknown areas)."""
        if self.costmap is None or self.map_metadata is None:
            rospy.logwarn("No costmap data available")
            return []
        
        # Free space (0), occupied (>0), unknown (-1)
        free = (self.costmap == 0)
        unknown = (self.costmap == -1)
        
        # Find boundaries: free cells adjacent to unknown cells
        frontier = np.zeros_like(self.costmap, dtype=np.bool_)
        for i in range(1, self.costmap.shape[0] - 1):
            for j in range(1, self.costmap.shape[1] - 1):
                if free[i, j]:
                    neighbors = self.costmap[i-1:i+2, j-1:j+2]
                    if np.any(neighbors == -1):
                        frontier[i, j] = True
        
        # Label connected frontier regions
        labeled, num_features = label(frontier)
        frontiers = []
        
        # Convert min_frontier_size from meters to cells
        min_cells = int(self.min_frontier_size / self.map_metadata.resolution)
        
        for i in range(1, num_features + 1):
            frontier_points = np.where(labeled == i)
            if len(frontier_points[0]) < min_cells:
                continue
                
            # Compute centroid
            centroid_y = np.mean(frontier_points[0]) * self.map_metadata.resolution + self.map_metadata.origin.position.y
            centroid_x = np.mean(frontier_points[1]) * self.map_metadata.resolution + self.map_metadata.origin.position.x
            frontiers.append((centroid_x, centroid_y))
        
        rospy.logdebug(f"Found {len(frontiers)} frontier regions")
        return frontiers
    
    def select_goal(self, frontiers):
        """Select the closest frontier as the navigation goal."""
        if not frontiers:
            rospy.logdebug("No frontiers available for selection")
            return None
            
        robot_pose = self.get_robot_pose()
        if not robot_pose:
            rospy.logwarn("Could not get robot pose for goal selection")
            return None
            
        robot_x = robot_pose.pose.position.x
        robot_y = robot_pose.pose.position.y
        
        # Filter frontiers that are reachable (not behind obstacles)
        valid_frontiers = []
        for fx, fy in frontiers:
            # Simple check: if goal is at least min_distance_to_obstacle away from robot
            distance = np.sqrt((fx - robot_x)**2 + (fy - robot_y)**2)
            if distance > self.min_distance_to_obstacle:
                valid_frontiers.append((fx, fy))
        
        if not valid_frontiers:
            rospy.logdebug("No valid frontiers after filtering")
            return None
            
        # Choose the closest valid frontier
        distances = [np.sqrt((fx - robot_x)**2 + (fy - robot_y)**2) for fx, fy in valid_frontiers]
        closest_idx = np.argmin(distances)
        selected_goal = valid_frontiers[closest_idx]
        rospy.logdebug(f"Selected goal at {selected_goal} from {len(valid_frontiers)} valid frontiers")
        return selected_goal
    
    def get_robot_pose(self):
        """Get the current robot pose in the map frame."""
        try:
            self.tf_listener.waitForTransform('map', self.robot_frame, 
                                           rospy.Time(0), 
                                           rospy.Duration(self.transform_tolerance))
            (trans, rot) = self.tf_listener.lookupTransform('map', self.robot_frame, rospy.Time(0))
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = trans[0]
            pose.pose.position.y = trans[1]
            pose.pose.position.z = trans[2]
            pose.pose.orientation.x = rot[0]
            pose.pose.orientation.y = rot[1]
            pose.pose.orientation.z = rot[2]
            pose.pose.orientation.w = rot[3]
            return pose
        except Exception as e:
            rospy.logwarn(f"Failed to get robot pose: {e}")
            return None
    
    def publish_goal(self, goal_pos):
        """Publish a navigation goal to move_base."""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = goal_pos[0]
        goal.target_pose.pose.position.y = goal_pos[1]
        goal.target_pose.pose.position.z = 0.0
        
        # Calculate orientation to face the goal from current position
        robot_pose = self.get_robot_pose()
        if robot_pose:
            dx = goal_pos[0] - robot_pose.pose.position.x
            dy = goal_pos[1] - robot_pose.pose.position.y
            yaw = np.arctan2(dy, dx)
            quat = quaternion_from_euler(0, 0, yaw)
            goal.target_pose.pose.orientation.x = quat[0]
            goal.target_pose.pose.orientation.y = quat[1]
            goal.target_pose.pose.orientation.z = quat[2]
            goal.target_pose.pose.orientation.w = quat[3]
        else:
            # Fallback to default orientation if robot pose not available
            goal.target_pose.pose.orientation.w = 1.0
        
        try:
            self.move_base_client.send_goal(goal)
            self.goal_active = True
            rospy.loginfo(f"Successfully sent goal to move_base: {goal_pos}")
        except Exception as e:
            rospy.logerr(f"Failed to send goal to move_base: {str(e)}")
            self.goal_active = False
        
    def check_goal_status(self):
        """Check if the current goal is still being pursued or has been reached."""
        if not self.goal_active:
            return False
            
        state = self.move_base_client.get_state()
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal reached successfully")
            self.goal_active = False
            return False
        elif state in [actionlib.GoalStatus.ABORTED, 
                      actionlib.GoalStatus.REJECTED,
                      actionlib.GoalStatus.PREEMPTED,
                      actionlib.GoalStatus.RECALLED,
                      actionlib.GoalStatus.LOST]:
            rospy.logwarn(f"Goal failed with status: {state}")
            self.goal_active = False
            return False
        return True
        
    def avoid_obstacles(self):
        """Adjust velocity to avoid obstacles based on laser scan."""
        if self.min_range < self.min_distance_to_obstacle:
            cmd_vel = Twist()
            # Stop and rotate to avoid obstacle
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.3 if np.random.rand() > 0.5 else -0.3  # Random rotation direction
            self.cmd_vel_pub.publish(cmd_vel)
            rospy.loginfo(f"Obstacle detected at {self.min_range:.2f}m, avoiding")
            return True
        return False
        
    def run(self):
        """Main exploration loop."""
        rospy.loginfo("Starting exploration")
        
        while not rospy.is_shutdown():
            try:
                # Check for obstacles
                obstacle_detected = self.avoid_obstacles()
                
                # Skip planning if too close to an obstacle or goal is active
                if obstacle_detected or self.check_goal_status():
                    self.rate.sleep()
                    continue
                    
                # Find and select frontier
                frontiers = self.get_frontiers()
                goal_pos = self.select_goal(frontiers)
                
                if goal_pos:
                    self.publish_goal(goal_pos)
                else:
                    rospy.loginfo("No valid frontiers found")
                    # Recovery behavior: small random rotation
                    if np.random.rand() > 0.7:  # 30% chance to rotate
                        cmd_vel = Twist()
                        cmd_vel.angular.z = 0.5 if np.random.rand() > 0.5 else -0.5
                        self.cmd_vel_pub.publish(cmd_vel)
                        rospy.loginfo("Performing random rotation to find new frontiers")
                
                self.rate.sleep()
                
            except Exception as e:
                rospy.logerr(f"Error in main loop: {str(e)}")
                self.rate.sleep()

if __name__ == '__main__':
    try:
        explorer = RobotExplorer()
        explorer.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Exploration finished")
