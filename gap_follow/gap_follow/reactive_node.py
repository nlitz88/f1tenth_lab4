import math
from typing import List, Any
import rclpy
from rclpy.node import Node
from enum import Enum

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from std_msgs.msg import String
from rcl_interfaces.msg import SetParametersResult
from threading import Lock

from lidarutils import IndexRange, get_index_range_from_angles
from gap_follow_utils import ranges_under_threshold

class FollowGapState(Enum):
    INIT = 1
    MOVING_STRAIGHT = 2
    DISPARITY_CONTROL = 3

class ReactiveFollowGap(Node):
    """
    Implement Wall Following on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('reactive_node')
        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # Set up separate parameter dictionary and then declare the parameters.
        self.__parameters_mutex = Lock()
        self.__parameters = {
            "gap_scan_angle_range_deg": 90,
            "side_safety_dist_minimum_m": 0.15,
            "gap_depth_threshold_m": 2.5, # 3 was in lecture
            "range_upper_bound_m": 3,
            "disparity_threshold_m": 0.3,
            "lidar_angle_min_rad": -2.3499999046325684,
            "lidar_angle_max_rad": 2.3499999046325684,
            "lidar_angle_increment_rad": 0.004351851996034384,
            "lidar_num_ranges": 1080
        }
        self.declare_parameters(namespace="", 
                                parameters=[(str(key), self.__parameters[key]) for key in self.__parameters])

        # Register a parameter callback.
        self.add_on_set_parameters_callback(callback=self.__parameter_callback)

        # Create a scan subscriber.
        self.__scan_subscriber = self.create_subscription(msg_type=LaserScan,
                                                          topic=lidarscan_topic, 
                                                          callback=self.__lidar_callback, 
                                                          qos_profile=10)
        # Create a drive publisher.
        self.__drive_publisher = self.create_publisher(msg_type=AckermannDriveStamped,
                                                       topic=drive_topic,
                                                       qos_profile=10)
        
        # Variable for robot width.
        self.__car_width_m = 0.2032

        # Store last drive message.
        self.__last_drive_message = AckermannDriveStamped()

        # TODO: Call functions or add code here to initialize values that can be
        # computed up front / at initialization. Writing a function could be
        # good for the sake of being able to recompute those values at runtime
        # as a part of the parameter callback (if one of these values depends on
        # one or more parameters).
        
        # Grab relevant LiDAR values to compute these ranges with.
        angle_increment = self.__get_local_parameter("lidar_angle_increment_rad")
        angle_min = self.__get_local_parameter("lidar_angle_min_rad")
        angle_max = self.__get_local_parameter("lidar_angle_max_rad")
        num_ranges = self.__get_local_parameter("lidar_num_ranges")
        gap_scan_angle_range_rad = math.radians(self.__get_local_parameter("gap_scan_angle_range_deg"))
        # Get lower left side index range.
        self.__left_start_angle_rad = gap_scan_angle_range_rad
        self.__left_end_angle_rad = self.__get_local_parameter("lidar_angle_max_rad")
        self.__left_side_index_range: IndexRange = get_index_range_from_angles(start_angle_rad=self.__left_start_angle_rad,
                                                                               end_angle_rad=self.__left_end_angle_rad,
                                                                               angle_min_rad=angle_min,
                                                                               angle_max=angle_max,
                                                                               angle_increment_rad=angle_increment,
                                                                               num_ranges=num_ranges)
        # Get lower right side index range.
        self.__right_start_angle_rad = self.__get_local_parameter("lidar_angle_min_rad")
        self.__right_end_angle_rad = -gap_scan_angle_range_rad
        self.__right_side_index_range: IndexRange = get_index_range_from_angles(start_angle_rad=self.__right_start_angle_rad,
                                                                                end_angle_rad=self.__right_end_angle_rad,
                                                                                angle_min_rad=angle_min,
                                                                                angle_max=angle_max,
                                                                                angle_increment_rad=angle_increment,
                                                                                num_ranges=num_ranges)
        # Get the (main) middle index range.
        self.__middle_start_angle_rad = -gap_scan_angle_range_rad
        self.__middle_end_angle_rad = gap_scan_angle_range_rad
        self.__middle_index_range: IndexRange = get_index_range_from_angles(start_angle_rad=self.__middle_start_angle_rad,
                                                                            end_angle_rad=self.__middle_end_angle_rad,
                                                                            angle_min_rad=angle_min,
                                                                            angle_max_rad=angle_max,
                                                                            angle_increment_rad=angle_increment,
                                                                            num_ranges=num_ranges)
        # NOTE: If I don't want to have to deal with any rounding issues here,
        # could really just set the middle index range to +1 from the last of
        # the left, and -1 from the first of the right. I.e., like:
        # self.__middle_index_range = IndexRange(starting_index=self.__right_side_index_range.ending_index,
        #                                        ending_index=self.__left_side_index_range.starting_index)
        
        # State Variable.
        self.__current_state = FollowGapState.INIT

    def __get_local_parameter(self, param_name: str) -> Any:
        """Function to get the value of a parameter from the parameter
        dictionary maintained locally by the node.

        Args:
            param_name (str): The key for the parameter in the self.__parameters
            dictionary.

        Raises:
            exc: KeyError exception if the provided param_name isn't a key in
            the self.__parameters dictionary.

        Returns:
            Any: The value of the parameter if it exists in the parameters
            dictionary.
        """
        with self.__parameters_mutex:
            try:
                value = self.__parameters[param_name]
            except KeyError as exc:
                self.get_logger().error(f"Failed to get parameter with name {param_name}. Key not found in parameter dictionary!")
                raise exc
            else:
                return value
        
    def __parameter_callback(self, params: List[rclpy.Parameter]) -> SetParametersResult:
        """Function called whenever any of the node's parameters are updated and
        updates a local copy of the parameter used during publishing.
        """
        with self.__parameters_mutex:
            for param in params:
                if param.name in list(self.__parameters):
                    self.__parameters[param.name] = param.value
                    self.get_logger().info(f"Received updated version of parameter {str(param.name)}, new value {param.value}")
            return SetParametersResult(successful=True)
        
    def publish_control(self, 
                        new_steering_angle: float, 
                        new_velocity: float) -> None:
        """Helper function that handles publishing the provided steering angle
        and velocity to the drive topic. Constructs an AckermannDriveStamped
        message and publishes it to the drive topic.

        :param new_steering_angle: The steering angle (in radians) to be
            commanded. 
        :type steering_angle: float
        :param new_velocity: The velocity (in m/s) to be commanded.
        :type velocity: float
        """
        # Construct new Ackermann drive message, set velocity and steering
        # angle.
        drive_message = AckermannDriveStamped()
        drive_message.drive.steering_angle = new_steering_angle
        drive_message.drive.speed = new_velocity
        # Publish the populated message to the drive topic.
        self.__drive_publisher.publish(drive_message)
        return

    def __sides_too_close(self, ranges: List[float]) -> bool:
        """Returns whether or not the car is too close to an obstacle on either
        one of its sides. Uses the precomputed side 

        Args:
            ranges (List[float]): List of range values from the LaserScan to be
            used to check for car's proximity to nearby obstacles on its sides.

        Returns:
            bool: True if the car is within the side_safety_dist_minimum_m
            threshold on either side, False if not.
        """
        # First, grab the minimum distance threshold that we'll use for both
        # sides.
        minimum_distance_threshold = self.__get_local_parameter("side_safety_dist_minimum_m")
        # First, check to see if it's too close to anything on its right side.
        right_indices = self.__right_side_index_range.get_indices()
        if ranges_under_threshold(ranges=ranges,
                                  range_indices=right_indices,
                                  minimum_distance_m=minimum_distance_threshold):
            return True
        # If the right is fine, check to left to make sure everything is okay.
        left_indices = self.__left_side_index_range.get_indices()
        if ranges_under_threshold(ranges=ranges,
                                  range_indices=left_indices,
                                  minimum_distance_m=minimum_distance_threshold):
            return True
        # Otherwise, it's not too close on either side, return False.
        return False

    def __going_to_hit_wall(self, ranges: List[float], last_steering_angle_rad: float) -> bool:
        """Returns whether or not the car is currently bound to hit the wall
        based on its current distance to obstacles on either of its sides and
        its current steering angle.

        Args:
            ranges (List[float]): List of range values from LaserScan.
            last_steering_angle_rad (float): Last steering angle (in radians)
            published to /drive.

        Returns:
            bool: True if the car is bound to collide with an obstacle on one of
            its sides, False if not.
        """
        return self.__sides_too_close(self) and last_steering_angle_rad != 0

    def __moving_straight_state(self) -> None:
        """Wrapper function for all the operations ("side effects") to be
        carried out in the MOVING_FORWARD state. Sets steering angle to 0, speed
        equal to its previous value, and publishes the Ackermann message to
        /drive.
        """
        new_steering_angle = 0.0
        # TODO: In case the speed is too high in these scenarios, add an
        # instance variable that stores the last followed gap depth, and
        # compute what the new velocity should be based on that previous
        # depth, but with the distance travelled since that was sent
        # (calculated using current velocity and time between cycles).
        new_speed = self.__last_drive_message.drive.speed
        # Then, use the drive publisher to publish the ackermann steering
        # message with these values.
        self.publish_control(new_steering_angle=new_steering_angle, new_velocity=new_speed)
        return
    
    def __disparity_control_state(self, ranges: List[float]) -> None:
        """Wrapper function for all the operations/actions to be carried out
        while the car is in the DISPARITY_CONTROL state.
        """

        # Before doing anything else, grab the working range of indices. I.e.,
        # all range values except for those on the side.
        range_indices = self.__middle_index_range.get_indices()

        # 1. (TODO) Clamp all range values to maximum depth value. Choosing not
        #    to implement this first, just to see what performance is like
        #    without it.

        # 2. Next, find all the index pairs in the ranges array where there is a
        #    disparity that exceeds the disparity threshold.
        

        # 3. Then, extend each disparity according to the width of the car and
        #    the depth (range) at which that disparity occurs.
        # NOTE: steps 2 and 3 could probably be combined into a single function.
        # However, probably more testable and easier to understand if they're
        # separate for now.

        # 4. Then, write function for that finds the start and end index of each
        #    gap in the range. Can find the max depth of each gap as well, and
        #    return them in sorted order (using something like heapify?).
        #    Probably break this step up internally into two smaller steps.

        # NOTE: MAKE SURE TO CHECK FOR WHEN FIND_GAPS DOESN'T RETURN ANY
        # GAPS--NEED SOME SORT OF OTHER STATE TO DROP INTO OR SOME RECOURSE
        # PLAN! MAYBE MAKE A BLOCKED STATE OR SOMETHING LIKE THAT?

        # 5. Then, call a function that gets the middle of the selected gap.
        #    I.e., returns the index of the middle of that gap. Can call the
        #    lidarutils function that gets an angle from that index and returns
        #    that.

        # 6. Finally, call a separate function that computes velocity as a
        #    function of the depth of the selected gap. Could also use a
        #    simplified version of this function for now that just uses constant
        #    speed, or could also base it on steering angle.

        # 7. Finally, call function to publish newly computed steering angle and
        #    speed.
        self.publish_control(new_steering_angle=18239192387123.232, new_velocity=891283213.23)
        return

    def __lidar_callback(self, laser_scan: LaserScan):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an
        AckermannDriveStamped Message.
        """
        ranges: List[float] = laser_scan.ranges

        if self.__current_state == FollowGapState.INIT:
            # Unconditional guard used to transition into DISPARITY_CONTROL
            # state.
            if True:
                self.__current_state = FollowGapState.DISPARITY_CONTROL

        elif self.__current_state == FollowGapState.MOVING_STRAIGHT:
            
            # Apply side effects of being in MOVING_STRAIGHT state. I.e., set
            # steering angle to 0 and make speed unchanged.
            self.__moving_straight_state()

            # Guard condition here.
            # If side is still too close, continue moving straight.
            # TODO: just NOTE that I MAY HAVE TO also add a second guard
            # condition here to check whether our steering angle is nonzero.
            # This comes directly from the slides on disparity based control.
            if self.__going_to_hit_wall(ranges=ranges, last_steering_angle_rad=self.__last_drive_message.drive.steering_angle):
                self.__current_state = FollowGapState.MOVING_STRAIGHT
            # Otherwise, switch back to disparity control state.
            else:
                self.__current_state = FollowGapState.DISPARITY_CONTROL

        elif self.__current_state == FollowGapState.DISPARITY_CONTROL:

            # Take the ranges and 
            self.__disparity_control_state(ranges=ranges)

            # Guard Condition here.
            # If side is still too close, continue moving straight.
            if self.__going_to_hit_wall(ranges=ranges, last_steering_angle_rad=self.__last_drive_message.drive.steering_angle):
                self.__current_state = FollowGapState.MOVING_STRAIGHT
            # Otherwise, remain in disparity control state.
            else:
                self.__current_state = FollowGapState.DISPARITY_CONTROL


def main(args=None):
    rclpy.init(args=args)
    print("GapFollow Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()