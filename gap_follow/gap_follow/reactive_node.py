import math
from typing import List, Any
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from std_msgs.msg import String
from rcl_interfaces.msg import SetParametersResult
from threading import Lock

from lidarutils import IndexRange, get_index_range_from_angles

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
        self.__robot_width = 0.1016
        self.__robot_width_mutex = Lock()

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
        fake_ranges = [0 for _ in range(num_ranges)]
        gap_scan_angle_range_rad = math.radians(self.__get_local_parameter("gap_scan_angle_range_deg"))
        # Get lower left side index range.
        self.__left_start_angle_rad = gap_scan_angle_range_rad
        self.__left_end_angle_rad = self.__get_local_parameter("lidar_angle_max_rad")
        self.__left_side_index_range: IndexRange = get_index_range_from_angles(start_angle_rad=self.__left_start_angle_rad,
                                                                               end_angle_rad=self.__left_end_angle_rad,
                                                                               angle_min_rad=angle_min,
                                                                               angle_max=angle_max,
                                                                               angle_increment_rad=angle_increment,
                                                                               ranges_m=fake_ranges)
        # Get lower right side index range.
        self.__right_start_angle_rad = self.__get_local_parameter("lidar_angle_min_rad")
        self.__right_end_angle_rad = -gap_scan_angle_range_rad
        self.__right_side_index_range: IndexRange = get_index_range_from_angles(start_angle_rad=self.__right_start_angle_rad,
                                                                                end_angle_rad=self.__right_end_angle_rad,
                                                                                angle_min_rad=angle_min,
                                                                                angle_max=angle_max,
                                                                                angle_increment_rad=angle_increment,
                                                                                ranges_m=fake_ranges)
        # Get the (main) middle index range.
        self.__middle_start_angle_rad = -gap_scan_angle_range_rad
        self.__middle_end_angle_rad = gap_scan_angle_range_rad
        self.__middle_index_range: IndexRange = get_index_range_from_angles(start_angle_rad=self.__middle_start_angle_rad,
                                                                            end_angle_rad=self.__middle_end_angle_rad,
                                                                            angle_min_rad=angle_min,
                                                                            angle_max_rad=angle_max,
                                                                            angle_increment_rad=angle_increment,
                                                                            ranges_m=fake_ranges)

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

    def robot_description_callback(self, robot_description: String) -> None:
        """Callback used upon receiving an updated robot description.

        Args:
            robot_description (String): Robot description as defined in the
            robot's URDF file.
        """
        # TODO: For now, I'm going to forgo implementing this, as there aren't
        # any messages being published to the /ego_racecar_description topic
        with self.__robot_width_mutex:
            # Grab the robot width from the description here.
            # self.__robot_width = robot_description
            pass

    def get_robot_width(self) -> float:
        """Returns the width of the robot in meters.

        Returns:
            float: Robot width in meters.
        """
        with self.__robot_width_mutex:
            return self.__robot_width
        
    def publish_control(self, new_steering_angle: float, 
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
    
    # Maybe as a general rule of thumb (which sorta stems from stateful design:
    # separate the check from the control). That is, no state should have any
    # conditionally executed logic--therefore, if we're trying to model our
    # functions properly (with some hint of stateful design thrown in there),
    # then we should separate the logic from the actions. The logic should
    # determine which state we go into next--not what action we take here and
    # now!

    def __side_too_close(self, laser_scan: LaserScan) -> bool:
        """Checks whether the robot is too close to an obstacle or wall on
        either side. Takes the ranges on its side and compares them against a
        minimum distance threshold. 

        Args:
            laser_scan (LaserScan): LaserScan message from LiDAR containing the
            ranges that'll be evalauted.

        Returns:
            bool: True if the car is too close, False if not.
        """
        # Grab ranges from laser_scan and threshold value from parameters.
        ranges = laser_scan.ranges
        minimum_dist = self.__get_local_parameter("side_safety_dist_minimum_m")        
        # Look through the ranges between the left side indices to see if there
        # are any that fall under the threshold.
        for index in self.__left_side_index_range:
            if ranges[index] < minimum_dist:
                return True        
        # Look through the ranges between the right side indices to see if there
        # are any that fall under the threshold.
        for index in self.__right_side_index_range:
            if ranges[index] < minimum_dist:
                return True
        # Otherwise, return False, as the car isn't too close to an object or
        # wall on either side.

        # TODO: Am I sure that we don't need to also add the check somewhere
        # here that looks to see if we're currently turning?
        # I would think that it's not really necessary, as if this returns true,
        # we can just literally have it steer straight (steering angle=0),
        # whether we're already steering at that angle or not. UNLESS there's
        # some advantage to just letting the algorithm handling it at that
        # point.
        return False

    def __steer_straight(self):
        """Simple function that, when invoked, will publish a new drive message
        with steering angle zero and speed unchanged.
        """
        new_steering_angle = 0.0
        new_speed = self.__last_drive_message.drive.speed
        self.publish_control(new_steering_angle=new_steering_angle, new_velocity=new_speed)

    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        proc_ranges = ranges
        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        return None
    
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	    Naive: Choose the furthest point within ranges and go there
        """
        return None

    def __lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an
        AckermannDriveStamped Message.
        """
        ranges = data.ranges
        proc_ranges = self.preprocess_lidar(ranges)
        
        # TODO:
        #Find closest point to LiDAR

        #Eliminate all points inside 'bubble' (set them to zero) 

        #Find max length gap 

        #Find the best point in the gap 

        #Publish Drive message

        # TODO: My take on the steps for implementing FTG with disparity tweak +
        # wobble decrease tweak.

        # 1. First, establish the range of lidar values that we'll actually use
        #    (-90 to 90. We'll use the angles further to the side for side
        #    checking later). Could just pick indices for this (precompute
        #    these in constructor?). Set variables containing the idices for
        #    those further side values, as well (in the constructor).
        # 2. Before any disparity calculation, check for the corner case == are
        #    we too close too some safety distance to an object on our side AND
        #    is the LAST STEERING ANGLE we published != 0? If so, set steering
        #    angle to zero, don't change velocity, and move onto next timestep.
        # OTHERWISE:
        # 3. When we get a LaserScan message, looking only at the range we
        #    computed earlier: scan through for for disparities. How? Need some
        #    sort of disparity threshold? Compute disparity from one point to
        #    the next by taking abs(difference). If greater, then have to extend
        #    based on sign of difference. If diff negative, we extend the
        #    earlier (lower index) value to greater indices. If positive, then
        #    we extend the higher index value to lower indices. The amount we
        #    extend each by is == 1/2 width of car? (Make this another tunable
        #    parameter). Basically, you have to extend to the number of indices
        #    (angles) that, at the given distance, sum up to the 1/2 car width.
        #    I.e., circumference depnds on how far away you are! (radius).
        # 4. Once we have effectively generated only the feasible gaps using
        #    disparity + extension, choose the gap containing the greatest
        #    depth. Maybe as a step before this, we should generate a list /
        #    array of gap tuples, and then sort them by increasing distance
        #    (logn with heapify?). OR, just search through each gap and keep
        #    track of the largest.
        # 5. Choose the gap with the greatest depth. Choose the steering angle
        #    that corresponds to the middle of that gap. That will be the next
        #    published steering angle.
        # 6* Rahul says to limit/clamp all distance measurements to some maximum
        # value (like 3m or something) to mitigate the wobbling. However, prof
        # dolan made a point that, we're thinking that you shouldn't need to set
        # this maximum, but instead to just choose the gap with the greatest
        # depth, and then just follow the center of that gap. Maybe you get
        # better performance if you set the limit, but I'll try it without that
        # first. The beauty is, if I need to add that later, that's a numpy
        # one-liner with boolean-indexing!




def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()