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

import lidarutils

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
            "side_safety_dist_threshold_m": 0.15,
            "range_upper_bound_m": 3,
            "disparity_threshold_m": 0.3,
            
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

        # 1. Look through the ranges from the laser_scan on both the left and
        #    right past the 90 degree mark. Basically, want to look through the
        #    ranges between these angles and identify if there are any ranges
        #    that are below the "side_safety_dist_threshold_m" value.

        # TODO: So, this is the key: Write out the functionality first so that
        # I'm actually trying something and making progress, and then figure out
        # the best way to wrap up that functionality into a convenience
        # function later. Granted, this isn't great practice for designing an
        # interface, but I've also been wasting too much time not writing
        # code--so I kind of need to try this! Given my time constraint--it's
        # kinda the best option.

        # Get starting and ending indices for left side zone.
        left_start_angle_rad = math.radians(self.__get_local_parameter("gap_scan_angle_range_deg"))
        left_end_angle_rad = laser_scan.angle_max

        left_start_index = lidarutils.get_index_from_angle(angle_rad=left_start_angle_rad, laser_scan=laser_scan)
        left_end_index = lidarutils.get_index_from_angle(angle_rad=left_end_angle_rad, laser_scan=laser_scan)
        
        # Get Starting and ending indices for the right side zone.
        right_start_angle_rad = -math.degrees(self.__get_local_parameter("gap_scan_angle_range_deg"))
        right_end_angle_rad = laser_scan.angle_min
        
        right_start_index = lidarutils.get_index_from_angle(angle_rad=right_start_angle_rad, laser_scan=laser_scan)
        right_end_index = lidarutils.get_index_from_angle(angle_rad=right_end_angle_rad, laser_scan=laser_scan)



        # 2. If any of those values fall below the safety range, then return
        #    true. Otherwise, return false.

        pass


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