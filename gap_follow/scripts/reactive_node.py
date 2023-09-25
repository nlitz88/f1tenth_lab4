import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

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

        # Set up parameters according to backend robotics parameter file
        # tutorial.

        # TODO: Subscribe to LIDAR
        # TODO: Publish to drive

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

    def lidar_callback(self, data):
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