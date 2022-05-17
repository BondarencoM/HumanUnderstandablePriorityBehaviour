import rclpy
from rclpy.node import Node
import numpy as np
import cv2

from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.path_subscription = self.create_subscription(Path, '/plan', self.listener_callback, 10)
        self.path_subscription  # prevent unused variable warning
        self.map_subscription = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.map_subscription


    def listener_callback(self, msg: Path):
        i = 0
        window_x = 768
        window_y = 1024
        scaling_factor = 256                    #768/3 = 256
        map = cv2.imread('~/turtlebot3_ws/src/path_projection/path_projection/map.png', cv2.IMREAD_UNCHANGED)
        frame = np.full((window_x,window_y, 3),0).astype(np.uint8)
        subframe1 = np.full((window_x,window_y, 3),0).astype(np.uint8)
        subframe2 = np.full((window_x,window_y, 3),0).astype(np.uint8)
        finalframe = np.full((window_x,window_y, 3),0).astype(np.uint8)

        """for poseStamped in msg.poses:
            x = poseStamped.pose.position.x
            y = poseStamped.pose.position.y

            self.get_logger().info(f"{i+1}: x={x:.3f} y={y:.3f}")
            i = i + 1
            cv2.circle(frame, (int((y+0.5)*scaling_factor),int(x*scaling_factor)), 5, (0,0,255), -1)"""
        
        points = [(int((pose.pose.position.y+0.5)*scaling_factor),int(pose.pose.position.x*scaling_factor)) for pose in msg.poses]
        for pt1, pt2 in zip(points[0:-1], points[1:]):

            cv2.line(frame, pt1, pt2, (0, 0, 255), 2)
            self.get_logger().info(f"{i+1}: {pt1}->{pt2}")
        
        with open('msg.txt', 'w') as f:
            print(points, file=f)
            self.get_logger().info(f"{f}")
        
        self.get_logger().info("")
        subframe1 = cv2.rotate(frame, cv2.ROTATE_180)
        
        cv2.namedWindow('map', cv2.WND_PROP_FULLSCREEN)
        cv2.setWindowProperty('map', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

        cv2.imshow('map', subframe1)
        

        # path = cv2.imread('/home/student/turtlebot3_ws/path.png', cv2.IMREAD_UNCHANGED)
        # self.get_logger().info(map.shape)
        # addedimage = cv2.addWeighted(map, 0.5, path, 0.5, 0)
        # cv2.imshow("map", addedimage)
        cv2.waitKey(1)

    def map_callback(self, msg: OccupancyGrid):
        print('Frame:')
        print('Height:', msg.info.height)
        print('Width:', msg.info.width)
        print('')
        print('Orientation:')
        print('X:', msg.info.origin._position._x)
        print('Y:', msg.info.origin._position._y)
        print('X:', msg.info.origin._position._z)
        
        

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()