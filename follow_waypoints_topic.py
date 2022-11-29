import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped 
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from nav2_msgs.action import FollowWaypoints
from std_msgs.msg import String
from tf_transformations import quaternion_from_euler
from math import radians
# from rclpy.duration import Duration # Handles time for ROS 2

class ClientFollowPoints(Node):

    def __init__(self):
        super().__init__('client_follow_points')
        self._client = ActionClient(self, FollowWaypoints, '/FollowWaypoints')
        
        ###################topic 구독
        self.create_subscription(String, '/place_object', self.call_back , 10)
        self.go_tb3 = String()
       
        ###############topic 발행
        self.turtle_pub = self.create_publisher(String, 'go_turtle', 10)
        self.msg = String()
        
    def call_back(self, msg):
        self.go_tb3 = msg.data
        #print(self.go_tb3)
        if self.go_tb3 == 'Find_stuff1':
            print("turtle is Carring stuff1!")
            print('-------------') 
            '''
            print('Goal : (x, y, z) = ({}, {}, {})'.format(
                self.rgoal.pose.position.x,
                self.rgoal.pose.position.y,
                self.rgoal.pose.position.z,
            ))
            print('-------------') 
            '''
            self.rgoal = PoseStamped()
            self.rgoal.header.frame_id = "map"
            self.rgoal.header.stamp.sec = 0
            self.rgoal.header.stamp.nanosec = 0
            
            #3번 AR마커(도착지점) 좌표
            self.rgoal.pose.position.z = 0.0403
            self.rgoal.pose.position.x = -2.14
            self.rgoal.pose.position.y = 0.0306
            
            #euler = (0.0, 0.0, radians(90))
            quart = quaternion_from_euler(radians(120), 0.0, 0.0, 'rzyx')
            self.rgoal.pose.orientation.x = quart[0]
            self.rgoal.pose.orientation.y = quart[1]
            self.rgoal.pose.orientation.z = quart[2]
            self.rgoal.pose.orientation.w = quart[3]
            #self.rgoal.pose.orientation.w = 1.0
            '''
            #4번 AR마커(시작지점) 좌표
            self.rgoal.pose.position.z = -0.00143
            self.rgoal.pose.position.x = 0.121
            self.rgoal.pose.position.y = -0.0068
            '''
            mgoal = [self.rgoal]
        
            self.send_points(mgoal)
    
        
    def send_points(self, points):
        msg = FollowWaypoints.Goal()
        msg.poses = points

        self._client.wait_for_server()
        self._send_goal_future = self._client.send_goal_async(msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
        self.msg.data = ('Arrival_at_marker3')
        self.turtle_pub.publish(self.msg)
        
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
        
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.missed_waypoints))

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.current_waypoint))
    
        
def main(args=None):
    rclpy.init(args=args)
    
    follow_points_client = ClientFollowPoints()
    print('client inited')

    rclpy.spin(follow_points_client)
