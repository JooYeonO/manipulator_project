import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped 
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from nav2_msgs.action import FollowWaypoints
from std_msgs.msg import String
# from rclpy.duration import Duration # Handles time for ROS 2

class ClientFollowPoints(Node):

    def __init__(self):
        super().__init__('client_follow_points')
        self._client = ActionClient(self, FollowWaypoints, '/FollowWaypoints')
        
        ###################topic 구독
        self.create_subscription(String, '/place_object', self.callback , 10)
        self.place_object = String()

    def callback(self, msg):
        self.place_object = mag.data
        
    	if msg.data == 'put'
            msg = FollowWaypoints.Goal()
            msg.poses = msg
    '''
    def send_points(self, points):
        msg = FollowWaypoints.Goal()
        msg.poses = points
    '''
        self._client.wait_for_server()
        self._send_goal_future = self._client.send_goal_async(msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

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

    rgoal = PoseStamped()
    rgoal.header.frame_id = "map"
    rgoal.header.stamp.sec = 0
    rgoal.header.stamp.nanosec = 0
    '''
    rgoal.pose.position.z = 0.0
    rgoal.pose.position.x = 4.15
    rgoal.pose.position.y = -0.37
    '''
    #'''
    rgoal.pose.position.z = 0.0
    rgoal.pose.position.x = 2.94
    rgoal.pose.position.y = -0.917
    #'''
    rgoal.pose.orientation.w = 1.0
    print(rgoal)
    mgoal = [rgoal]

    follow_points_client.send_points(mgoal)

    rclpy.spin(follow_points_client)
