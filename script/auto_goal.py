import rospy
import random
from move_base_msgs.msg import MoveBaseActionGoal
from visualization_msgs.msg import Marker
from move_base_msgs.msg import MoveBaseActionResult


class goal_pub():
    def __init__(self):
        self.goal_id = 0
        print("reached?")
        self.marker = Marker()
        self.marker.type = Marker.SPHERE
        self.marker.action = Marker.ADD
        self.marker.scale.x = 10
        self.marker.scale.y = 10
        self.marker.scale.z = 10
        self.marker.color.a = 1
        self.marker.pose.orientation.w = 1.0
        self.marker.pose.position.x = 2
        self.marker.pose.position.y = 2
        self.marker.pose.position.z = 2
        self.marker.header.frame_id = '/map'
        self.goal_message = MoveBaseActionGoal()
        
        rospy.init_node("goal_generator", anonymous=False)
        rospy.loginfo("Goal Generator initiated")
        # rospy.on_shutdown(self.shutdown)
        rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.goalCallback)
        self.rbt_goal = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size = 2)
        self.rbt_goal.publish(self.goal_message)
        self.vis_goal = rospy.Publisher('/visualization_marker', Marker, queue_size= 2)
        self.vis_goal.publish(self.marker)
        rospy.loginfo(self.marker)
        rospy.spin()

    def goalCallback(self, data):
        self.goal_id += 1
        self.goal_message.goal_id = self.goal_id
        x = random.randint(-6,7)
        y = random.randint(-6,8)
        self.goal_message.goal.target_pose.pose.position.x = x
        self.goal_message.goal.target_pose.pose.position.y = y
        self.goal_message.goal.target_pose.pose.position.z = 0
        self.rbt_goal.publish(self.goal_message)
        self.marker.pose.position.x = x
        self.marker.pose.position.y = y
        self.vis_goal.publish(marker)

    def shutdown(self):
        print("here?")
        rospy.loginfo("auto goal terminating")


if __name__ == "__main__":
    # try:
    a = goal_pub()
    # except:
    #     print("shit happened")
    #     rospy.loginfo("auto goal generator terminated;")