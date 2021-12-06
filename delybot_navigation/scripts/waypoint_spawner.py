#!/usr/bin python3
# license removed for brevity
import rospy
import json
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseActionGoal,MoveBaseGoal


### JSON UTILS ###
from dataclasses import dataclass
from typing import Any, List, TypeVar, Type, cast, Callable


T = TypeVar("T")


def from_int(x: Any) -> int:
    assert isinstance(x, int) and not isinstance(x, bool)
    return x


def to_class(c: Type[T], x: Any) -> dict:
    assert isinstance(x, c)
    return cast(Any, x).to_dict()


def from_list(f: Callable[[Any], T], x: Any) -> List[T]:
    assert isinstance(x, list)
    return [f(y) for y in x]


@dataclass
class Pose:
    position_x: int
    position_y: int
    position_z: int
    orientation_x: int
    orientation_y: int
    orientation_z: int
    orientation_w: int

    @staticmethod
    def from_dict(obj: Any) -> 'Pose':
        assert isinstance(obj, dict)
        position_x = from_int(obj.get("position_x"))
        position_y = from_int(obj.get("position_y"))
        position_z = from_int(obj.get("position_z"))
        orientation_x = from_int(obj.get("orientation_x"))
        orientation_y = from_int(obj.get("orientation_y"))
        orientation_z = from_int(obj.get("orientation_z"))
        orientation_w = from_int(obj.get("orientation_w"))
        return Pose(position_x, position_y, position_z, orientation_x, orientation_y, orientation_z, orientation_w)

    def to_dict(self) -> dict:
        result: dict = {}
        result["position_x"] = from_int(self.position_x)
        result["position_y"] = from_int(self.position_y)
        result["position_z"] = from_int(self.position_z)
        result["orientation_x"] = from_int(self.orientation_x)
        result["orientation_y"] = from_int(self.orientation_y)
        result["orientation_z"] = from_int(self.orientation_z)
        result["orientation_w"] = from_int(self.orientation_w)
        return result


@dataclass
class GoalElement:
    pose: Pose

    @staticmethod
    def from_dict(obj: Any) -> 'GoalElement':
        assert isinstance(obj, dict)
        pose = Pose.from_dict(obj.get("pose"))
        return GoalElement(pose)

    def to_dict(self) -> dict:
        result: dict = {}
        result["pose"] = to_class(Pose, self.pose)
        return result


def goal_from_dict(s: Any) -> List[GoalElement]:
    return from_list(GoalElement.from_dict, s)


def goal_to_dict(x: List[GoalElement]) -> Any:
    return from_list(lambda x: to_class(GoalElement, x), x)


### WAYPOINT SPAWNER ###
id = 0

def callback(data):

    if data.status_list: 
        status = data.status_list[0].status
    else:
        status = 3

    if not goal_list:
        #rospy.loginfo("Execution completed.")
        pass

    if status == 3 and goal_list:
        global id

        goal = goal_list.pop(0)

        pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)
        goal_msg = MoveBaseActionGoal()

        rospy.loginfo(data.status_list)

        # Header
        goal_msg.goal.target_pose.header.stamp = rospy.Time.now()
        goal_msg.goal.target_pose.header.frame_id = "map"
        # Position
        goal_msg.goal.target_pose.pose.position.x = goal.pose.position_x
        goal_msg.goal.target_pose.pose.position.y = goal.pose.position_y
        goal_msg.goal.target_pose.pose.position.z = goal.pose.position_z
        # Orientation
        goal_msg.goal.target_pose.pose.orientation.x = goal.pose.orientation_x
        goal_msg.goal.target_pose.pose.orientation.y = goal.pose.orientation_y
        goal_msg.goal.target_pose.pose.orientation.z = goal.pose.orientation_z
        goal_msg.goal.target_pose.pose.orientation.w = goal.pose.orientation_w

        goal_msg.goal_id.stamp = rospy.Time.now()
        goal_msg.goal_id.id = str(id)
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.header.frame_id = "map"

        rospy.loginfo(goal_msg)
        pub.publish(goal_msg)

        id = id + 1

def waypoint_spawner():
    
    rospy.init_node('waypoint_spawner', anonymous=True)

    rospy.Subscriber("/move_base/status", GoalStatusArray, callback)

    rospy.spin()
        
if __name__ == '__main__':
    try:

        f = open ('/home/giuseppe/catkin_ws/src/delybot/delybot_navigation/scripts/waypoint.json', "r")
        goal_list = goal_from_dict(json.loads(f.read()))

        rospy.loginfo(f"Waypoint list: {goal_list}")

        waypoint_spawner()
    except rospy.ROSInterruptException:
        pass

# std_msgs/Header header
#   uint32 seq
#   time stamp
#   string frame_id
# geometry_msgs/Pose pose
#   geometry_msgs/Point position
#     float64 x
#     float64 y
#     float64 z
#   geometry_msgs/Quaternion orientation
#     float64 x
#     float64 y
#     float64 z
#     float64 w

# rostopic info /move_base/status 
# Type: actionlib_msgs/GoalStatusArray




