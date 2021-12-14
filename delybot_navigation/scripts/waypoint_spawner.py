#!/usr/bin python3
# license removed for brevity
import os
import actionlib
import rospy
import json
from move_base_msgs.msg import  MoveBaseGoal, MoveBaseAction



### JSON UTILS ###

from dataclasses import dataclass
from typing import Any, List, TypeVar, Type, cast, Callable


T = TypeVar("T")


def from_float(x: Any) -> float:
    assert isinstance(x, (float, int)) and not isinstance(x, bool)
    return float(x)


def to_float(x: Any) -> float:
    assert isinstance(x, float)
    return x


def from_str(x: Any) -> str:
    assert isinstance(x, str)
    return x


def to_class(c: Type[T], x: Any) -> dict:
    assert isinstance(x, c)
    return cast(Any, x).to_dict()


def from_list(f: Callable[[Any], T], x: Any) -> List[T]:
    assert isinstance(x, list)
    return [f(y) for y in x]


@dataclass
class Pose:
    position_x: float
    position_y: float
    position_z: float
    orientation_x: float
    orientation_y: float
    orientation_z: float
    orientation_w: float

    @staticmethod
    def from_dict(obj: Any) -> 'Pose':
        assert isinstance(obj, dict)
        position_x = from_float(obj.get("position_x"))
        position_y = from_float(obj.get("position_y"))
        position_z = from_float(obj.get("position_z"))
        orientation_x = from_float(obj.get("orientation_x"))
        orientation_y = from_float(obj.get("orientation_y"))
        orientation_z = from_float(obj.get("orientation_z"))
        orientation_w = from_float(obj.get("orientation_w"))
        return Pose(position_x, position_y, position_z, orientation_x, orientation_y, orientation_z, orientation_w)

    def to_dict(self) -> dict:
        result: dict = {}
        result["position_x"] = to_float(self.position_x)
        result["position_y"] = to_float(self.position_y)
        result["position_z"] = to_float(self.position_z)
        result["orientation_x"] = to_float(self.orientation_x)
        result["orientation_y"] = to_float(self.orientation_y)
        result["orientation_z"] = to_float(self.orientation_z)
        result["orientation_w"] = to_float(self.orientation_w)
        return result


@dataclass
class GoalElement:
    name: str
    pose: Pose

    @staticmethod
    def from_dict(obj: Any) -> 'GoalElement':
        assert isinstance(obj, dict)
        name = from_str(obj.get("name"))
        pose = Pose.from_dict(obj.get("pose"))
        return GoalElement(name, pose)

    def to_dict(self) -> dict:
        result: dict = {}
        result["name"] = from_str(self.name)
        result["pose"] = to_class(Pose, self.pose)
        return result


def goal_from_dict(s: Any) -> List[GoalElement]:
    return from_list(GoalElement.from_dict, s)


def goal_to_dict(x: List[GoalElement]) -> Any:
    return from_list(lambda x: to_class(GoalElement, x), x)


def waypoint_spawner():

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    client.wait_for_server()

    movebase_goal = MoveBaseGoal()
    # Header
    movebase_goal.target_pose.header.stamp = rospy.Time.now()
    movebase_goal.target_pose.header.frame_id = "map"
    # Position
    movebase_goal.target_pose.pose.position.x = goal.pose.position_x
    movebase_goal.target_pose.pose.position.y = goal.pose.position_y
    movebase_goal.target_pose.pose.position.z = goal.pose.position_z
    # Orientation
    movebase_goal.target_pose.pose.orientation.x = goal.pose.orientation_x
    movebase_goal.target_pose.pose.orientation.y = goal.pose.orientation_y
    movebase_goal.target_pose.pose.orientation.z = goal.pose.orientation_z
    movebase_goal.target_pose.pose.orientation.w = goal.pose.orientation_w

    client.send_goal(movebase_goal)

    wait = client.wait_for_result()

    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()


if __name__ == '__main__':
    try:

        f = open (os.getenv('HOME') + '/catkin_ws/src/DelyBot/delybot_navigation/scripts/waypoint.json', "r")
        goal_list = goal_from_dict(json.loads(f.read()))

        rospy.loginfo(f"Waypoint list: {goal_list}")

        rospy.init_node('waypoint_spawner')

        while True:
            print("\nGOAL LIST:")
            for idx, goal in enumerate(goal_list):
                print(f"{idx}) {goal.name}")
            
            value = input("\nInsert goal index: ")
            
            if value.isdigit():
                value_parsed = int(value)

            while not( value.isdigit() and value_parsed in range(len(goal_list))):
                print(f"Error: the input must be an integer between 0 and {len(goal_list) - 1}")
                value = input("Insert goal index: ")
            
                if value.isdigit():
                    value_parsed = int(value)

            goal = goal_list[value_parsed]
            result = waypoint_spawner()

            if result:
                rospy.loginfo(f"\n({goal.name} x: {goal.pose.position_x}, y: {goal.pose.position_y}) Goal execution done!\n")

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




