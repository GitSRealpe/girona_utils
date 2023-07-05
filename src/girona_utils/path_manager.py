#! /usr/bin/env python

import rospy
import numpy as np

import actionlib
from actionlib_msgs.msg import GoalStatus
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose
from girona_utils.msg import PIDAction, PIDActionGoal, PIDActionFeedback


class Manager:
    error = 0
    done = False
    path: Path
    timer: rospy.Timer
    client = actionlib.SimpleActionClient("path_execution", PIDAction)

    def updateGoal(self, event):
        # print("updatin goal")

        if len(self.path.poses) != 0:
            if self.error < 0.2:
                req = PIDActionGoal(goal=self.path.poses.pop(0).pose)
                self.client.send_goal(req, feedback_cb=self.feedbackCB)
                self.error = 10
        else:
            self.done = True
            self.timer.shutdown()

        if self.client.get_state() == GoalStatus.PREEMPTED:
            self.client.cancel_all_goals()
            self.timer.shutdown()

    def feedbackCB(self, msg: PIDActionFeedback):
        feedback = msg.current

        self.error = np.linalg.norm(
            [feedback.position.x, feedback.position.y, feedback.position.z]
        )
        # print(self.error)

    def init(self):
        print("client initialized")
        self.path = rospy.wait_for_message("/planner/path_result", Path)
        print("path gotten")

        self.client.wait_for_server()
        self.error = 0

        self.timer = rospy.Timer(rospy.Duration(0.1), self.updateGoal)

        # req = PIDActionGoal(goal=path.poses[0].pose)


if __name__ == "__main__":
    rospy.init_node("path_client_py")
    man = Manager()
    man.init()
