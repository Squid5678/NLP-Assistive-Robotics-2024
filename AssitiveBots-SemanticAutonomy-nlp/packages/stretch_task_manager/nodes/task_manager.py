#!/usr/bin/env python

import rospy
import os

from std_srvs.srv import Trigger
from std_msgs.msg import String, Bool

class TaskManager(object):
    def __init__(self):
        rospy.init_node('task_manager')

        # set rate as 10Hz
        self.rate = 10

        self.PARSED_TASK_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../stretch_task_parsing/config/result.txt')

        # store parsed tasks
        self.plan = []      

        # publisher for navigation and manipulation manager
        self.nav_pub = rospy.Publisher('/task/nav_manager', String, queue_size=10)
        self.mani_pub = rospy.Publisher('/task/mani_manager', String, queue_size=10)

        # subscriber for checking if navigation and manipulation tasks are finished
        self.nav_task_done = False
        self.mani_task_done = False
        self.nav_finish_sub = rospy.Subscriber('/task/nav_task_done', Bool, self.nav_task_callback)
        self.mani_finish_sub = rospy.Subscriber('/task/mani_task_done', Bool, self.mani_task_callback)
    
    def nav_task_callback(self, data):
        self.nav_task_done = data.data
    
    def mani_task_callback(self, data):
        self.mani_task_done = data.data

    def load_parsed_task(self):
        rospy.loginfo('Load parsed tasks')

        with open(self.PARSED_TASK_PATH, 'r') as f:
            line = f.readline()
            while line:
                items = line.split()
                single_task = []
                for item in items:
                    single_task.append(item)
                self.plan.append(single_task)
                line = f.readline()

    def trigger_task_plan_callback(self, request):
        rospy.loginfo('Receive the request of executing tasks')

        # load the parsed task consisting of navigation and manipulation tasks
        self.load_parsed_task()

        for plan in self.plan:
            # navigation task
            if "navigate" in plan[0]:
                rospy.loginfo("Navigation task")
                self.send_request(self.nav_pub, plan)

                while not self.nav_task_done:
                    continue

                self.nav_task_done = False
            # manipulation task
            else:
                rospy.loginfo("Manipulation task")
                self.send_request(self.mani_pub, plan)

                while not self.mani_task_done:
                    continue
                    
                self.mani_task_done = False
        
        # reset
        self.plan = []    

    def send_request(self, pub, plan):
        pub.publish(" ".join(plan))

    def main(self):
        self.trigger_grasp_object_service = rospy.Service('/task/trigger_task_execution',
                                                           Trigger,
                                                           self.trigger_task_plan_callback)
        
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    try:
        node = TaskManager()
        rospy.loginfo('Launch task manager')
        node.main()
    except KeyboardInterrupt:
        rospy.loginfo('interrupt received, so shutting down')
