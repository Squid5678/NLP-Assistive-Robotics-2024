#!/usr/bin/env python

import replicate
import rospy
import os
import subprocess

from std_msgs.msg import String
from std_srvs.srv import Trigger

class InstructionParser(object):
    def __init__(self):
        rospy.init_node('instruction_parser')

        # set rate as 10Hz
        self.rate = 10
        self.current_file_path = os.path.dirname(os.path.abspath(__file__))
        self.domain_file_path = os.path.join(self.current_file_path, '../config/pick_n_place_nav_domain.pddl')
        self.problem_file_path = os.path.join(self.current_file_path, '../config/pick_n_place_nav_problem.pddl')
        self.solution_path = os.path.join(self.current_file_path, '../config/result.txt')

        # pre-load prompt template
        try:
            with open(os.path.join(self.current_file_path, '../config/prompt_template.txt'), 'r') as f:
                self.prompt = f.read()
        except FileNotFoundError:
            rospy.loginfo('Template file of prompt doesn not exist!')
        
        # Services
        rospy.wait_for_service('/task/trigger_task_execution')
        self.task_manager_service = rospy.ServiceProxy('/task/trigger_task_execution', Trigger)

        # Subscribers
        self.ins_sub = rospy.Subscriber('/task/instruction_input', String, self.instruction_callback)
    
    def instruction_callback(self, data):
        instruction = data.data
        rospy.loginfo("Receive the human instruction: {}".format(instruction))

        # write domain definition in prompt
        domain = self.load_domain_definition()
        self.prompt = self.prompt.replace('domain_definition', domain)

        # TBD: replace exact initial specification in prompt
        pddl_problem_wo_goal = self.pddl_problem_parsing()
        self.prompt = self.prompt.replace('test_problem_definition', pddl_problem_wo_goal)

        # replace exact human instruction in prompt 
        self.prompt = self.prompt.replace("test_human_instruction", instruction)

        # call replicate to interpret human instruction
        output = replicate.run( 
            "replicate/llama-2-70b-chat:02e509c789964a7ea8736978a43525956ef40397be9033abf9fd2badfe68c9e3",
            input={"prompt": self.prompt} 
        )
        goal_specification = ''
        for item in output:
            goal_specification += item
        
        rospy.loginfo("Parse received human instruction into PDDL goal specification: {}".format(goal_specification))

        # write the problem
        self.write_pddl_problem(pddl_problem_wo_goal, goal_specification)

        # call PDDL solver
        if self.pddl_solving():
            # trigger the service of task manager
            self.trigger_task_manager()
        else:
            rospy.loginfo("Solver can't find a solution, please give more complete instruction.")

    def load_domain_definition(self):
        domain = ''
        with open(os.path.join(self.current_file_path, '../config/pick_n_place_nav_domain.pddl'), 'r') as f:
            domain = f.read()
        
        return domain

    def write_pddl_problem(self, problem_wo_goal, goal):
        with open(self.problem_file_path, 'w') as f:
            problem_list = problem_wo_goal.split('\n')
            for i, s in enumerate(problem_list):
                # right bracket locates at the last 2 rather than the last 1
                if i == len(problem_list) - 2:
                    # insert goal specification
                    goal_list = goal.split('\n')
                    processed_goal_list = self.llm_output_postprocess(goal_list)
                    for sg in processed_goal_list:
                        f.write(' '*4 + sg + '\n')
                    f.write(s)
                else:
                    f.write(s + '\n')

    def llm_output_postprocess(self, output_list):
        out = []

        i = 0
        # skip all other unrelated outputs
        while i < len(output_list):
            if output_list[i] and output_list[i][0] == "(":
                break
            i += 1

        while i < len(output_list):            
            out.append(output_list[i])
            i += 1
        
        return out

    def pddl_problem_parsing(self):
        problem = ''
        with open(os.path.join(self.current_file_path, '../config/test_initial_specification.pddl'), 'r') as f:
            problem = f.read()
        return problem

    def trigger_task_manager(self):
        try:
            response = self.task_manager_service()
            if response.success:
                rospy.loginfo("Service call succeeded: %s", response.message)
            else:
                rospy.logwarn("Service call failed: %s", response.message)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

    def pddl_solving(self):
        # call PDDL solver to solve written problem
        process_path = os.path.join(self.current_file_path, '../../../downward/fast-downward.py')
        subprocess.call(
            process_path + ' ' + self.domain_file_path + ' ' + self.problem_file_path + ' --search "lazy_greedy([ff()], preferred=[ff()])"',
            shell=True
        )
        
        # append plan to list
        plan = []
        if os.path.exists('./sas_plan'):
            with open('./sas_plan') as f:
                for i, line in enumerate(f.readlines()[:-1]):
                    # List of words in the current line of the sas_plan
                    elements = line.split('\n')[0]
                    elements = elements[1:-1].split()

                    plan.append(' '.join(elements))
            # delete file
            os.remove('./sas_plan')

            # save the solution
            with open(self.solution_path, 'w') as f:
                for sub_task in plan:
                    f.write(sub_task + '\n')
            
            return True
        else:
            return False
        
    def main(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            rate.sleep()
        

if __name__ == '__main__':
    try:
        node = InstructionParser()
        rospy.loginfo('Launch instruction parser which transforms human instruction into PDDL goal specification')
        node.main()
    except KeyboardInterrupt:
        rospy.loginfo('interrupt received, so shutting down')
