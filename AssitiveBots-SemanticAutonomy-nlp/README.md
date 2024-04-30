# AssitiveBots-SemanticAutonomy
This branch is for NLP team. The goal is to utilize LLM and PDDL to perform high-level task planning. 
For example, given the instruction of "I am hungry and bring me an apple", parsed sequential actions 
should be 1. navigate to the apple, 2. grasp the apple, 3. navigate to the human user and 4. place
the apple on the hand.

## Installation
Virtual environment like conda is highly recommended. 
```bash
conda create -n NAME python=3.8
pip install replicate
```
Meanwhile you will need to have [ROS](https://wiki.ros.org/noetic/Installation/Ubuntu) installed.

## Usage
1. Run master node
```bash
roscore
```
2. Launch the task manager
```bash
python packages/stretch_task_manager/nodes/task_manager.py
```
3. Launch the instruction parser
```bash
python packages/stretch_task_parsing/scripts/instruction_parser.py
```
4. Input the instruction via terminal
```bash
rostopic pub -1 /task/instruction_input std_msgs/String "data: 'Move the coke from dining_table to sink in the kitchen'"
```
