#!/usr/bin/env python3
##
# @file complete_sequence.py
#
# @Create3 robot assignment
#
# @program description
# This program uses a single actionClient class that is called to instantiate new nodes
# to perform work/commands/tasks from instructions given in main using goals
#
# @Note: This program only uses actions to complete the sequence

# Imports
import rclpy
import time
import math

from rclpy.action import ActionClient
from rclpy.node import Node

from irobot_create_msgs.action import Undock
from irobot_create_msgs.action import DockServo
from irobot_create_msgs.action import DriveDistance
from irobot_create_msgs.action import DriveArc
from irobot_create_msgs.action import AudioNoteSequence
from irobot_create_msgs.msg import AudioNoteVector, AudioNote
from builtin_interfaces.msg import Duration
# End Imports

# actionClient Class
#
# Used to create new nodes
#
class actionClient(Node):
    def __init__(self, nodeName, actionType, actionName):
        self.node = rclpy.create_node(nodeName)
        self.action_client = ActionClient(self.node, actionType, f"/create3_0561/{actionName}")

    # Receives Goals from main and spins until callback is received that task is done
    def send_goal(self, goal):
        goal_msg = goal

        self.action_client.wait_for_server()

        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)
        self.result = None
        # Spin till callback received
        while(self.result == None):
            rclpy.spin_once(self.node)

    # Callback for when goal has been accepted/rejected
    def goal_response_callback(self, future):
        goal_handle = future.result()
    
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)
    # Callback for when the task is complete
    def get_result_callback(self, future):
        self.result = future.result().result

def main(args=None):
    """! Main program entry. """

    # Initialize Nodes
    rclpy.init(args=args)

    undockAction = actionClient('undockAction', Undock, 'undock')
    goal = Undock.Goal()
    result = undockAction.send_goal(goal)

    # Drive Forward 2 meters
    driveAction = actionClient('driveAction', DriveDistance, 'drive_distance')
    goal = DriveDistance.Goal()
    goal.distance = 1.9
    goal.max_translation_speed = 0.5
    result = driveAction.send_goal(goal)

    # Turn 135 deg
    turnAction = actionClient('turnAction', DriveArc, 'drive_arc')
    goal = DriveArc.Goal()
    goal.angle = (135 * math.pi) / 180
    goal.max_translation_speed = 0.3
    result = turnAction.send_goal(goal)
    
    # Drive Forward .8 meters
    goal = DriveDistance.Goal()
    goal.distance = 0.8
    goal.max_translation_speed = 0.3
    result = driveAction.send_goal(goal)

    # Turn 120 deg
    goal = DriveArc.Goal()
    goal.angle = (120 * math.pi) / 180
    goal.max_translation_speed = 0.3
    result = turnAction.send_goal(goal)

    # Drive forward .8 meters
    goal = DriveDistance.Goal()
    goal.distance = 0.8
    goal.max_translation_speed = 0.3
    result = driveAction.send_goal(goal)

    # Turn 360 deg
    goal = DriveArc.Goal()
    goal.angle = (360 * math.pi) / 180
    goal.max_translation_speed = 0.3
    result = turnAction.send_goal(goal)
    
    # Turn -130 deg (point at dock)
    goal = DriveArc.Goal()
    goal.angle = (-120 * math.pi) / 180
    goal.max_translation_speed = 0.3
    result = turnAction.send_goal(goal)
    
    # Drive forward .9 meters
    goal = DriveDistance.Goal()
    goal.distance = 0.9
    goal.max_translation_speed = 0.3
    result = driveAction.send_goal(goal)

    # Initiate dock action
    dockAction = actionClient('dockAction', DockServo, 'dock')
    goal = DockServo.Goal()
    result = dockAction.send_goal(goal)
    
    # Play Happy sound
    happySounds = actionClient('happySounds', AudioNoteSequence, 'audio_note_sequence')
    goal = AudioNoteSequence.Goal()
    frequencies = [392, 523, 587, 784]
    notes = [AudioNote() for x in frequencies]
    for i in range(len(notes)):
        notes[i].frequency = frequencies[i]
        notes[i].max_runtime = Duration()
        notes[i].max_runtime.sec = 0
        notes[i].max_runtime.nanosec = 355000000
    note_sequence = AudioNoteVector()
    note_sequence.notes = notes
    goal.iterations = 1
    goal.note_sequence = note_sequence
    result = happySounds.send_goal(goal)
    
    # Shutdown nodes
    rclpy.shutdown()

if __name__ == '__main__':
    main()
