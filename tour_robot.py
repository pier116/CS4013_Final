#!/usr/bin/env python

'''
Copyright (c) 2015, Mark Silliman
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endprint(user_input)
orse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

# TurtleBot must have minimal.launch & amcl_demo.launch
# running prior to starting this script
# For simulation: launch gazebo world & amcl_demo prior to run this script

import rospy
import itertools
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion

def manhattan_distance(node1, node2):
    x1, y1 = node1
    x2, y2 = node2
    return abs(x1 - x2) + abs(y1 - y2)

def total_distance(path, nodes):
    distance = 0
    for i in range(len(path) - 1):
        distance += manhattan_distance(nodes[path[i]], nodes[path[i + 1]])
    distance += manhattan_distance(nodes[path[-1]], nodes[path[0]])
    return distance

def tsp_manhattan(nodes):
    num_nodes = len(nodes)
    if num_nodes <= 2:
        return nodes

    node_labels = [chr(ord('A') + i) for i in range(num_nodes)]  # Create labels

    # Create a list of node labels (excluding the starting node, which is assumed to be the first one, 'A')
    node_indices = node_labels[1:]

    shortest_path = None
    shortest_distance = float('inf')

    for perm in itertools.permutations(node_indices):
        path = ['A'] + list(perm) + ['A']  # Start and end at 'A'
        distance = total_distance(path, nodes)
        if distance < shortest_distance:
            shortest_path = path
            shortest_distance = distance
    print(shortest_path)
    path = []
    for node in shortest_path:
        path.append(nodes[node])
    
    print(path)

    return path

#TODO
#def navigate_path(path):

class GoToPose():
    def __init__(self):

        self.goal_sent = False

        # What to do if shut down (e.g. Ctrl-C or failure)
        rospy.on_shutdown(self.shutdown)
        
        # Tell the action client that we want to spin a thread by default
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Wait for the action server to come up")

        # Allow up to 5 seconds for the action server to come up
        self.move_base.wait_for_server(rospy.Duration(5))

    def goto(self, pos, quat):

        # Send a goal
        self.goal_sent = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                        Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

        # Start moving
        self.move_base.send_goal(goal)

        # Allow TurtleBot up to 60 seconds to complete task
        success = self.move_base.wait_for_result(rospy.Duration(60)) 

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)

    def navigate_to_multiple_points(self, points):
        for point in points:
            position = {'x': point[0], 'y': point[1]}
            quaternion = {'r1': 0.000, 'r2': 0.000, 'r3': 0.000, 'r4': 1.000}

            rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
            success = self.goto(position, quaternion)

            if success:
                rospy.loginfo("Hooray, reached the desired pose")
            else:
                rospy.loginfo("The base failed to reach the desired pose")
            rospy.sleep(1)

    def create_highlight(self, node_name, x, y):
        highlights = open("highlights.txt", 'r')
        for row in highlights.readlines():
            print(row)
        highlights.close()
        

    
   
if __name__ == '__main__':
    try:
        rospy.init_node('nav_test', anonymous=False)
        navigator = GoToPose()
        print("Weclome to robot tour guide!")
        while(1):
            print('===================================================')
            print("Would you like a point-to-point tour or tour guide?")
            user_input = input("Enter 'p' for point to point, 't' for a tour,'r' to go back to its reacharging station, or 'q' to quit")
            #user_input = input()
            print(user_input)
            if user_input == 'p':
                print("Point-to-point mode activated")
                x = input("Pleas enter the x coordinate: ")
                y = input("Pleas enter the y coordinate: ")
                position = {'x': float(x), 'y' : float(y)}
                quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}

                rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
                success = navigator.goto(position, quaternion)

                if success:
                    rospy.loginfo("Hooray, reached the desired pose")
                else:
                    rospy.loginfo("The base failed to reach the desired pose")

                # Sleep to give the last log messages time to be sent
                rospy.sleep(1)
            elif user_input == 't':
                print("Tour guide mode activated!!")
                nodes = {'A': (-.362, 3.8), 'B': (.911, 4.6), 'C': (-.383, 5.56), 'D': (1.5, 5.71)}
                user_input = input("Would you like to do a tour of all four levels or a custom tour? 1 = tour of all four levels, 2 = custom tour")
                if user_input == '1':
                    path_one = {'A': (.254, -.287), 'B': (-.665, 1.87)} 
                    shortest_path = tsp_manhattan(path_one)
                    navigator.navigate_to_multiple_points(shortest_path)
                    # Sleep to give the last log messages time to be sent
                    rospy.sleep(1)
                    print("Changing levels")
                    path_two = {'A': (0.0701, 3.34), 'B': (0.0701, 3.34), 'C': (1.08, 4.84)}
                    shortest_path = tsp_manhattan(path_two)
                    navigator.navigate_to_multiple_points(shortest_path)
                    # Sleep to give the last log messages time to be sent
                    rospy.sleep(1)
                    
                    print("Changing levels")
                    path_three = {'A': (0.0701, 3.34), 'B': (1.86, 5.3), 'C': (1.88, 8.16), 'D': (0.2, 7.59), 'E': (-.272, 5.69), 'F': (1.08, 4.84), 'G': (1.08, 4.84)}
                    shortest_path = tsp_manhattan(path_three)
                    navigator.navigate_to_multiple_points(shortest_path)
                    # Sleep to give the last log messages time to be sent
                    rospy.sleep(1)
                    
                    print("Changing levels")
                    path_four = {'A': (-.0306, 5.83), 'B': (0.55, 9.48)}
                    shortest_path = tsp_manhattan(path_four)
                    navigator.navigate_to_multiple_points(shortest_path)
                    # Sleep to give the last log messages time to be sent
                    rospy.sleep(1)
                    print("Tour has ended")
                else:
                    points = {}
                    while(1):
                        user_input = raw_input("Would you like to add a point? 1 = Yes, 2 = No\n")
                        if user_input == '1':
                            node_name = raw_input("Please enter highlight name:\n")
                            x = raw_input("Please enter the x coordinate\n")
                            y = raw_input("Please enter the y coodinate\n")
                            points[node_name] = (float(x), float(y))
                            create_highlight(node_name, x, y)
                        else:
                            break
                    if len(points) == 2:
                        shortest_path = tsp_manhattan(points)
                        navigator.navigate_to_multiple_points([points[node] for node in shortest_path])
                        rospy.sleep(1)
                    else:
                        path = tsp_manhattan(points)
                        navigator.navigate_to_multiple_points(path)
                        rospy.sleep(1)
                    print("Custom tour has ended")
            elif user_input == 'r':
                print("Going back to recharging station")
                position = {'x': .254, 'y' : -.287}
                quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}

                rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
                success = navigator.goto(position, quaternion)

                if success:
                    rospy.loginfo("Hooray, reached the desired pose")
                else:
                    rospy.loginfo("The base failed to reach the desired pose")

                # Sleep to give the last log messages time to be sent
            elif user_input== 'q':
                print("Exiting tour robot mode......")
                print("Have a great day!!!")
                break

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")

