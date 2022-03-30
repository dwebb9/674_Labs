from unittest import installHandler
import numpy as np
import sys
sys.path.append('..')
from chap11.dubins_parameters import DubinsParameters
from message_types.msg_path import MsgPath


class PathManager:
    def __init__(self):
        # message sent to path follower
        self.path = MsgPath()
        # pointers to previous, current, and next waypoints
        self.ptr_previous = 0
        self.ptr_current = 1
        self.ptr_next = 2
        self.num_waypoints = 0
        self.halfspace_n = np.inf * np.ones((3,1))
        self.halfspace_r = np.inf * np.ones((3,1))
        # state of the manager state machine
        self.manager_state = 1
        
        self.manager_requests_waypoints = True
        self.dubins_path = DubinsParameters()

    def update(self, waypoints, radius, state):
        if waypoints.num_waypoints == 0:
            self.manager_requests_waypoints = True
        if self.manager_requests_waypoints is True \
                and waypoints.flag_waypoints_changed is True:
            self.manager_requests_waypoints = False
        if waypoints.type == 'straight_line':
            self.line_manager(waypoints, state)
        elif waypoints.type == 'fillet':
            self.fillet_manager(waypoints, radius, state)
        elif waypoints.type == 'dubins':
            self.dubins_manager(waypoints, radius, state)
        else:
            print('Error in Path Manager: Undefined waypoint type.')
        return self.path

    def initialize_pointers(self):
        if self.num_waypoints >= 3:
            self.ptr_previous = 0
            self.ptr_current = 1
            self.ptr_next = 2
        else:
            print('Error Path Manager: need at least three waypoints')

    def increment_pointers(self):
        if self.ptr_next == self.num_waypoints - 1:
            next = 0
        else:
            next = self.ptr_next + 1
        self.ptr_previous = self.ptr_current
        self.ptr_current = self.ptr_next
        self.ptr_next = next

    def inHalfSpace(self, pos):
        temp = np.transpose(pos - self.halfspace_r)
        if (np.dot(temp, self.halfspace_n)) >= 0: 
            return True
        else:
            return False

    def line_manager(self, waypoints, state):
        mav_pos = np.array([[state.north, state.east, -state.altitude]]).T
        # if the waypoints have changed, update the waypoint pointer

        if waypoints.flag_waypoints_changed is True: # initialize pointers, update flag, and set halfspace and calulate path
            self.num_waypoints = waypoints.num_waypoints
            self.initialize_pointers()
            waypoints.flag_waypoints_changed = False
            self.construct_line(waypoints)
            

        # state machine for line path
        if self.inHalfSpace(mav_pos): #incriment pointers, update half space, and update path. 
            self.increment_pointers()
            self.construct_line(waypoints)


    def construct_line(self, waypoints):
        # IF STATEMENT USED FOR SITUATIONS WHERE WAYPOINTS SHOULD NOT REPEAT TO ZERO WHEN PATH END IS REACHED. 
        # USE FOR MULTI-AGENT PATH PLANNING.

        # previous = waypoints.ned[:, self.ptr_previous:self.ptr_previous+1]
        # if self.ptr_current == 9999:
        #     current = 
        # else:
        #     current = 
        # if self.ptr_next == 9999:
        #     next = 
        # else:
        #     next = 


        #update halfspace variables

        previous = waypoints.ned[:, self.ptr_previous:self.ptr_previous+1]
        current = waypoints.ned[:, self.ptr_current:self.ptr_current+1]
        next = waypoints.ned[:, self.ptr_next:self.ptr_next+1]

        self.halfspace_r = current
        qiMin1 = (current - previous)/(np.linalg.norm(current - previous))
        qi = (next - current)/(np.linalg.norm(next - current))
        self.halfspace_n = (qiMin1 + qi)/np.linalg.norm((qiMin1 + qi))

        # print("in create line")
        # print("halfspace r: \n", self.halfspace_r)
        # print("halfspace n: \n", self.halfspace_n)

        #update path variables
        self.path.type = 'line'
        self.path.airspeed = waypoints.airspeed
        self.path.line_origin = previous
        self.path.line_direction = qiMin1



    def fillet_manager(self, waypoints, radius, state):
        mav_pos = np.array([[state.north, state.east, -state.altitude]]).T
        # if the waypoints have changed, update the waypoint pointer

        if waypoints.flag_waypoints_changed is True: # initialize pointers, update flag, and set halfspace and calulate path
            self.num_waypoints = waypoints.num_waypoints
            self.initialize_pointers()
            waypoints.flag_waypoints_changed = False
            self.manager_state = 1
            self.construct_fillet_line(waypoints, radius)

        # state machine for fillet path
        if self.inHalfSpace(mav_pos):
            if self.manager_state == 1:
                # print("changed to orbit")
                self.manager_state = 2
                self.construct_fillet_circle(waypoints, radius)

                
            elif self.manager_state == 2:
                # print("changed to line")
                self.manager_state = 1
                self.construct_fillet_line(waypoints, radius)
                self.increment_pointers()


    def construct_fillet_line(self, waypoints, radius):
        # if self.ptr_current == 9999:
        #     current = 
        # else:
        #     current = 
        # if self.ptr_next == 9999:
        #     next = 
        # else:
        #     next = 
        #update path variables
        previous = waypoints.ned[:, self.ptr_previous:self.ptr_previous+1]
        current = waypoints.ned[:, self.ptr_current:self.ptr_current+1]
        next = waypoints.ned[:, self.ptr_next:self.ptr_next+1]

        qiMin1 = (current - previous)/(np.linalg.norm(current - previous))
        qi = (next - current)/(np.linalg.norm(next - current))
        Q = np.arccos(np.transpose(-qiMin1)@qi)
        z = current - (radius/np.tan(Q/2))*qiMin1

        # print("line q values")
        # print("qiMin1: \n", qiMin1)
        # print("qi: \n", qi)
       
        self.halfspace_r = z
        self.halfspace_n = qiMin1

        #update path variables
        self.path.type = 'line'
        self.path.airspeed = waypoints.airspeed
        self.path.line_origin = previous
        self.path.line_direction = qiMin1
       

    def construct_fillet_circle(self, waypoints, radius):
        previous = waypoints.ned[:, self.ptr_previous:self.ptr_previous+1]
        # if self.ptr_current == 9999:
        #     current = 
        # else:
        #     current = 
        # if self.ptr_next == 9999:
        #     next = 
        # else:
        #     next = 

        previous = waypoints.ned[:, self.ptr_previous:self.ptr_previous+1]
        current = waypoints.ned[:, self.ptr_current:self.ptr_current+1]
        next = waypoints.ned[:, self.ptr_next:self.ptr_next+1]

        qiMin1 = (current - previous)/(np.linalg.norm(current - previous))
        qi = (next - current)/(np.linalg.norm(next - current))
        Q = np.arccos(np.transpose(-qiMin1)@qi)
        z = current + (radius/np.tan(Q/2))*qi
        c = current - (radius/np.sin(Q/2))*(qiMin1 - qi)/np.linalg.norm(qiMin1 - qi)
        direction = np.sign(qiMin1.item(0)*qi.item(1) - qiMin1.item(1)*qi.item(0))

        # print("orbit q values")
        # print("qiMin1: \n", qiMin1)
        # print("qi: \n", qi)

        self.halfspace_r = z
        self.halfspace_n = qi

        #update path variables
        self.path.type = 'orbit'
        self.path.airspeed = waypoints.airspeed
        self.path.orbit_center = c
        self.path.orbit_direction = direction
        self.path.orbit_radius = radius
         #update path variables

    def dubins_manager(self, waypoints, radius, state):
        mav_pos = np.array([[state.north, state.east, -state.altitude]]).T
        # if the waypoints have changed, update the waypoint pointer
        if waypoints.flag_waypoints_changed is True: # initialize pointers, update flag, and set halfspace and calulate path
            self.num_waypoints = waypoints.num_waypoints
            self.initialize_pointers()
            waypoints.flag_waypoints_changed = False
            self.manager_state = 1

            


        # state machine for dubins path
        if self.manager_state == 1:
            flag


    def construct_dubins_circle_start(self, waypoints, dubins_path):
        #update path variables
        self.path.type = 'orbit'
        self.path.airspeed = waypoints.airspeed
        self.path.orbit_center = dubins_path.center_s
        self.path.orbit_radius = dubins_path.radius
        self.path.orbit_direction = dubins_path.dir_s

        i = 0
        return i

    def construct_dubins_line(self, waypoints, dubins_path):
        #update path variables 
        i = 0
        return i

    def construct_dubins_circle_end(self, waypoints, dubins_path):
        #update path variables
        i = 0
        return i

