# rrt straight line path planner for mavsim_python
#
# mavsim_python
#     - Beard & McLain, PUP, 2012
#     - Last updated:
#         4/3/2019 - Brady Moon
#         4/11/2019 - RWB
#         3/31/2020 - RWB
from audioop import reverse
import re
from tracemalloc import start
import numpy as np
from message_types.msg_waypoints import MsgWaypoints
from chap11.draw_waypoints import DrawWaypoints
from chap12.draw_map import DrawMap
import pyqtgraph as pg
import pyqtgraph.opengl as gl


class RRTStraightLine:
    def __init__(self):
        self.segment_length = 300 # standard length of path segments
        self.plot_window = []
        self.plot_app = []

    def update(self, start_pose, end_pose, Va, world_map, radius):
        #generate tree
        tree = MsgWaypoints()
        #tree.type = 'straight_line'
        tree.type = 'fillet'
        # add the start pose to the tree
        tree.add(start_pose)

        # check to see if start_pose connects directly to end_pose
        if distance(end_pose, start_pose) <= self.segment_length: 
            print("ERROR: end path is already connected to start path")
            tree.add(end_pose)
            return tree
        else:
            # find path with minimum cost to end_node
            waypoints = find_minimum_path(tree, end_pose, world_map, self.segment_length)
            print("waypoints: \n", waypoints.ned)
            # waypoints = #smooth_path()
       
        return waypoints

    def extend_tree(self, tree, end_pose, Va, world_map):
        # extend tree by randomly selecting pose and extending tree toward that pose
        flag = True
        return flag

    def plot_map(self, world_map, tree, waypoints, smoothed_waypoints, radius):
        scale = 4000
        # initialize Qt gui application and window
        self.plot_app = pg.QtGui.QApplication([])  # initialize QT
        self.plot_window = gl.GLViewWidget()  # initialize the view object
        self.plot_window.setWindowTitle('World Viewer')
        self.plot_window.setGeometry(0, 0, 1500, 1500)  # args: upper_left_x, upper_right_y, width, height
        grid = gl.GLGridItem() # make a grid to represent the ground
        grid.scale(scale/20, scale/20, scale/20) # set the size of the grid (distance between each line)
        self.plot_window.addItem(grid) # add grid to viewer
        self.plot_window.setCameraPosition(distance=scale, elevation=50, azimuth=-90)
        self.plot_window.setBackgroundColor('k')  # set background color to black
        self.plot_window.show()  # display configured window
        #self.plot_window.raise_() # bring window to the front

        blue = np.array([[30, 144, 255, 255]])/255.
        red = np.array([[204, 0, 0]])/255.
        green = np.array([[0, 153, 51]])/255.
        DrawMap(world_map, self.plot_window)
        DrawWaypoints(waypoints, radius, blue, self.plot_window)
        DrawWaypoints(smoothed_waypoints, radius, red, self.plot_window)
        draw_tree(tree, green, self.plot_window)
        # draw things to the screen
        self.plot_app.processEvents()


def smooth_path(waypoints, world_map):
    # smooth the waypoint path
    smooth = [0]  # add the first waypoint

    # construct smooth waypoint path
    smooth_waypoints = MsgWaypoints()

    return smooth_waypoints


def find_minimum_path(tree, end_pose, world_map, segment_length):
    # find the lowest cost path to the end node
    # find nodes that connect to end_node
    connecting_nodes = []
   
    # # find minimum cost last node
    # idx = 

    # # construct lowest cost path order
    # path = 

    itt = 0

    last = tree.ned[:,0:1]

    while  not (distance(last, end_pose) <= segment_length):
    # while  not itt==100:

        p = random_pose(world_map, tree.ned[:,0:1])
        v_star, itt_star = closest_node(p, tree)
        v_plus = segment_length*(p - v_star)/np.linalg.norm(p - v_star)

        if not collision(v_star, v_plus, world_map):
            tree.add(v_plus + v_star)
            itt += 1
            connecting_nodes.append([itt_star, itt])
            last = v_plus + v_star
            
        if (distance(v_plus + v_star, end_pose) <= segment_length) and not collision(v_plus, end_pose, world_map):
            tree.add(end_pose)
            itt += 1
            connecting_nodes.append([itt_star, itt])
            last = end_pose
        
    print(connecting_nodes)
    # return tree

    path = []

    # find shortest path
    last_node = itt
    # for i in connecting_nodes:
    #     if i[1] == last_node:
    #         path.append(last_node)
    #         last_node = i[0]
    #         break

    while not last_node == 0:
        for i in connecting_nodes:
            if i[1] == last_node:
                path.append(last_node)
                last_node = i[0]
    
    path.reverse()
    print("path: \n", path)

    # construct waypoint path
    waypoints = MsgWaypoints()
    waypoints.type = 'fillet'

    for i in path:
        waypoints.add(tree.ned[:,i:i+1])

    print("waypoints: \n", waypoints.ned)
   
    return waypoints


def closest_node(p, tree):
    # print("clossest node ts: ")
    # print("tree: \n", tree.ned)
    # print("p: \n", p)

    tree_itt = 0
    close = tree.ned[:,tree_itt:tree_itt + 1]
    dist = np.linalg.norm(p - close)

    # print("starting dist: ", dist)

    for i in range(1, tree.num_waypoints - 1):
        temp = np.linalg.norm(p - tree.ned[:,i:i + 1])
        if temp < dist: 
            dist = temp
            close = tree.ned[:,i:i + 1]
            tree_itt = i
    #     print("new dist: ", temp)
    # print("end clossest node \n")
    return close, tree_itt

def random_pose(world_map, pd):
    # generate a random pose
    xy = np.random.rand(2)

    # print("random pose stuff")
    # print(xy)
    # print(xy[1])

    pose = np.array([[xy[0]*world_map.city_width], [xy[1]*world_map.city_width], [pd.item(2)]])
    # print("end random pose \n")
    return pose


def distance(start_pose, end_pose):
    # compute distance between start and end pose
    d = np.linalg.norm(end_pose - start_pose)
    return d


def collision(start_pose, end_pose, world_map):
    # check to see of path from start_pose to end_pose colliding with map
    num_points = 10
    points = points_along_path(start_pose, end_pose, num_points)
    width = world_map.building_width

    collision_flag = False

    for i in range(0, num_points):
        for j in range(0, world_map.num_city_blocks):
            n = world_map.building_north[0,j]
            e = world_map.building_east[0,j]
            if (points[i][0] > n-width and points[i][0] < n+width) and (points[i][1] > e-width and points[i][1] < e+width):
                if points[i][2] < world_map.building_height[0,j]:
                    collision_flag = True

    end = start_pose + end_pose

    if end.item(0) > 2000 or end.item(0) < 0 or end.item(1) > 2000 or end.item(1) < 0:
        collision_flag = True

    # if collision_flag:
    #     print("collided")
    return collision_flag


def height_above_ground(world_map, point):
    # find the altitude of point above ground level
    point_height = point.item(2)
    map_height = 0
    h_agl = point_height - map_height
    return h_agl


def draw_tree(tree, color, window):
    R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
    points = R @ tree.ned
    for i in range(points.shape[1]):
        line_color = np.tile(color, (2, 1))
        parent = int(tree.parent.item(i))
        line_pts = np.concatenate((column(points, i).T, column(points, parent).T), axis=0)
        line = gl.GLLinePlotItem(pos=line_pts,
                                 color=line_color,
                                 width=2,
                                 antialias=True,
                                 mode='line_strip')
        window.addItem(line)


def points_along_path(start_pose, end_pose, N):
    # returns points along path separated by Del
    deltaN = (start_pose.item(0) + end_pose.item(0))/N
    deltaE = (start_pose.item(1) + end_pose.item(1))/N

    points = np.zeros((N, 3))

    for i in range(0, N):
        points[i][0] = start_pose.item(0) + i*deltaN
        points[i][1] = start_pose.item(1) + i*deltaE
        points[i][2] = start_pose.item(2)
    
    return points


def column(A, i):
    # extracts the ith column of A and return column vector
    tmp = A[:, i]
    col = tmp.reshape(A.shape[0], 1)
    return col