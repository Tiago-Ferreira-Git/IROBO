import random
import math

from tkinter import *


from node import Node


WINDOW_X = 1200
WINDOW_Y = 600
RECTANGLE_WIDTH = 14

RRT_STEP_SIZE = 50
RRT_N = 100

RRT_ITERATION = 0
RRT_NBR_TRIES = 10_000

#middle
#RRT_INITIAL = (600, 300)
#top-left
RRT_INITIAL = (20, 20)

#right-top
RRT_FINAL   = (1180, 20)
#top-top
#RRT_FINAL   = (20, 20)
#left-bottom
#RRT_FINAL   = (20, 590)
#bottom-right
#RRT_FINAL   = (1180, 590)



_rrt = {}

# about 136 iterations
_obstacles = []

# about 671 iterations
# _obstacles = [
#     [(10, 60),(260, 180)],
#     [(300, 410), (1000, 550)],
# ]

# about 27_847 iterations
_obstacles = [
    [(10, 60),(560, 180)],
    [(300, 310), (1000, 550)],
    [(600, 0), (650, 310)],
]


## PRINT AND PLOT
##

def print_tree(tree):
    for (key, values) in tree.items():
        print(f'key={key} [', end="")
        for value in values:
            print(value, end="")
        print("]")


def plot_list_points(points_list):
    master_canvas = Tk()
    master_canvas.title("RRT")
    canvas = Canvas(master_canvas, width=WINDOW_X, height=WINDOW_Y)
    canvas.pack()
    #
    for values in points_list:
        print(values)
        canvas.create_rectangle(values[0], values[1], values[0]+RECTANGLE_WIDTH, values[1]+RECTANGLE_WIDTH)
    master_canvas.mainloop()


def draw_node(canvas, label, p, color="#FFFF00"):
    canvas.create_oval(p[0], p[1], p[0] + RECTANGLE_WIDTH, p[1] + RECTANGLE_WIDTH, fill=color)
    canvas.create_text(p[0] + RECTANGLE_WIDTH / 2, p[1] + RECTANGLE_WIDTH / 2, text=label)


def draw_line(canvas, node1, node2, color="#FFFF00"):
    canvas.create_line(node1.coords[0], node1.coords[1], node2.coords[0], node2.coords[1])


def draw_obstacles(canvas, obstables):
    for obstacle in obstables:
        canvas.create_rectangle(obstacle[0][0], obstacle[0][1], obstacle[1][0], obstacle[1][1], fill="#888888")


def plot_tree(tree):
    master_canvas = Tk()
    master_canvas.title("RRT")
    canvas = Canvas(master_canvas, width=WINDOW_X, height=WINDOW_Y)
    canvas.pack()
    #
    for (node, values) in tree.items():
        for link_node in values:
            draw_line(canvas, node, link_node)
        draw_node(canvas, node.label, node.coords)
    master_canvas.mainloop()


def plot_tree_with_org_and_dest(tree, p_origin, p_dest):
    master_canvas = Tk()
    master_canvas.title("RRT")
    canvas = Canvas(master_canvas, width=WINDOW_X, height=WINDOW_Y)
    canvas.pack()
    #
    draw_obstacles(canvas, _obstacles)
    for (node, values) in tree.items():
        for link_node in values:
            draw_line(canvas, node, link_node)
        draw_node(canvas, node.label, node.coords)

    draw_node(canvas, "S", RRT_INITIAL, "#00ff00")
    draw_node(canvas, "G", RRT_FINAL, "#ff0000")
    #
    master_canvas.mainloop()


## UTILS
##


def closest_point_to_dest(q_rand_list, q_dest):
    d_closest = 0
    q_closest = None
    for q_temp in q_rand_list:
        if not q_closest:
            d_closest = math.dist(q_temp, q_dest)
            q_closest = q_temp
        else:
            d_temp = math.dist(q_temp, q_dest)
            if (d_closest > d_temp):
                d_closest = d_temp
                q_closest = q_temp
    return q_closest


def closest_node_tree_to_dest(tree, q_dest):
    d_closest = 0
    node_closest = None
    for (key, linked_nodes_list) in tree.items():
        if not node_closest:
            d_closest = math.dist(key.coords, q_dest)
            node_closest = key
        else:
            d_temp = math.dist(key.coords, q_dest)
            if (d_closest > d_temp):
                d_closest = d_temp
                node_closest = key
    return node_closest


def no_obstacles(q_new):
    # 1. check does not leave room
    if q_new[0] < 0 or q_new[0] > WINDOW_X or q_new[1] < 0 or q_new[1] > WINDOW_Y :
        return False
    # 2. check for obstacles
    for box in _obstacles:
        #print("q_new", q_new , "box ", box)
        if q_new[0] > box[0][0] and q_new[0] < box[1][0] \
            and q_new[1] > box[0][1] and q_new[1] < box[1][1]:
            return False
    return True


## LIST AND TREE OPS
##


def add_to_tree(tree, node_closest, q_new):
    node_label=str(len(tree))
    node_new = Node(node_label, q_new)

    #print_tree(tree)

    tree[node_closest].append(node_new)
    tree[node_new]=[]


def expand_rrt(tree, q_closest_random, node_closest):

    d_random_closest = math.dist(q_closest_random, node_closest.coords)

    angle = math.atan2( (q_closest_random[1] - node_closest.coords[1]), (q_closest_random[0] - node_closest.coords[0]) )

    # q1=node_closest.coords[0] \
    #    + RRT_STEP_SIZE*math.cos(angle)
    # q2=node_closest.coords[1] \
    #    + RRT_STEP_SIZE*math.sin(angle)

    q1=node_closest.coords[0] \
       + RRT_STEP_SIZE*math.cos(angle)
    q2=node_closest.coords[1] \
       + RRT_STEP_SIZE*math.sin(angle)

    q_new = (q1, q2)

    if no_obstacles(q_new):
        add_to_tree(tree, node_closest, q_new)

    # print("-- R closest=", q_closest_random)
    # print("-- N closest=", node_closest, node_closest.coords)
    # print("-- d_random_closest=", d_random_closest)
    # print("-- cos=", math.cos((q_closest_random[0] - node_closest.coords[0])/d_random_closest))
    # print("-- sin=", math.sin((q_closest_random[1] - node_closest.coords[1])/d_random_closest))

    return tree


def build_rrt(tree):
    global RRT_ITERATION

    for count in range(RRT_NBR_TRIES):
        print(".", end="")

        q_rand_list=[]
        q_rand = (WINDOW_X * random.random(), WINDOW_Y * random.random())
        q_rand_list.append(q_rand)

        #for i in range(RRT_N):
            #print(q_rand)
            #q_rand = (WINDOW_X * random.random(), WINDOW_Y * random.random())
            #q_rand_list.append(q_rand)

        # find random neighbour
        q_closest_random = closest_point_to_dest(q_rand_list, RRT_FINAL)

        # find closest neighbour
        node_closest = closest_node_tree_to_dest(tree, q_closest_random)

        # GOAL ACHIRVED ?
        d_node_goal = math.dist(node_closest.coords, RRT_FINAL)
        if d_node_goal <= RRT_STEP_SIZE and no_obstacles(RRT_FINAL):
            add_to_tree(tree, node_closest, RRT_FINAL)
            print(f"DONE at iteration {count}!")
            break
        else:
            expand_rrt(tree, q_closest_random, node_closest)

        #plot_tree(tree)

    return



if __name__ == '__main__':
    print("## ")
    print("## RRT GENERATION")
    print("## ")
    print("## INITIAL POINT: ", RRT_INITIAL)
    print("## GOAL POINT: ", RRT_FINAL)
    print("## STEPS: ", RRT_STEP_SIZE)
    print("## ")

    # init tree
    start_node = Node()
    start_node.set_label("0")
    start_node.set_coords(RRT_INITIAL)
    _rrt[start_node]=[]
    # start simulation
    build_rrt(_rrt)

    plot_tree_with_org_and_dest(_rrt, RRT_INITIAL, RRT_FINAL)

    #print_tree(_rrt)

