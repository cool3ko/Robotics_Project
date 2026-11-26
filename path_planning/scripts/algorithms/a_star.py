import rospy
import heapq

from algorithms.neighbors import find_neighbors


EPSILON = 1.0      
MOVE_COST = 1      




def to_xy(index, width):
    x = index % width
    y = index // width

    return (x, y)


def manhattan(a, b, width):
    ax, ay = to_xy(a, width)
    bx, by = to_xy(b, width)

    return abs(ax - bx) + abs(ay - by)




def a_star(start, goal, width, height, costmap, resolution, origin, grid_visualisation):


    if start == goal:
        return [start]


    open_list = []         
    closed_set = set()
    g_cost = {start: 0}
    parent = {}
    examined = 0
    counter = 0

    h = manhattan(start, goal, width)

    heapq.heappush(
        open_list,
        (EPSILON * h, h, counter, start)
    )

    while open_list:


        f, h, _, current = heapq.heappop(open_list)


        if current in closed_set:
            continue


        if current == goal:
            path = reconstruct(parent, start, goal)

            path_size =len(path)
            expanded = examined - path_size
            rospy.loginfo("Path size: %d", path_size)
            rospy.loginfo("Examined nodes: %d", examined)
            rospy.loginfo("Examined nodes: %d", expanded)

            return path


        closed_set.add(current)
        examined += 1

        grid_visualisation.set_color(current, "pale yellow")


        for n, _ in find_neighbors(current, width, height, costmap, MOVE_COST):

            if n in closed_set:
                continue

            new_g = g_cost[current] + MOVE_COST

            if n not in g_cost or new_g < g_cost[n]:

                g_cost[n] = new_g
                parent[n] = current

                h_n = manhattan(n, goal, width)
                f_n = new_g + EPSILON * h_n

                counter += 1

                heapq.heappush(
                    open_list,
                    (f_n, h_n, counter, n)
                )

                grid_visualisation.set_color(n, "orange")


    rospy.logwarn("No path found")

    return []


def reconstruct(parent, start, goal):

    path = [goal]
    node = goal

    while node != start:
        node = parent[node]
        path.append(node)

    return list(reversed(path))



