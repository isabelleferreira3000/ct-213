from grid import Node, NodeGrid
from math import inf
import heapq


class PathPlanner(object):
    """
    Represents a path planner, which may use Dijkstra, Greedy Search or A* to plan a path.
    """
    def __init__(self, cost_map):
        """
        Creates a new path planner for a given cost map.

        :param cost_map: cost used in this path planner.
        :type cost_map: CostMap.
        """
        self.cost_map = cost_map
        self.node_grid = NodeGrid(cost_map)

    @staticmethod
    def construct_path(goal_node):
        """
        Extracts the path after a planning was executed.

        :param goal_node: node of the grid where the goal was found.
        :type goal_node: Node.
        :return: the path as a sequence of (x, y) positions: [(x1,y1),(x2,y2),(x3,y3),...,(xn,yn)].
        :rtype: list of tuples.
        """
        node = goal_node
        # Since we are going from the goal node to the start node following the parents, we
        # are transversing the path in reverse
        reversed_path = []
        while node is not None:
            reversed_path.append(node.get_position())
            node = node.parent
        return reversed_path[::-1]  # This syntax creates the reverse list

    def dijkstra(self, start_position, goal_position):
        """
        Plans a path using the Dijkstra algorithm.

        :param start_position: position where the planning stars as a tuple (x, y).
        :type start_position: tuple.
        :param goal_position: goal position of the planning as a tuple (x, y).
        :type goal_position: tuple.
        :return: the path as a sequence of positions and the path cost.
        :rtype: list of tuples and float.
        """
        # Todo: implement the Dijkstra algorithm
        start_node = self.node_grid.get_node(start_position[0], start_position[1])
        goal_node = self.node_grid.get_node(goal_position[0], goal_position[1])
        print("start_node: ("+str(start_node.get_position()[0])+", "+str(start_node.get_position()[1])+")")
        print("goal_node: (" + str(goal_node.get_position()[0]) + ", " + str(goal_node.get_position()[1]) + ")")

        pq = []
        start_node.f = 0
        heapq.heappush(pq, (start_node.f, start_node))

        while pq:
            f, current_node = heapq.heappop(pq)
            current_node.closed = True
            print("------------")
            print("CURRENT NODE: ("+str(current_node.get_position()[0])+", "+str(current_node.get_position()[1])+")")
            if current_node == goal_node:
                break

            current_node_i, current_node_j = current_node.get_position()
            for successor in self.node_grid.get_successors(current_node_i, current_node_j):
                successor_node = self.node_grid.get_node(successor[0], successor[1])

                if not successor_node.closed and \
                        successor_node.f > current_node.f + self.cost_map.get_edge_cost(current_node.get_position(),
                                                                                        successor_node.get_position()):
                    successor_node.f = current_node.f + self.cost_map.get_edge_cost(current_node.get_position(),
                                                                                    successor_node.get_position())
                    successor_node.parent = current_node
                    print("parent of (" +
                          str(successor_node.get_position()[0])+", " +
                          str(successor_node.get_position()[1])+"): (" +
                          str(successor_node.parent.get_position()[0])+", " +
                          str(successor_node.parent.get_position()[1])+")")
                    if goal_node.parent:
                        print("PARENT OF GOAL: (" +
                              str(goal_node.parent.get_position()[0])+", " +
                              str(goal_node.parent.get_position()[1])+")")
                    else:
                        print("GOAL WITHOUT PARENT")
                    heapq.heappush(pq, (successor_node.f, successor_node))
        # The first return is the path as sequence of tuples (as returned by the method construct_path())
        # The second return is the cost of the path

        path = self.construct_path(goal_node)
        print("final: "+str(path))
        self.node_grid.reset()
        return path, goal_node.f  # Feel free to change this line of code

    def greedy(self, start_position, goal_position):
        """
        Plans a path using greedy search.

        :param start_position: position where the planning stars as a tuple (x, y).
        :type start_position: tuple.
        :param goal_position: goal position of the planning as a tuple (x, y).
        :type goal_position: tuple.
        :return: the path as a sequence of positions and the path cost.
        :rtype: list of tuples and float.
        """
        # Todo: implement the Greedy Search algorithm
        # The first return is the path as sequence of tuples (as returned by the method construct_path())
        # The second return is the cost of the path
        start_node = self.node_grid.get_node(start_position[0], start_position[1])
        goal_node = self.node_grid.get_node(goal_position[0], goal_position[1])
        print("start_node: (" + str(start_node.get_position()[0]) + ", " + str(start_node.get_position()[1]) + ")")
        print("goal_node: (" + str(goal_node.get_position()[0]) + ", " + str(goal_node.get_position()[1]) + ")")

        pq = []
        start_node.f = start_node.distance_to(goal_node.get_position()[0], goal_node.get_position()[1])
        heapq.heappush(pq, (start_node.f, start_node))

        while pq:
            f, current_node = heapq.heappop(pq)
            current_node.closed = True
            print("------------")
            print("CURRENT NODE: ("+str(current_node.get_position()[0])+", "+str(current_node.get_position()[1]) + ")")
            if current_node == goal_node:
                break

            current_node_i, current_node_j = current_node.get_position()
            for successor in self.node_grid.get_successors(current_node_i, current_node_j):
                successor_node = self.node_grid.get_node(successor[0], successor[1])

                if not successor_node.closed:
                    successor_node.parent = current_node

                    successor_node.f = successor_node.distance_to(goal_node.get_position()[0], goal_node.get_position()[1])
                    heapq.heappush(pq, (successor_node.f, successor_node))

                    print("parent of (" +
                          str(successor_node.get_position()[0]) + ", " +
                          str(successor_node.get_position()[1]) + "): (" +
                          str(successor_node.parent.get_position()[0]) + ", " +
                          str(successor_node.parent.get_position()[1]) + ")")
                    if goal_node.parent:
                        print("PARENT OF GOAL: (" +
                              str(goal_node.parent.get_position()[0]) + ", " +
                              str(goal_node.parent.get_position()[1]) + ")")
                    else:
                        print("GOAL WITHOUT PARENT")

        # The first return is the path as sequence of tuples (as returned by the method construct_path())
        # The second return is the cost of the path

        path = self.construct_path(goal_node)
        print("final: " + str(path))
        self.node_grid.reset()
        return path, goal_node.f  # Feel free to change this line of code

    def a_star(self, start_position, goal_position):
        """
        Plans a path using A*.

        :param start_position: position where the planning stars as a tuple (x, y).
        :type start_position: tuple.
        :param goal_position: goal position of the planning as a tuple (x, y).
        :type goal_position: tuple.
        :return: the path as a sequence of positions and the path cost.
        :rtype: list of tuples and float.
        """
        # Todo: implement the A* algorithm
        start_node = self.node_grid.get_node(start_position[0], start_position[1])
        goal_node = self.node_grid.get_node(goal_position[0], goal_position[1])
        print("start_node: (" + str(start_node.get_position()[0]) + ", " + str(start_node.get_position()[1]) + ")")
        print("goal_node: (" + str(goal_node.get_position()[0]) + ", " + str(goal_node.get_position()[1]) + ")")

        pq = []
        start_node.g = 0
        start_node.f = start_node.distance_to(goal_node.get_position()[0], goal_node.get_position()[0])
        heapq.heappush(pq, (start_node.f, start_node))

        while pq:
            f, current_node = heapq.heappop(pq)
            current_node.closed = True
            print("------------")
            print("CURRENT NODE: (" + str(current_node.get_position()[0]) + ", " + str(
                current_node.get_position()[1]) + ")")
            if current_node == goal_node:
                break

            current_node_i, current_node_j = current_node.get_position()
            for successor in self.node_grid.get_successors(current_node_i, current_node_j):
                successor_node = self.node_grid.get_node(successor[0], successor[1])

                if not successor_node.closed and \
                        successor_node.f > current_node.g +\
                        self.cost_map.get_edge_cost(current_node.get_position(), successor_node.get_position()) +\
                        successor_node.distance_to(goal_node.get_position()[0], goal_node.get_position()[0]):
                    successor_node.g = current_node.g + self.cost_map.get_edge_cost(current_node.get_position(),
                                                                                    successor_node.get_position())
                    successor_node.f = successor_node.g + successor_node.distance_to(goal_node.get_position()[0],
                                                                                     goal_node.get_position()[0])
                    successor_node.parent = current_node
                    print("parent of (" +
                          str(successor_node.get_position()[0]) + ", " +
                          str(successor_node.get_position()[1]) + "): (" +
                          str(successor_node.parent.get_position()[0]) + ", " +
                          str(successor_node.parent.get_position()[1]) + ")")
                    if goal_node.parent:
                        print("PARENT OF GOAL: (" +
                              str(goal_node.parent.get_position()[0]) + ", " +
                              str(goal_node.parent.get_position()[1]) + ")")
                    else:
                        print("GOAL WITHOUT PARENT")
                    heapq.heappush(pq, (successor_node.f, successor_node))
        # The first return is the path as sequence of tuples (as returned by the method construct_path())
        # The second return is the cost of the path

        path = self.construct_path(goal_node)
        print("final: " + str(path))
        self.node_grid.reset()
        return path, goal_node.f  # Feel free to change this line of code
