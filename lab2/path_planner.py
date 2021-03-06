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

        pq = []
        start_node.f = 0
        heapq.heappush(pq, (start_node.f, start_node))

        while pq:
            f, current_node = heapq.heappop(pq)
            current_node.closed = True
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
                    # print("custo: "+str(successor_node.f))
                    successor_node.parent = current_node
                    heapq.heappush(pq, (successor_node.f, successor_node))

        # The first return is the path as sequence of tuples (as returned by the method construct_path())
        # The second return is the cost of the path
        path = self.construct_path(goal_node)
        total_cost = goal_node.f
        self.node_grid.reset()
        return path, total_cost  # Feel free to change this line of code

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
        start_node = self.node_grid.get_node(start_position[0], start_position[1])
        goal_node = self.node_grid.get_node(goal_position[0], goal_position[1])

        pq = []
        start_node.g = start_node.distance_to(goal_node.get_position()[0], goal_node.get_position()[1])
        start_node.f = start_node.g
        heapq.heappush(pq, (start_node.g, start_node))

        finished = False
        while pq and not finished:
            g, current_node = heapq.heappop(pq)
            current_node.closed = True
            current_node_i, current_node_j = current_node.get_position()
            for successor in self.node_grid.get_successors(current_node_i, current_node_j):
                successor_node = self.node_grid.get_node(successor[0], successor[1])

                if not successor_node.closed:
                    successor_node.closed = True
                    successor_node.parent = current_node

                    if current_node == goal_node:
                        finished = True
                        break
                    successor_node.g = successor_node.distance_to(goal_node.get_position()[0],
                                                                  goal_node.get_position()[1])
                    successor_node.f = current_node.f + self.cost_map.get_edge_cost(current_node.get_position(),
                                                                                    successor_node.get_position())
                    heapq.heappush(pq, (successor_node.g, successor_node))

        # The first return is the path as sequence of tuples (as returned by the method construct_path())
        # The second return is the cost of the path
        path = self.construct_path(goal_node)
        total_cost = goal_node.f
        self.node_grid.reset()
        return path, total_cost  # Feel free to change this line of code

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

        pq = []
        start_node.g = 0
        start_node.f = start_node.distance_to(goal_node.get_position()[0], goal_node.get_position()[1])
        heapq.heappush(pq, (start_node.f, start_node))

        while pq:
            f, current_node = heapq.heappop(pq)
            current_node.closed = True
            if current_node == goal_node:
                break

            current_node_i, current_node_j = current_node.get_position()
            for successor in self.node_grid.get_successors(current_node_i, current_node_j):
                successor_node = self.node_grid.get_node(successor[0], successor[1])

                if not successor_node.closed and \
                        successor_node.f > current_node.g +\
                        self.cost_map.get_edge_cost(current_node.get_position(), successor_node.get_position()) +\
                        successor_node.distance_to(goal_node.get_position()[0], goal_node.get_position()[1]):
                    successor_node.g = current_node.g + self.cost_map.get_edge_cost(current_node.get_position(),
                                                                                    successor_node.get_position())
                    successor_node.f = successor_node.g + successor_node.distance_to(goal_node.get_position()[0],
                                                                                     goal_node.get_position()[1])
                    successor_node.parent = current_node
                    heapq.heappush(pq, (successor_node.f, successor_node))

        # The first return is the path as sequence of tuples (as returned by the method construct_path())
        # The second return is the cost of the path
        path = self.construct_path(goal_node)
        total_cost = goal_node.f
        self.node_grid.reset()
        return path, total_cost  # Feel free to change this line of code
