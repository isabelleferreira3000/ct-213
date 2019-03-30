from math import exp
import random


def simulated_annealing(cost_function, random_neighbor, schedule, theta0, epsilon, max_iterations):
    """
    Executes the Simulated Annealing (SA) algorithm to minimize (optimize) a cost function.

    :param cost_function: function to be minimized.
    :type cost_function: function.
    :param random_neighbor: function which returns a random neighbor of a given point.
    :type random_neighbor: numpy.array.
    :param schedule: function which computes the temperature schedule.
    :type schedule: function.
    :param theta0: initial guess.
    :type theta0: numpy.array.
    :param epsilon: used to stop the optimization if the current cost is less than epsilon.
    :type epsilon: float.
    :param max_iterations: maximum number of iterations.
    :type max_iterations: int.
    :return theta: local minimum.
    :rtype theta: np.array.
    :return history: history of points visited by the algorithm.
    :rtype history: list of np.array.
    """
    theta = theta0
    history = [theta0]
    # Todo: Implement Simulated
    iteration = 0
    while not check_stopping_condition(cost_function(theta), epsilon, iteration, max_iterations):
        temperature = schedule(iteration)
        if temperature < 0.0:
            break
        neighbor = random_neighbor(theta)
        delta_e = cost_function(neighbor) - cost_function(theta)

        if delta_e < 0:
            theta = neighbor
        else:
            r = random.uniform(0.0, 1.0)  # Draws random number w/ uniform dist.
            if r <= exp(- delta_e / temperature):
                theta = neighbor

        history.append(theta)
        iteration = iteration + 1

    return theta, history


def check_stopping_condition(cost_function_theta, epsilon, iteration, max_iterations):
    if iteration > max_iterations:
        return True
    if cost_function_theta < epsilon:
        return True
    return False
