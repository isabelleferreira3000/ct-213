import numpy as np
from math import inf, fabs
from utils import *


def random_policy(grid_world):
    """
    Creates a random policy for a grid world.

    :param grid_world: the grid world.
    :type grid_world: GridWorld.
    :return: random policy.
    :rtype: tridimensional NumPy array.
    """
    dimensions = grid_world.dimensions
    policy = (1.0 / NUM_ACTIONS) * np.ones((dimensions[0], dimensions[1], NUM_ACTIONS))
    return policy


def greedy_policy(grid_world, value, epsilon=1.0e-3):
    """
    Computes a greedy policy considering a value function for a grid world. If there are more than
    one optimal action for a given state, then the optimal action is chosen at random.


    :param grid_world: the grid world.
    :type grid_world: GridWorld.
    :param value: the value function.
    :type value: bidimensional NumPy array.
    :param epsilon: tolerance used to consider that more than one action is optimal.
    :type epsilon: float.
    :return: greedy policy.
    :rtype: tridimensional NumPy array.
    """
    dimensions = grid_world.dimensions
    policy = np.zeros((dimensions[0], dimensions[1], NUM_ACTIONS))
    for i in range(dimensions[0]):
        for j in range(dimensions[1]):
            current_state = (i, j)
            if not grid_world.is_cell_valid(current_state):
                # Assuming random action if the cell is an obstacle
                policy[i, j] = (1.0 / NUM_ACTIONS) * np.ones(NUM_ACTIONS)
                continue
            max_value = -inf
            action_value = np.zeros(NUM_ACTIONS)  # Creating a temporary q(s, a)
            for action in range(NUM_ACTIONS):
                r = grid_world.reward(current_state, action)
                action_value[action] = r
                for next_state in grid_world.get_valid_sucessors((i, j), action):
                    transition_prob = grid_world.transition_probability(current_state, action, next_state)
                    action_value[action] += grid_world.gamma * transition_prob * value[next_state[0], next_state[1]]
                if action_value[action] > max_value:
                    max_value = action_value[action]
            # This post-processing is necessary since we may have more than one optimal action
            num_actions = 0
            for action in range(NUM_ACTIONS):
                if fabs(max_value - action_value[action]) < epsilon:
                    policy[i, j, action] = 1.0
                    num_actions += 1
            for action in range(NUM_ACTIONS):
                policy[i, j, action] /= num_actions
    return policy


def policy_evaluation(grid_world, initial_value, policy, num_iterations=10000, epsilon=1.0e-5):
    """
    Executes policy evaluation for a policy executed on a grid world.

    :param grid_world: the grid world.
    :type grid_world: GridWorld.
    :param initial_value: initial value function used to bootstrap the algorithm.
    :type initial_value: bidimensional NumPy array.
    :param policy: policy to be evaluated.
    :type policy: tridimensional NumPy array.
    :param num_iterations: maximum number of iterations used in policy evaluation.
    :type num_iterations: int.
    :param epsilon: tolerance used in stopping criterion.
    :type epsilon: float.
    :return: value function of the given policy.
    :rtype: bidimensional NumPy array.
    """
    dimensions = grid_world.dimensions
    value = np.copy(initial_value)
    # Todo: implement policy evaluation.
    last_value = np.copy(initial_value)

    for iteration in range(num_iterations):
        value = np.zeros(dimensions)

        for i in range(dimensions[0]):
            for j in range(dimensions[1]):
                current_state = (i, j)

                for action in range(NUM_ACTIONS):
                    pi = policy[i][j][action]
                    r = grid_world.reward(current_state, action)
                    value[i][j] += pi * r

                    for next_state in grid_world.get_valid_sucessors((i, j), action):
                        prob = grid_world.transition_probability(current_state, action, next_state)
                        value[i][j] += grid_world.gamma * pi * prob * last_value[next_state[0]][next_state[1]]

        if np.max(np.abs(value - last_value)) < epsilon:
            last_value = value
            break

        last_value = value

    return last_value


def value_iteration(grid_world, initial_value, num_iterations=10000, epsilon=1.0e-5):
    """
    Executes value iteration for a grid world.

    :param grid_world: the grid world.
    :type grid_world: GridWorld.
    :param initial_value: initial value function used to bootstrap the algorithm.
    :type initial_value: bidimensional NumPy array.
    :param num_iterations: maximum number of iterations used in policy evaluation.
    :type num_iterations: int.
    :param epsilon: tolerance used in stopping criterion.
    :type epsilon: float.
    :return value: optimal value function.
    :rtype value: bidimensional NumPy array.
    """
    dimensions = grid_world.dimensions
    value = np.copy(initial_value)
    # Todo: implement value iteration.
    max_value = np.copy(initial_value)
    last_value = np.copy(initial_value)

    for iteration in range(num_iterations):
        print("VALUE ITERATION ITERATION: " + str(iteration))
        value = np.zeros(dimensions)

        for i in range(dimensions[0]):
            for j in range(dimensions[1]):
                current_state = (i, j)

                for action in range(NUM_ACTIONS):
                    r = grid_world.reward(current_state, action)
                    value[i][j] += r

                    for next_state in grid_world.get_valid_sucessors((i, j), action):
                        prob = grid_world.transition_probability(current_state, action, next_state)
                        value[i][j] += grid_world.gamma * prob * last_value[next_state[0]][next_state[1]]

                    if value[i][j] > max_value[i][j]:
                        max_value[i][j] = value[i][j]

                value[i][j] = np.copy(max_value[i][j])

        print(value)

        if np.max(np.abs(value - last_value)) < epsilon:
            last_value = value
            break

        last_value = value

    return last_value


def policy_iteration(grid_world, initial_value, initial_policy, evaluations_per_policy=3, num_iterations=10000,
                     epsilon=1.0e-5):
    """
    Executes policy iteration for a grid world.

    :param grid_world: the grid world.
    :type grid_world: GridWorld.
    :param initial_value: initial value function used to bootstrap the algorithm.
    :type initial_value: bidimensional NumPy array.
    :param initial_policy: initial policy used to bootstrap the algorithm.
    :type initial_policy: tridimensional NumPy array.
    :param evaluations_per_policy: number of policy evaluations per policy iteration.
    :type evaluations_per_policy: int.
    :param num_iterations: maximum number of iterations used in policy evaluation.
    :type num_iterations: int.
    :param epsilon: tolerance used in stopping criterion.
    :type epsilon: float.
    :return value: value function of the optimal policy.
    :rtype value: bidimensional NumPy array.
    :return policy: optimal policy.
    :rtype policy: tridimensional NumPy array.
    """
    value = np.copy(initial_value)
    policy = np.copy(initial_policy)
    # Todo: implement policy iteration.
    last_policy = np.copy(initial_policy)
    last_value = np.copy(initial_value)

    for iteration in range(num_iterations):
        print("POLICY ITERATION ITERATION: " + str(iteration))

        value = policy_evaluation(grid_world, last_value, last_policy, evaluations_per_policy, epsilon)

        policy = greedy_policy(grid_world, value, epsilon)

        if np.max(np.fabs(policy - last_policy)) < epsilon:
            if np.max(np.fabs(value - last_value)) < epsilon:
                break

        last_value = value
        last_policy = policy

    return value, policy

