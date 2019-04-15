import numpy as np
import random
from math import inf


class Particle:
    """
    Represents a particle of the Particle Swarm Optimization algorithm.
    """
    def __init__(self, lower_bound, upper_bound):
        """
        Creates a particle of the Particle Swarm Optimization algorithm.

        :param lower_bound: lower bound of the particle position.
        :type lower_bound: numpy array.
        :param upper_bound: upper bound of the particle position.
        :type upper_bound: numpy array.
        """
        # Todo: implement
        self.position = np.zeros(np.size(lower_bound))
        self.velocity = np.zeros(np.size(lower_bound))
        for i in range(np.size(lower_bound)):
            delta_bound = upper_bound[i] - lower_bound[i]
            random_position = random.uniform(lower_bound[i], upper_bound[i])
            random_velocity = random.uniform(-delta_bound, delta_bound)
            self.position[i] = random_position
            self.velocity[i] = random_velocity

        self.my_best_value = -inf
        self.my_best_position = self.position


class ParticleSwarmOptimization:
    """
    Represents the Particle Swarm Optimization algorithm.
    Hyperparameters:
        inertia_weight: inertia weight.
        cognitive_parameter: cognitive parameter.
        social_parameter: social parameter.

    :param hyperparams: hyperparameters used by Particle Swarm Optimization.
    :type hyperparams: Params.
    :param lower_bound: lower bound of particle position.
    :type lower_bound: numpy array.
    :param upper_bound: upper bound of particle position.
    :type upper_bound: numpy array.
    """
    def __init__(self, hyperparams, lower_bound, upper_bound):
        # Todo: implement
        self.num_particles = hyperparams.num_particles
        self.inertia_weight = hyperparams.inertia_weight
        self.cognitive_parameter = hyperparams.cognitive_parameter
        self.social_parameter = hyperparams.social_parameter

        self.lower_bound = lower_bound
        self.upper_bound = upper_bound
        self.quantity_of_dimensions = np.size(lower_bound)

        self.particles = []
        for i in range(self.num_particles):
            particle = Particle(self.lower_bound, self.upper_bound)
            self.particles.append(particle)

        self.global_best_value = -inf
        self.global_best_position = self.particles[0].position

        self.count_particle_to_evaluate = 0

    def get_best_position(self):
        """
        Obtains the best position so far found by the algorithm.

        :return: the best position.
        :rtype: numpy array.
        """
        # Todo: implement
        return self.global_best_position

    def get_best_value(self):
        """
        Obtains the value of the best position so far found by the algorithm.

        :return: value of the best position.
        :rtype: float.
        """
        # Todo: implement
        return self.global_best_value

    def get_position_to_evaluate(self):
        """
        Obtains a new position to evaluate.

        :return: position to evaluate.
        :rtype: numpy array.
        """
        # Todo: implement
        position_to_evaluate = self.particles[self.count_particle_to_evaluate].position

        return position_to_evaluate

    def advance_generation(self):
        """
        Advances the generation of particles.
        """
        # Todo: implement
        print("\nAdvance generation")
        i = 0
        for particle in self.particles:
            print("\nparticula " + str(i))
            r_p = random.uniform(0, 1)
            r_g = random.uniform(0, 1)

            print("velocity antiga: " + str(particle.velocity))
            particle.velocity = self.inertia_weight * particle.velocity + \
                                self.cognitive_parameter * r_p * (particle.my_best_position - particle.position) - \
                                self.social_parameter * r_g * (self.global_best_position - particle.position)
            print("primeiro calculo de velocity: " + str(particle.velocity))

            for i in range(self.quantity_of_dimensions):
                particle.velocity[i] = min(max(particle.velocity[i], -(self.upper_bound[i] - self.lower_bound[i])),
                                           self.upper_bound[i] - self.lower_bound[i])
            print("velocity corrigido: " + str(particle.velocity))

            print("position antiga: " + str(particle.position))
            particle.position = particle.position + particle.velocity
            print("primeiro calculo de position: " + str(particle.position))

            for i in range(self.quantity_of_dimensions):
                particle.position[i] = min(max(particle.position[i], self.lower_bound[i]), self.upper_bound[i])
            print("position corrigido: " + str(particle.position))

    def notify_evaluation(self, value):
        """
        Notifies the algorithm that a particle position evaluation was completed.

        :param value: quality of the particle position.
        :type value: float.
        """
        # Todo: implement
        current_particle_evaluated = self.particles[self.count_particle_to_evaluate]
        print("\nParticula " + str(self.count_particle_to_evaluate))
        print("current position: " + str(current_particle_evaluated.position))
        print("current velocity: " + str(current_particle_evaluated.velocity))
        print("current value: " + str(value))
        print("global best value: " + str(self.global_best_value))
        print("my best value: " + str(current_particle_evaluated.my_best_value))
        print("my best position: " + str(current_particle_evaluated.my_best_position))

        if value > current_particle_evaluated.my_best_value:
            print("* melhor que my best value")
            current_particle_evaluated.my_best_value = float(value)
            current_particle_evaluated.my_best_position = np.array(current_particle_evaluated.position)

        if value > self.global_best_value:
            print("** melhor que global best value")
            self.global_best_value = float(value)
            self.global_best_position = np.array(current_particle_evaluated.position)

        self.count_particle_to_evaluate = self.count_particle_to_evaluate + 1
        if self.count_particle_to_evaluate == self.num_particles:
            self.count_particle_to_evaluate = 0
            self.advance_generation()
