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
        quantity_of_dimensions = len(lower_bound)

        self.position = np.zeros(np.size(lower_bound))
        for i in range(quantity_of_dimensions):
            random_position = random.uniform(lower_bound[i], upper_bound[i])
            self.position[i] = random_position

        self.velocity = 0
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
        self.hyperparams = hyperparams
        self.lower_bound = lower_bound
        self.upper_bound = upper_bound
        self.quantity_of_dimensions = np.size(lower_bound)

        self.particles = []
        for i in range(self.hyperparams.num_particles):
            self.particles.append(Particle(self.lower_bound, self.upper_bound))

        self.global_best_value = -inf
        self.global_best_position = self.particles[0].position

        self.count_particles_evaluated = 0

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
        position_to_evaluate = self.particles[self.count_particles_evaluated].position
        # print("\nAnalisando particula " + str(self.count_particles_evaluated) + ": posicao " + str(position_to_evaluate))

        return position_to_evaluate

    def advance_generation(self):
        """
        Advances the generation of particles.
        """
        # Todo: implement
        for particle in self.particles:
            r_p = random.uniform(0, 1)
            r_g = random.uniform(0, 1)

            particle.velocity = self.hyperparams.inertia_weight * particle.velocity + \
                                self.hyperparams.cognitive_parameter * r_p * \
                                (particle.my_best_position - particle.position) - \
                                self.hyperparams.social_parameter * r_g * \
                                (self.global_best_position - particle.position)

            for i in range(self.quantity_of_dimensions):
                particle.velocity[i] = min(max(particle.velocity[i], -(self.upper_bound[i] - self.lower_bound[i])),
                                           self.upper_bound[i] - self.lower_bound[i])

            # print("velocidade: " + str(particle.velocity))

            # particle.velocity = min(max(particle.velocity, -(self.upper_bound - self.lower_bound)),
            #                         self.upper_bound - self.lower_bound)

            particle.position = particle.position + particle.velocity

            for i in range(self.quantity_of_dimensions):
                particle.position[i] = min(max(particle.position[i], -(self.upper_bound[i] - self.lower_bound[i])),
                                           self.upper_bound[i] - self.lower_bound[i])
            # particle.position = min(max(particle.position, self.lower_bound), self.upper_bound)

    def notify_evaluation(self, value):
        """
        Notifies the algorithm that a particle position evaluation was completed.

        :param value: quality of the particle position.
        :type value: float.
        """
        # Todo: implement
        # print(self.global_best_position)
        # print("valor para essa posicao: " + str(value))
        # print("------------------------")
        current_particle_evaluated = self.particles[self.count_particles_evaluated]
        # print("valor antigo: " + str(current_particle_evaluated.my_best_value))
        # print("valor melhor de todos: " + str(self.global_best_value))

        if value > current_particle_evaluated.my_best_value:
            # print("valor melhor que o antigo")
            current_particle_evaluated.my_best_value = value
            current_particle_evaluated.my_best_position = current_particle_evaluated.position

        if value > self.global_best_value:
            # print("valor melhor que o melhor de todos")
            self.global_best_value = value
            self.global_best_position = current_particle_evaluated.position

        self.count_particles_evaluated = self.count_particles_evaluated + 1
        if self.count_particles_evaluated == self.hyperparams.num_particles:
            self.count_particles_evaluated = 0
            self.advance_generation()
            # print("\nNOVA GERACAO")
