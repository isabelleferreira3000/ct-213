import numpy as np
from utils import sigmoid, sigmoid_derivative
from math import log


class NeuralNetwork:
    """
    Represents a two-layers Neural Network (NN) for multi-class classification.
    The sigmoid activation function is used for all neurons.
    """
    def __init__(self, num_inputs, num_hiddens, num_outputs, alpha):
        """
        Constructs a two-layers Neural Network.

        :param num_inputs: number of inputs of the NN.
        :type num_inputs: int.
        :param num_hiddens: number of neurons in the hidden layer.
        :type num_hiddens: int.
        :param num_outputs: number of outputs of the NN.
        :type num_outputs: int.
        :param alpha: learning rate.
        :type alpha: float.
        """
        self.num_inputs = num_inputs
        self.num_hiddens = num_hiddens
        self.num_outputs = num_outputs
        self.alpha = alpha
        self.weights = [None] * 3
        self.biases = [None] * 3
        self.weights[1] = 0.001 * np.matrix(np.random.randn(num_hiddens, num_inputs))
        self.weights[2] = 0.001 * np.matrix(np.random.randn(num_outputs, num_hiddens))
        self.biases[1] = np.zeros((num_hiddens, 1))
        self.biases[2] = np.zeros((num_outputs, 1))

    def forward_propagation(self, input):
        """
        Executes forward propagation.
        Notice that the z and a of the first layer (l = 0) are equal to the NN's input.

        :param input: input to the network.
        :type input: (num_inputs, 1) numpy matrix.
        :return z: values computed by applying weights and biases at each layer of the NN.
        :rtype z: 3-dimensional list of (num_neurons[l], 1) numpy matrices.
        :return a: activations computed by applying the activation function to z at each layer.
        :rtype a: 3-dimensional list of (num_neurons[l], 1) numpy matrices.
        """
        z = [None] * 3
        a = [None] * 3
        z[0] = input
        a[0] = input

        # Add logic for neural network inference
        # z[1] = W[1] x + b[1]
        # a[1] = g[1](z[1])
        # z[2] = W[2] a[1] + b[2]
        # y = a[2] = g[2](z[2])

        z[1] = self.weights[1] * input + self.biases[1]
        a[1] = sigmoid(z[1])
        z[2] = self.weights[2] * a[1] + self.biases[2]
        a[2] = sigmoid(z[2])

        return z, a

    def compute_cost(self, inputs, expected_outputs):
        """
        Computes the logistic regression cost of this network.

        :param inputs: inputs to the network.
        :type inputs: list of numpy matrices.
        :param expected_outputs: expected outputs of the network.
        :type expected_outputs: list of numpy matrices.
        :return: logistic regression cost.
        :rtype: float.
        """
        cost = 0.0
        num_cases = len(inputs)
        outputs = [None] * num_cases
        for i in range(num_cases):
            z, a = self.forward_propagation(inputs[i])
            outputs[i] = a[-1]
        y = expected_outputs
        yhat = outputs
        for i in range(num_cases):
            for c in range(self.num_outputs):
                cost += -(y[i][c] * log(yhat[i][c]) + (1.0 - y[i][c]) * log(1.0 - yhat[i][c]))
        cost /= num_cases
        return cost

    def compute_gradient_back_propagation(self, inputs, expected_outputs):
        """
        Computes the gradient with respect to the NN's parameters using back propagation.

        :param inputs: inputs to the network.
        :type inputs: list of numpy matrices.
        :param expected_outputs: expected outputs of the network.
        :type expected_outputs: list of numpy matrices.
        :return weights_gradient: gradients of the weights at each layer.
        :rtype weights_gradient: L-dimensional list of numpy matrices.
        :return biases_gradient: gradients of the biases at each layer.
        :rtype biases_gradient: L-dimensional list of numpy matrices.
        """
        weights_gradient = [None] * 3
        biases_gradient = [None] * 3
        weights_gradient[1] = np.zeros((self.num_hiddens, self.num_inputs))
        weights_gradient[2] = np.zeros((self.num_outputs, self.num_hiddens))
        biases_gradient[1] = np.zeros((self.num_hiddens, 1))
        biases_gradient[2] = np.zeros((self.num_outputs, 1))

        # Add logic to compute the gradients
        for i in range(len(inputs)):
            z, a = self.forward_propagation(inputs[i])

            # dz[2] = a[2] - y
            # dW[2] = dz[2] a[1].T
            # db[2] = dz[2]
            # dz[1] = W[2].T dz[2] * g[1]'(z[1])
            # dW[1] = dz[1] x.T
            # db[1] = dz[1]

            dz2 = np.matrix(a[2] - expected_outputs[i])
            weights_gradient[2] += dz2 * a[1].T
            biases_gradient[2] += dz2

            dz1 = np.matrix(np.multiply(self.weights[2].T * dz2, sigmoid_derivative(z[1])))
            weights_gradient[1] += dz1 * inputs[i].T
            biases_gradient[1] += dz1

        return weights_gradient, biases_gradient

    def back_propagation(self, inputs, expected_outputs):
        """
        Executes the back propagation algorithm to update the NN's parameters.

        :param inputs: inputs to the network.
        :type inputs: list of numpy matrices.
        :param expected_outputs: expected outputs of the network.
        :type expected_outputs: list of numpy matrices.
        """
        # Add logic to update the weights and biases
        weights_gradient, biases_gradient = self.compute_gradient_back_propagation(inputs, expected_outputs)

        # W1 = W1 - learning_rate * dW1
        # b1 = b1 - learning_rate * db1
        # W2 = W2 - learning_rate * dW2
        # b2 = b2 - learning_rate * db2

        self.weights[1] -= self.alpha * weights_gradient[1]
        self.biases[1] -= self.alpha * biases_gradient[1]
        self.weights[2] -= self.alpha * weights_gradient[2]
        self.biases[2] -= self.alpha * biases_gradient[2]
