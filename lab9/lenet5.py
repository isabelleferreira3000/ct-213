from keras import layers, activations
from keras.models import Sequential


def make_lenet5():
    model = Sequential()

    # Todo: implement LeNet-5 model
    # 1A CAMADA:
    model.add(layers.Conv2D(filters=6,
                            kernel_size=(5, 5),
                            strides=(1, 1),
                            activation=activations.tanh,
                            input_shape=(32, 32, 1)))

    # 2A CAMADA:
    model.add(layers.AveragePooling2D(pool_size=(6, 6),
                                      strides=(2, 2)))

    # 3A CAMADA:
    model.add(layers.Conv2D(filters=16,
                            kernel_size=(5, 5),
                            strides=(1, 1),
                            activation=activations.tanh))

    # 4A CAMADA:
    model.add(layers.AveragePooling2D(pool_size=(16, 16),
                                      strides=(2, 2)))

    # 5A CAMADA:
    model.add(layers.Conv2D(filters=120,
                            kernel_size=(5, 5),
                            strides=(1, 1),
                            activation=activations.tanh))

    # TRANSICAO:
    model.add(layers.Flatten())

    # 6A CAMADA:
    model.add(layers.Dense(84, activation=activations.tanh))

    # 7A CAMADA:
    model.add(layers.Dense(10, activation=activations.softmax))

    return model
