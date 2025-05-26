import tensorflow as tf
import numpy as np
import wandb

def create_model():
    x = tf.keras.layers.Input(shape=(1,))
    y = tf.keras.layers.Dense(128, activation='relu')(x)
    y = tf.keras.layers.Dense(1, activation='tanh')(y)
    model = tf.keras.models.Model(inputs=x, outputs=y)
    return model

def data_generator(data_size: int = 64):
    x = np.random.uniform(-5, 5, size=(data_size, 1))
    y = np.sin(x) + np.random.normal(0, 0.05, size=(data_size, 1))
    return x, y

def train_model(train_x, train_y, model, epochs: int = 100):
    optimizer = tf.keras.optimizers.Adam()
    for epoch in range(epochs):
        with tf.GradientTape() as tape:
            pred_y = model(train_x, training=True)
            loss = tf.reduce_mean(tf.square(pred_y - train_y))
        grads = tape.gradient(loss, model.trainable_variables)
        optimizer.apply_gradients(zip(grads, model.trainable_variables))
        wandb.log({"epoch": epoch + 1, "loss": loss.numpy()})
        print(f"Epoch {epoch + 1}/{epochs}, Loss: {loss.numpy()}")

def main():
    wandb.init(project="tf-wandb-demo", config={
        "epochs": 100,
        "batch_size": 64,
        "learning_rate": 0.001,
        "model": "simple_dense"
    })

    config = wandb.config

    model = create_model()

    train_x, train_y = data_generator(data_size=config.batch_size)

    train_model(train_x, train_y, model, epochs=config.epochs)

    model.save("model.h5") # Save the model
    wandb.save("model.h5") # Save the model to Weights & Biases

    wandb.finish()

if __name__ == "__main__":
    main()
