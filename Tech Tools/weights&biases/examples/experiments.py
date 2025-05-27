import tensorflow as tf
import numpy as np
import wandb
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# Sweep config
sweep_config = {
    'method': 'grid',
    'metric': {
        'name': 'val_loss',
        'goal': 'minimize'
    },
    'parameters': {
        'epochs': {'values': [50, 200, 400]},
        'batch_size': {'values': [64]},
        'learning_rate': {'values': [0.005, 0.0001]},
        'hidden_units': {'values': [64, 128]}
    }
}


def create_model(hidden_units):
    x = tf.keras.layers.Input(shape=(1,))
    y = tf.keras.layers.Dense(hidden_units, activation='relu')(x)
    y = tf.keras.layers.Dense(1, activation='tanh')(y)
    model = tf.keras.models.Model(inputs=x, outputs=y)
    return model

def data_generator(data_size: int = 64):
    x = np.random.uniform(-5, 5, size=(data_size, 1))
    y = np.sin(x) + np.random.normal(0, 0.05, size=(data_size, 1))
    return x, y

def train():
    wandb.init()
    config = wandb.config

    model = create_model(config.hidden_units)
    optimizer = tf.keras.optimizers.Adam(learning_rate=config.learning_rate)
    early_stop_patience = 10
    best_val_loss = float('inf')
    wait = 0

    # Generate training and validation data
    train_x, train_y = data_generator(data_size=512)
    val_x, val_y = data_generator(data_size=128)

    for epoch in range(config.epochs):
        with tf.GradientTape() as tape:
            pred_y = model(train_x, training=True)
            loss = tf.reduce_mean(tf.square(pred_y - train_y))
        grads = tape.gradient(loss, model.trainable_variables)
        # grads = [tf.clip_by_norm(g, 1.0) for g in grads]  # Gradient clipping
        optimizer.apply_gradients(zip(grads, model.trainable_variables))

        val_pred = model(val_x, training=False)
        val_loss = tf.reduce_mean(tf.square(val_pred - val_y))

        wandb.log({
            "epoch": epoch + 1,
            "train_loss": loss.numpy(),
            "val_loss": val_loss.numpy()
        })

        for layer in model.layers:
            weights = layer.get_weights()
            for i, w in enumerate(weights):
                wandb.log({f"{layer.name}_w{i}_hist": wandb.Histogram(w)})

        print(f"Epoch {epoch + 1}/{config.epochs}, Loss: {loss.numpy()}, Val Loss: {val_loss.numpy()}")

        if val_loss < best_val_loss:
            best_val_loss = val_loss
            wait = 0
            model.save("best_model.h5")
            wandb.save("best_model.h5")
        else:
            wait += 1
            if wait >= early_stop_patience:
                print("Early stopping")
                break

    # Log predictions
    val_pred = model(val_x, training=False)
    plt.figure()
    plt.scatter(val_x, val_y, label="Ground Truth")
    plt.scatter(val_x, val_pred.numpy(), label="Prediction", alpha=0.7)
    plt.legend()
    plt.title("Predictions vs Ground Truth")
    plt.xlabel("Input")
    plt.ylabel("Output")
    plt.grid(True)
    plt.savefig("predictions.png")
    wandb.log({"predictions_plot": wandb.Image("predictions.png")})

    # Save model artifact
    artifact = wandb.Artifact('best_model', type='model')
    artifact.add_file("best_model.h5")
    wandb.log_artifact(artifact)
    wandb.finish()

if __name__ == "__main__":
    sweep_id = wandb.sweep(sweep_config, project='tf-wandb-sweep-demo3')
    wandb.agent(sweep_id, function=train, count=6)
