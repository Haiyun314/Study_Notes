# 🧪 Getting Started with Weights & Biases (WandB)

**Weights & Biases (WandB)** is a tool for tracking machine learning experiments, visualizing metrics, and collaborating with team members. It integrates with popular ML frameworks like PyTorch, TensorFlow, Keras, and more.

```bash
pip install wandb
wandb login (--relogin to force relogin) # then past the api key
```

## implement structure
```bash
sweep_config = {...}
sweep_id = wandb.sweep(sweep_config, project='tf-wandb-sweep-demo3')
wandb.agent(sweep_id, function=train, count=6)

def train():
    wandb.init()
    config = wandb.config

    data_artifact = wandb.Artifact('training_data', type='dataset')
    np.save("train_x.npy", train_x)
    np.save("train_y.npy", train_y)
    data_artifact.add_file("train_x.npy")
    data_artifact.add_file("train_y.npy")
    wandb.log_artifact(data_artifact)

    ...
    for epoch in epochs():
        wandb.log({
        "epoch": epoch + 1,
        "train_loss": loss.numpy(),
        "val_loss": val_loss.numpy()
        })
    wandb.log({"predictions_plot": wandb.Image("predictions.png")})

    model.save("best_model.h5")
    wandb.save("best_model.h5")
    artifact = wandb.Artifact('best_model', type='model')
    artifact.add_file("best_model.h5")
    wandb.log_artifact(artifact)
    
    wandb.finish()
```