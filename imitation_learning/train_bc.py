"""
Behavioral Cloning (BC) Training Pipeline
=========================================
CS532 Autonomous JetBot — Module 07-imitation-learning
Based on UIT ITCourses RL-101 (Bài 10: Học bắt chước - Imitation Learning)

Mathematical Formulation:
  min_theta E_{(s, a) ~ D_E} [ w_v * (v - hat{v}_theta(s))^2 + w_w * (w - hat{w}_theta(s))^2 ] + lambda * ||theta||_2^2

Features:
  - Weighted Action MSE Loss (w_v=1.0, w_w=1.5)
  - AdamW Optimizer + Cosine Annealing Learning Rate
  - Validation Split & Early Stopping Protection
  - Automatic Export to PyTorch (.pt) and Pure NumPy JSON (.json)
"""

import os
import sys
import time
import argparse
from typing import Tuple, Dict, Any, Optional
import numpy as np

import torch
import torch.nn as nn
from torch.utils.data import DataLoader, random_split

from config import (
    BATCH_SIZE,
    LEARNING_RATE,
    WEIGHT_DECAY,
    NUM_EPOCHS,
    COSINE_LR_MIN,
    VAL_SPLIT_RATIO,
    EARLY_STOP_PATIENCE,
    W_LINEAR_LOSS,
    W_ANGULAR_LOSS,
    POLICY_BACKBONE
)
from dataset import ImitationLearningDataset
from model import get_policy_network


def train_behavioral_cloning(
    data_path: str = "data/expert_demonstrations.npz",
    model_save_dir: str = "models",
    epochs: int = NUM_EPOCHS,
    batch_size: int = BATCH_SIZE,
    lr: float = LEARNING_RATE,
    backbone: str = POLICY_BACKBONE
) -> Tuple[nn.Module, Dict[str, Any]]:
    """
    Trains the policy network using Behavioral Cloning.
    """
    if not os.path.exists(data_path):
        raise FileNotFoundError(f"Demonstrations not found at: {data_path}. Run data_collector.py first!")

    os.makedirs(model_save_dir, exist_ok=True)
    best_pt_path = os.path.join(model_save_dir, "bc_policy_best.pt")
    onnx_path = os.path.join(model_save_dir, "bc_policy.onnx")
    numpy_json_path = os.path.join(model_save_dir, "bc_policy_numpy.json")

    # 1. Load Dataset
    print(f"\n[INFO] Loading demonstration dataset: {data_path}")
    raw_dataset = ImitationLearningDataset.load_npz(data_path, augment_symmetry=True)
    print(f"[INFO] Total transitions in dataset: {len(raw_dataset)}")

    # Train / Val split
    val_size = max(10, int(len(raw_dataset) * VAL_SPLIT_RATIO))
    train_size = len(raw_dataset) - val_size
    train_set, val_set = random_split(
        raw_dataset,
        [train_size, val_size],
        generator=torch.Generator().manual_seed(42)
    )

    train_loader = DataLoader(train_set, batch_size=batch_size, shuffle=True, drop_last=False)
    val_loader = DataLoader(val_set, batch_size=batch_size, shuffle=False)

    print(f"[INFO] Train samples: {train_size} | Validation samples: {val_size}")

    # 2. Model & Optimizer
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"[INFO] Training backbone: {backbone.upper()} on compute device: {device}")

    model = get_policy_network(backbone=backbone).to(device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=lr, weight_decay=WEIGHT_DECAY)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=epochs, eta_min=COSINE_LR_MIN)

    # 3. Training Loop
    best_val_loss = float("inf")
    patience_counter = 0
    history = {"train_loss": [], "val_loss": [], "lr": []}

    print("\n" + "=" * 70)
    print(f"{'Epoch':^8} | {'Train MSE':^14} | {'Val MSE':^14} | {'LR':^12} | {'Status'}")
    print("=" * 70)

    for ep in range(1, epochs + 1):
        model.train()
        train_loss_accum = 0.0
        train_batches = 0

        for batch_obs, batch_act in train_loader:
            batch_obs = batch_obs.to(device)
            batch_act = batch_act.to(device)

            pred_act = model(batch_obs)

            # Weighted MSE Loss: w_v * (v - v*)^2 + w_w * (w - w*)^2
            loss_v = nn.functional.mse_loss(pred_act[:, 0], batch_act[:, 0])
            loss_w = nn.functional.mse_loss(pred_act[:, 1], batch_act[:, 1])
            loss = W_LINEAR_LOSS * loss_v + W_ANGULAR_LOSS * loss_w

            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

            train_loss_accum += loss.item()
            train_batches += 1

        scheduler.step()
        avg_train_loss = train_loss_accum / max(1, train_batches)

        # Validation
        model.eval()
        val_loss_accum = 0.0
        val_batches = 0
        with torch.no_grad():
            for val_obs, val_act in val_loader:
                val_obs = val_obs.to(device)
                val_act = val_act.to(device)
                pred_v = model(val_obs)
                lv = nn.functional.mse_loss(pred_v[:, 0], val_act[:, 0])
                lw = nn.functional.mse_loss(pred_v[:, 1], val_act[:, 1])
                val_loss_accum += (W_LINEAR_LOSS * lv + W_ANGULAR_LOSS * lw).item()
                val_batches += 1

        avg_val_loss = val_loss_accum / max(1, val_batches)
        curr_lr = optimizer.param_groups[0]["lr"]

        history["train_loss"].append(avg_train_loss)
        history["val_loss"].append(avg_val_loss)
        history["lr"].append(curr_lr)

        # Checkpointing
        status_str = ""
        if avg_val_loss < best_val_loss:
            best_val_loss = avg_val_loss
            patience_counter = 0
            torch.save(model.state_dict(), best_pt_path)
            model.save_numpy_weights(numpy_json_path)
            try:
                model.export_onnx(onnx_path)
            except Exception as e:
                print(f"[WARN] ONNX export failed: {e}")
            status_str = "[BEST SAVED]"
        else:
            patience_counter += 1

        if ep % 5 == 0 or status_str or ep == epochs:
            print(f"{ep:^8d} | {avg_train_loss:^14.6f} | {avg_val_loss:^14.6f} | {curr_lr:^12.2e} | {status_str}")

        if patience_counter >= EARLY_STOP_PATIENCE:
            print(f"\n[INFO] Early stopping triggered after {ep} epochs (Patience={EARLY_STOP_PATIENCE})")
            break

    print("=" * 70)
    print(f"[SUCCESS] Behavioral Cloning training complete!")
    print(f"          Best Validation Loss: {best_val_loss:.6f}")
    print(f"          Saved PyTorch model : {best_pt_path}")
    print(f"          Saved ONNX model    : {onnx_path}")
    print(f"          Saved NumPy JSON    : {numpy_json_path}\n")

    # Load best weights before returning
    model.load_state_dict(torch.load(best_pt_path, map_location=device))
    return model, history


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Train Behavioral Cloning Policy")
    parser.add_argument("--data", type=str, default="data/expert_demonstrations.npz", help="Path to expert demonstrations")
    parser.add_argument("--save-dir", type=str, default="models", help="Directory to save models")
    parser.add_argument("--epochs", type=int, default=NUM_EPOCHS, help="Number of training epochs")
    parser.add_argument("--batch-size", type=int, default=BATCH_SIZE, help="Batch size")
    parser.add_argument("--backbone", type=str, default=POLICY_BACKBONE, choices=["spiral", "mlp"], help="Backbone architecture")
    args = parser.parse_args()

    train_behavioral_cloning(
        data_path=args.data,
        model_save_dir=args.save_dir,
        epochs=args.epochs,
        batch_size=args.batch_size,
        backbone=args.backbone
    )
