# Colab Training Guide - TECHIN517 Lab1

## Cell 1 - Mount Drive
from google.colab import drive
drive.mount('/content/drive')

## Cell 2 - Extract Dataset
import os
DATASET_DIR = '/content/lerobot_data'
os.makedirs(DATASET_DIR, exist_ok=True)
!tar -xzf "/content/drive/MyDrive/techin517_dataset.tar.gz" -C "{DATASET_DIR}"

## Cell 3 - Install LeRobot
!pip install -q "lerobot[all]==0.4.0"

## Cell 4 - Verify
import torch, json
print('GPU:', torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'No GPU!')
with open(f'{DATASET_DIR}/gix/test_task/meta/info.json') as f:
    info = json.load(f)
print(f"Dataset: {info['total_episodes']} episodes, {info['total_frames']} frames")

## Cell 5 - Train (1-2 hours)
import os
DATASET_DIR = '/content/lerobot_data'
os.environ['HF_HOME'] = DATASET_DIR
OUTPUT_DIR = '/content/drive/MyDrive/act_training_output'
os.makedirs(OUTPUT_DIR, exist_ok=True)
!lerobot-train --dataset.repo_id=gix/test_task --dataset.root="{DATASET_DIR}/gix/test_task" --policy.type=act --output_dir="{OUTPUT_DIR}" --job_name=act_test_task --device=cuda --wandb.enable=false

## Result
Model saved to Google Drive: act_training_output/checkpoints/last/pretrained_model/
