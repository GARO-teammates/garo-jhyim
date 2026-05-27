"""
GARO Snack Dataset -> LeRobot v2.0 변환 스크립트

원본 데이터 구조:
  2026_01_28/pick_and_place/episode_XXXX/
    ├── metadata.json
    ├── episode_data.json          (list of frames)
    ├── observation_images_top/    (frame_XXXXXX.jpg)
    ├── observation_images_wrist_left/
    └── observation_images_wrist_right/

사용법:
  cd /home/jhyim0823/garo_projects/openpi
  uv run scripts/convert_snack_to_lerobot.py
"""

import json
import shutil
from pathlib import Path

from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
import numpy as np
from PIL import Image
import torch
import tqdm


RAW_DIR = Path("/data/jhyim0823/2026_01_28")
REPO_ID = "garo/pi0_dataset_v43_new"
OUTPUT_ROOT = Path("/data/jhyim0823")
FPS = 11


def create_empty_dataset() -> LeRobotDataset:
    motors = [
        "R_Shoulder_FB", "R_Shoulder_UD", "R_Arm_Rot", "R_Elbow",
        "R_Wrist_Rot", "R_Wrist_FB", "R_Wrist_LR", "R_Gripper",
        "L_Shoulder_FB", "L_Shoulder_UD", "L_Arm_Rot", "L_Elbow",
        "L_Wrist_Rot", "L_Wrist_FB", "L_Wrist_LR", "L_Gripper",
    ]

    features = {
        "observation.state": {
            "dtype": "float32",
            "shape": (16,),
            "names": [motors],
        },
        "action": {
            "dtype": "float32",
            "shape": (16,),
            "names": [motors],
        },
        "observation.images.top": {
            "dtype": "image",
            "shape": (3, 480, 640),
            "names": ["channels", "height", "width"],
        },
        "observation.images.wrist_left": {
            "dtype": "image",
            "shape": (3, 480, 640),
            "names": ["channels", "height", "width"],
        },
        "observation.images.wrist_right": {
            "dtype": "image",
            "shape": (3, 480, 640),
            "names": ["channels", "height", "width"],
        },
    }

    dataset_path = OUTPUT_ROOT / REPO_ID
    if dataset_path.exists():
        shutil.rmtree(dataset_path)

    dataset = LeRobotDataset.create(
        repo_id=REPO_ID,
        fps=FPS,
        root=OUTPUT_ROOT / REPO_ID,
        robot_type="rx1_dual_arm",
        features=features,
        use_videos=False,
        image_writer_processes=4,
        image_writer_threads=4,
    )
    return dataset


# Camera folder name -> LeRobot key
CAM_MAP = {
    "observation_images_top": "observation.images.top",
    "observation_images_wrist_left": "observation.images.wrist_left",
    "observation_images_wrist_right": "observation.images.wrist_right",
}


def main():
    print(f"Raw dir: {RAW_DIR}")
    print(f"Output: {OUTPUT_ROOT / REPO_ID}")

    task_dirs = sorted([d for d in RAW_DIR.iterdir() if d.is_dir()])
    print(f"Task dirs: {[d.name for d in task_dirs]}")

    dataset = create_empty_dataset()

    total_episodes = 0
    total_frames = 0

    for task_dir in task_dirs:
        ep_dirs = sorted([d for d in task_dir.iterdir() if d.is_dir() and d.name.startswith("episode_")])
        print(f"\n[{task_dir.name}] {len(ep_dirs)} episodes")

        for ep_dir in tqdm.tqdm(ep_dirs, desc=task_dir.name):
            data_path = ep_dir / "episode_data.json"
            meta_path = ep_dir / "metadata.json"

            if not data_path.exists():
                print(f"  SKIP: {ep_dir.name} (no episode_data.json)")
                continue

            with open(meta_path) as f:
                meta = json.load(f)
            with open(data_path) as f:
                frames = json.load(f)

            task_str = meta.get("task", meta.get("language_instruction", task_dir.name))

            for frame in frames:
                fi = frame["frame_index"]

                frame_data = {
                    "observation.state": torch.tensor(frame["observation.state"], dtype=torch.float32),
                    "action": torch.tensor(frame["action"], dtype=torch.float32),
                    "task": task_str,
                }

                # Load images: frame_XXXXXX.jpg
                for folder_name, lerobot_key in CAM_MAP.items():
                    img_path = ep_dir / folder_name / f"frame_{fi:06d}.jpg"
                    if not img_path.exists():
                        img_path = ep_dir / folder_name / f"frame_{fi:06d}.png"
                    if img_path.exists():
                        frame_data[lerobot_key] = np.array(Image.open(img_path).convert("RGB"))
                    else:
                        print(f"  WARNING: missing {img_path}")
                        frame_data[lerobot_key] = np.zeros((480, 640, 3), dtype=np.uint8)

                dataset.add_frame(frame_data)
                total_frames += 1

            dataset.save_episode()
            total_episodes += 1

    print(f"\nConsolidating...")
    dataset.consolidate()

    print(f"\n=== Done ===")
    print(f"Episodes: {total_episodes}")
    print(f"Frames: {total_frames}")
    print(f"Saved to: {OUTPUT_ROOT / REPO_ID}")


if __name__ == "__main__":
    main()
