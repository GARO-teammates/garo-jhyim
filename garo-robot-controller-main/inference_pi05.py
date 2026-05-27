#!/usr/bin/env python3
"""
Pi0.5 Inference Script with Custom Weights
RRR Environment - pi0.5 custom model inference

Usage:
    python inference_pi05.py --model-path ./pi0.5_trained
"""

import torch
import numpy as np
from pathlib import Path
from PIL import Image

from lerobot.policies.pi05 import PI05Config, PI05Policy


def load_pi05_model(model_path: str, device: str = "cuda") -> PI05Policy:
    """
    Load pi0.5 model with custom weights.

    Args:
        model_path: Path to directory containing config.json and model.safetensors
        device: Device to load model on ('cuda' or 'cpu')

    Returns:
        Loaded PI05Policy model
    """
    model_path = Path(model_path)

    # Load using the from_pretrained method
    policy = PI05Policy.from_pretrained(model_path)
    policy.to(device)
    policy.eval()

    print(f"Model loaded from {model_path}")
    print(f"Device: {device}")
    print(f"Config type: {policy.config.type}")

    return policy


def preprocess_image(image: np.ndarray, target_size: tuple = (224, 224)) -> torch.Tensor:
    """
    Preprocess image for pi0.5 input.

    Args:
        image: Input image as numpy array (H, W, C) in BGR or RGB format
        target_size: Target size (height, width)

    Returns:
        Preprocessed image tensor (C, H, W)
    """
    if isinstance(image, np.ndarray):
        image = Image.fromarray(image)

    image = image.resize(target_size, Image.BILINEAR)
    image = np.array(image).astype(np.float32) / 255.0
    image = torch.from_numpy(image).permute(2, 0, 1)  # HWC -> CHW

    return image


def prepare_observation(
    images: dict,
    state: np.ndarray,
    device: str = "cuda"
) -> dict:
    """
    Prepare observation dictionary for pi0.5 inference.

    Args:
        images: Dictionary of camera images
                {'observation.images.top': np.ndarray,
                 'observation.images.wrist_left': np.ndarray,
                 'observation.images.wrist_right': np.ndarray}
        state: Robot state as numpy array (16-dim)
        device: Device for tensors

    Returns:
        Observation dictionary ready for model input
    """
    observation = {}

    # Process images
    for key, img in images.items():
        processed = preprocess_image(img)
        observation[key] = processed.unsqueeze(0).to(device)  # Add batch dim

    # Process state
    state_tensor = torch.from_numpy(state).float().unsqueeze(0).to(device)
    observation["observation.state"] = state_tensor

    return observation


def run_inference(
    policy: PI05Policy,
    images: dict,
    state: np.ndarray,
    device: str = "cuda"
) -> np.ndarray:
    """
    Run inference with pi0.5 model.

    Args:
        policy: Loaded PI05Policy model
        images: Dictionary of camera images
        state: Robot state as numpy array (16-dim)
        device: Device to run inference on

    Returns:
        Predicted action as numpy array
    """
    # Prepare observation
    observation = prepare_observation(images, state, device)

    # Run inference
    with torch.no_grad():
        action = policy.select_action(observation)

    return action.cpu().numpy()


class Pi05InferenceWrapper:
    """
    Wrapper class for easy pi0.5 inference.

    Example usage:
        wrapper = Pi05InferenceWrapper('./pi0.5_trained')

        # Get action from observations
        action = wrapper.get_action(
            top_image=top_img,
            wrist_left_image=wrist_left_img,
            wrist_right_image=wrist_right_img,
            state=robot_state
        )
    """

    def __init__(self, model_path: str, device: str = "cuda"):
        """
        Initialize inference wrapper.

        Args:
            model_path: Path to trained model directory
            device: Device to run inference on
        """
        self.device = device
        self.policy = load_pi05_model(model_path, device)
        self.policy.reset()

    def get_action(
        self,
        top_image: np.ndarray,
        wrist_left_image: np.ndarray,
        wrist_right_image: np.ndarray,
        state: np.ndarray
    ) -> np.ndarray:
        """
        Get action from observations.

        Args:
            top_image: Top camera image (H, W, 3)
            wrist_left_image: Left wrist camera image (H, W, 3)
            wrist_right_image: Right wrist camera image (H, W, 3)
            state: Robot state (16-dim)

        Returns:
            Action array (16-dim for single step, or chunk_size x 16 for action chunk)
        """
        images = {
            "observation.images.top": top_image,
            "observation.images.wrist_left": wrist_left_image,
            "observation.images.wrist_right": wrist_right_image,
        }

        return run_inference(self.policy, images, state, self.device)

    def reset(self):
        """Reset policy state (call when starting new episode)."""
        self.policy.reset()


def main():
    """Example usage of pi0.5 inference."""
    import argparse

    parser = argparse.ArgumentParser(description="Pi0.5 Inference")
    parser.add_argument(
        "--model-path",
        type=str,
        default="./pi0.5_trained",
        help="Path to trained model directory"
    )
    parser.add_argument(
        "--device",
        type=str,
        default="cuda",
        choices=["cuda", "cpu"],
        help="Device to run inference on"
    )
    args = parser.parse_args()

    # Check CUDA availability
    if args.device == "cuda" and not torch.cuda.is_available():
        print("CUDA not available, falling back to CPU")
        args.device = "cpu"

    # Load model
    print(f"\nLoading model from: {args.model_path}")
    wrapper = Pi05InferenceWrapper(args.model_path, args.device)

    # Example: Create dummy inputs for testing
    print("\n--- Testing with dummy inputs ---")

    dummy_top = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    dummy_wrist_left = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    dummy_wrist_right = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    dummy_state = np.zeros(16, dtype=np.float32)

    action = wrapper.get_action(
        top_image=dummy_top,
        wrist_left_image=dummy_wrist_left,
        wrist_right_image=dummy_wrist_right,
        state=dummy_state
    )

    print(f"Output action shape: {action.shape}")
    print(f"Action values: {action}")
    print("\nInference successful!")


if __name__ == "__main__":
    main()
