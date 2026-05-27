#!/usr/bin/env python3
"""
Episode Labeling GUI
Shows first and last frame of each episode and allows task type selection.
"""

import os
import tkinter as tk
from tkinter import ttk, messagebox
from PIL import Image, ImageTk
import json
import shutil
from pathlib import Path
import sys

class EpisodeLabeler:
    def __init__(self, root, data_path=None):
        self.root = root
        self.root.title("Episode Labeler")
        self.root.geometry("1400x850")

        # Data path (can be passed as argument)
        if data_path:
            self.base_path = Path(data_path)
        else:
            _gui_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            self.base_path = Path(os.path.join(_gui_root, "datasets", "rx1_teleop_v1", "2026_01_28", "pick_and_place"))

        self.camera_folder = "observation_images_top"

        # Episode list and labels
        self.episodes = []
        self.labels = {}
        self.current_index = 0

        # Label options
        self.label_options = [
            ("Not Selected", "none"),
            ("Put snack in box", "Put the snack in the box."),
            ("Take snack from box", "Take the snack out of the box."),
        ]

        self.setup_ui()
        self.load_episodes()
        self.load_saved_labels()
        if self.episodes:
            self.show_episode(0)

    def setup_ui(self):
        # Main frame
        main_frame = tk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Top: Image display area
        image_frame = tk.Frame(main_frame)
        image_frame.pack(fill=tk.BOTH, expand=True)

        # First frame (left)
        left_frame = tk.LabelFrame(image_frame, text="First Frame", font=('Arial', 12, 'bold'))
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)

        self.first_image_label = tk.Label(left_frame, bg='gray')
        self.first_image_label.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Last frame (right)
        right_frame = tk.LabelFrame(image_frame, text="Last Frame", font=('Arial', 12, 'bold'))
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5)

        self.last_image_label = tk.Label(right_frame, bg='gray')
        self.last_image_label.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Middle: Episode info and label selection
        info_frame = tk.Frame(main_frame)
        info_frame.pack(fill=tk.X, pady=10)

        # Episode name
        self.episode_label = tk.Label(info_frame, text="", font=('Arial', 16, 'bold'))
        self.episode_label.pack()

        # Frame info
        self.frame_info_label = tk.Label(info_frame, text="", font=('Arial', 11))
        self.frame_info_label.pack()

        # Label selection buttons
        label_frame = tk.Frame(info_frame)
        label_frame.pack(pady=10)

        tk.Label(label_frame, text="Task Type:", font=('Arial', 12)).pack(side=tk.LEFT, padx=5)

        self.label_var = tk.StringVar(value="none")
        for text, value in self.label_options:
            rb = tk.Radiobutton(label_frame, text=text, variable=self.label_var,
                               value=value, font=('Arial', 11),
                               command=self.on_label_change)
            rb.pack(side=tk.LEFT, padx=10)

        # Navigation buttons
        nav_frame = tk.Frame(main_frame)
        nav_frame.pack(fill=tk.X, pady=5)

        self.prev_btn = tk.Button(nav_frame, text="◀ Prev (A)", font=('Arial', 12),
                                  command=self.prev_episode, width=15)
        self.prev_btn.pack(side=tk.LEFT, padx=20)

        # Progress
        self.progress_label = tk.Label(nav_frame, text="", font=('Arial', 12))
        self.progress_label.pack(side=tk.LEFT, expand=True)

        self.next_btn = tk.Button(nav_frame, text="Next (D) ▶", font=('Arial', 12),
                                  command=self.next_episode, width=15)
        self.next_btn.pack(side=tk.RIGHT, padx=20)

        # Bottom: Save button and stats
        bottom_frame = tk.Frame(main_frame)
        bottom_frame.pack(fill=tk.X, pady=5)

        self.save_btn = tk.Button(bottom_frame, text="Save Labels (Ctrl+S)", font=('Arial', 12, 'bold'),
                                  command=self.save_labels, bg='#28a745', fg='white', width=18)
        self.save_btn.pack(side=tk.LEFT, padx=20)

        self.stats_label = tk.Label(bottom_frame, text="", font=('Arial', 11))
        self.stats_label.pack(side=tk.LEFT, expand=True)

        # Quick labeling buttons
        quick_frame = tk.Frame(bottom_frame)
        quick_frame.pack(side=tk.RIGHT, padx=20)

        tk.Button(quick_frame, text="Take Out (W)", font=('Arial', 11), bg='#dc3545', fg='white',
                  command=lambda: self.quick_label("Take the snack out of the box."), width=14).pack(side=tk.LEFT, padx=5)
        tk.Button(quick_frame, text="Put In (S)", font=('Arial', 11), bg='#007bff', fg='white',
                  command=lambda: self.quick_label("Put the snack in the box."), width=14).pack(side=tk.LEFT, padx=5)
        tk.Button(quick_frame, text="Delete (-)", font=('Arial', 11), bg='#6c757d', fg='white',
                  command=self.delete_episode, width=14).pack(side=tk.LEFT, padx=5)

        # Keyboard shortcuts help
        shortcut_frame = tk.LabelFrame(main_frame, text="Keyboard Shortcuts", font=('Arial', 10))
        shortcut_frame.pack(fill=tk.X, pady=5)

        shortcuts_text = "  W: Take Out + Next  |  S: Put In + Next  |  A/←: Previous  |  D/→: Next  |  -: Delete Episode  |  Ctrl+S: Save  "
        tk.Label(shortcut_frame, text=shortcuts_text, font=('Arial', 10), fg='#555').pack(pady=3)

        # Keyboard bindings
        self.root.bind('<Left>', lambda e: self.prev_episode())
        self.root.bind('<Right>', lambda e: self.next_episode())
        self.root.bind('a', lambda e: self.prev_episode())
        self.root.bind('d', lambda e: self.next_episode())
        self.root.bind('w', lambda e: self.quick_label("Take the snack out of the box."))
        self.root.bind('s', lambda e: self.quick_label("Put the snack in the box."))
        self.root.bind('-', lambda e: self.delete_episode())
        self.root.bind('<Control-s>', lambda e: self.save_labels())

    def load_episodes(self):
        """Load episode list"""
        if not self.base_path.exists():
            messagebox.showerror("Error", f"Path not found:\n{self.base_path}")
            return

        self.episodes = sorted([d.name for d in self.base_path.iterdir()
                               if d.is_dir() and d.name.startswith('episode_')])
        print(f"Loaded {len(self.episodes)} episodes from {self.base_path}")

    def load_saved_labels(self):
        """Load saved labels from individual episode files"""
        for ep_name in self.episodes:
            ep_dir = self.base_path / ep_name
            metadata_file = ep_dir / "metadata.json"
            if metadata_file.exists():
                with open(metadata_file, 'r') as f:
                    metadata = json.load(f)
                if 'task_type' in metadata:
                    self.labels[ep_name] = metadata['task_type']
        print(f"Loaded {len(self.labels)} saved labels")

    def save_labels(self):
        """Save labels to episode_data.json and metadata.json"""
        saved_count = 0

        for ep_name, label in self.labels.items():
            ep_dir = self.base_path / ep_name

            # Update episode_data.json
            ep_data_file = ep_dir / "episode_data.json"
            if ep_data_file.exists():
                with open(ep_data_file, 'r') as f:
                    ep_data = json.load(f)

                # Add task_type to each frame
                for frame in ep_data:
                    frame['task_type'] = label

                with open(ep_data_file, 'w') as f:
                    json.dump(ep_data, f)

            # Update metadata.json
            metadata_file = ep_dir / "metadata.json"
            if metadata_file.exists():
                with open(metadata_file, 'r') as f:
                    metadata = json.load(f)
            else:
                metadata = {}

            metadata['task_type'] = label

            with open(metadata_file, 'w') as f:
                json.dump(metadata, f, indent=2)

            saved_count += 1

        messagebox.showinfo("Save Complete", f"Labels saved to {saved_count} episodes.\n(episode_data.json & metadata.json)")
        self.update_stats()

    def show_episode(self, index):
        """Show episode"""
        if not self.episodes:
            return

        self.current_index = max(0, min(index, len(self.episodes) - 1))
        episode_name = self.episodes[self.current_index]
        episode_dir = self.base_path / episode_name
        cam_dir = episode_dir / self.camera_folder

        # Show episode info
        self.episode_label.config(text=episode_name)

        # Frame info
        ep_file = episode_dir / "episode_data.json"
        if ep_file.exists():
            with open(ep_file, 'r') as f:
                frames = json.load(f)
            duration = frames[-1].get('timestamp', 0) if frames else 0
            self.frame_info_label.config(text=f"Frames: {len(frames)} | Duration: {duration:.1f}s")

        # Load images
        if cam_dir.exists():
            frame_files = sorted(cam_dir.glob('frame_*.jpg'))
            if frame_files:
                # First frame
                first_img = self.load_and_resize_image(frame_files[0], 550, 380)
                if first_img:
                    self.first_image_label.config(image=first_img)
                    self.first_image_label.image = first_img

                # Last frame
                last_img = self.load_and_resize_image(frame_files[-1], 550, 380)
                if last_img:
                    self.last_image_label.config(image=last_img)
                    self.last_image_label.image = last_img

        # Show current label
        current_label = self.labels.get(episode_name, "none")
        self.label_var.set(current_label)

        # Update progress
        self.progress_label.config(text=f"{self.current_index + 1} / {len(self.episodes)}")
        self.update_stats()

    def load_and_resize_image(self, path, max_width, max_height):
        """Load and resize image"""
        try:
            img = Image.open(path)

            # Resize maintaining aspect ratio
            ratio = min(max_width / img.width, max_height / img.height)
            new_size = (int(img.width * ratio), int(img.height * ratio))
            img = img.resize(new_size, Image.LANCZOS)

            return ImageTk.PhotoImage(img)
        except Exception as e:
            print(f"Image load failed: {e}")
            return None

    def on_label_change(self):
        """On label change"""
        episode_name = self.episodes[self.current_index]
        label = self.label_var.get()

        if label == "none":
            self.labels.pop(episode_name, None)
        else:
            self.labels[episode_name] = label

        self.update_stats()

    def quick_label(self, label):
        """Quick labeling then next"""
        if not self.episodes:
            return
        episode_name = self.episodes[self.current_index]
        self.labels[episode_name] = label
        self.label_var.set(label)
        self.update_stats()
        self.next_episode()

    def prev_episode(self):
        """Previous episode"""
        if self.current_index > 0:
            self.show_episode(self.current_index - 1)

    def next_episode(self):
        """Next episode"""
        if self.current_index < len(self.episodes) - 1:
            self.show_episode(self.current_index + 1)

    def delete_episode(self):
        """Delete current episode folder"""
        if not self.episodes:
            return

        episode_name = self.episodes[self.current_index]
        episode_dir = self.base_path / episode_name

        # Confirm deletion
        if not messagebox.askyesno("Delete Episode",
                                   f"Delete {episode_name}?\nThis cannot be undone."):
            return

        # Delete folder
        try:
            shutil.rmtree(episode_dir)
            print(f"Deleted: {episode_name}")

            # Remove from list and labels
            self.labels.pop(episode_name, None)
            self.episodes.remove(episode_name)

            # Show next or previous episode
            if self.current_index >= len(self.episodes):
                self.current_index = len(self.episodes) - 1
            if self.episodes:
                self.show_episode(self.current_index)
            else:
                messagebox.showinfo("Info", "No episodes remaining.")

        except Exception as e:
            messagebox.showerror("Error", f"Failed to delete: {e}")

    def update_stats(self):
        """Update statistics"""
        total = len(self.episodes)
        labeled = len(self.labels)
        put_in = sum(1 for v in self.labels.values() if "Put" in v)
        take_out = sum(1 for v in self.labels.values() if "Take" in v)

        self.stats_label.config(
            text=f"Labeled: {labeled}/{total} | Put In: {put_in} | Take Out: {take_out} | Not Selected: {total - labeled}"
        )

def main():
    # Default path or command line argument
    data_path = None
    if len(sys.argv) > 1:
        data_path = sys.argv[1]

    root = tk.Tk()
    app = EpisodeLabeler(root, data_path)
    root.mainloop()

if __name__ == "__main__":
    main()
