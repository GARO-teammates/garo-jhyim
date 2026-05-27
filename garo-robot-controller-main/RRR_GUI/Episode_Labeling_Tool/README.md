# Episode Labeling Tool

A GUI tool for labeling robot teleoperation episodes. View the first and last frame of each episode to quickly classify the task type.

## Features

- View first/last frames of each episode side by side
- Quick labeling with keyboard shortcuts
- Delete unwanted episodes
- Labels are saved to `episode_data.json` and `metadata.json`
- Progress tracking and statistics

## Requirements

```bash
pip install pillow
```

## Usage

### Option 1: Run Script
```bash
cd Episode_Labeling_Tool
chmod +x run_labeler.sh
./run_labeler.sh
```

### Option 2: Python Direct
```bash
python3 episode_labeler.py
```

### Option 3: Custom Data Path
```bash
python3 episode_labeler.py /path/to/your/episodes
```

## Keyboard Shortcuts

| Key | Action |
|-----|--------|
| `W` | Label as "Take Out" + Next |
| `S` | Label as "Put In" + Next |
| `A` or `←` | Previous Episode |
| `D` or `→` | Next Episode |
| `-` | Delete Current Episode |
| `Ctrl+S` | Save All Labels |

## Label Types

- **Put In (put_in)**: Putting snack into the box
- **Take Out (take_out)**: Taking snack from the box

## Data Structure

The tool expects episodes in this structure:
```
episodes_folder/
├── episode_0000/
│   ├── episode_data.json
│   ├── metadata.json
│   └── observation_images_top/
│       ├── frame_000000.jpg
│       ├── frame_000001.jpg
│       └── ...
├── episode_0001/
│   └── ...
└── ...
```

## Output

Labels are saved to:
- `episode_data.json`: `task_type` field added to each frame
- `metadata.json`: `task_type` field added

Example metadata.json after labeling:
```json
{
  "task_type": "put_in"
}
```
