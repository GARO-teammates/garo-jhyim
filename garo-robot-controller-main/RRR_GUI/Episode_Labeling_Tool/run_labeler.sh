#!/bin/bash
#
# Episode Labeling Tool Runner
#
# Usage:
#   ./run_labeler.sh                    # Use default path
#   ./run_labeler.sh /path/to/episodes  # Use custom path
#

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -n "$1" ]; then
    python3 "$SCRIPT_DIR/episode_labeler.py" "$1"
else
    python3 "$SCRIPT_DIR/episode_labeler.py"
fi
