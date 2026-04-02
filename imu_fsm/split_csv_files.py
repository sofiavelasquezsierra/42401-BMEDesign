from pathlib import Path
import csv
from typing import List

# --- Configuration ---
PRE_ROWS = 1
POST_ROWS = 5
TARGET_STATE = "CHECK_FALL"
REQUIRED_LEN = 200
MIN_POST_CHECK_ROWS = 2

SCRIPT_DIR = Path(__file__).resolve().parent


# --- Helpers ---
def find_check_fall_windows(states):
    windows = []
    i = 0
    n = len(states)

    while i < n:
        if states[i] == TARGET_STATE:
            start = i
            while i + 1 < n and states[i + 1] == TARGET_STATE:
                i += 1
            end = i
            windows.append((start, end))
        i += 1

    return windows


def expand_window(start, end, n_rows):
    return max(0, start - PRE_ROWS), min(n_rows - 1, end + POST_ROWS)


# --- Core processing ---
def process_csv_file(input_path: Path, output_dir: Path):
    base_name = input_path.stem
    print(f"\nProcessing: {input_path}")

    with input_path.open('r', newline='') as f:
        reader = csv.reader(f)
        header = next(reader)
        rows = list(reader)

    if not rows:
        print("empty file")
        return

    states = [row[11] for row in rows]
    raw_windows = find_check_fall_windows(states)
    print(f"Found {len(raw_windows)} CHECK_FALL segments")
    file_idx = 0

    for start, end in raw_windows:
        block_len = end - start + 1

        if block_len < REQUIRED_LEN:
            continue

        s, e = expand_window(start, end, len(rows))
        post_rows = e - end
        if(post_rows < MIN_POST_CHECK_ROWS):
            print("Not enough rows after CHECK_FALL section")
            continue
        
        out_path = output_dir / f"{base_name}_{file_idx}.csv"
        file_idx += 1

        with out_path.open('w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(rows[s:e+1])


def process_folders(input_folders: List[str], output_root="separated_csv_files"):
    output_root = SCRIPT_DIR / output_root
    output_root.mkdir(exist_ok=True)

    for folder in input_folders:
        folder_path = (SCRIPT_DIR / folder).resolve()

        if not folder_path.exists():
            raise FileNotFoundError(f"Input folder not found: {folder_path}")

        folder_name = folder_path.name
        output_dir = output_root / f"{folder_name}_separated"
        output_dir.mkdir(exist_ok=True)

        for csv_file in folder_path.glob("*.csv"):
            process_csv_file(csv_file, output_dir)


# --- Entry point ---
if __name__ == "__main__":
    input_folders = [
        "harry_full_fsm"
    ]

    process_folders(input_folders)