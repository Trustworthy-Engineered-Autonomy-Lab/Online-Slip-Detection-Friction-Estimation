#!/usr/bin/env python3
"""
event_level_drift_eval.py

Purpose:
--------
Go through exported ROS bag CSV folders, reconstruct the linear drift score,
apply thresholds, suppress repeated detections using the same "mute/cooldown" idea
as the ROS algorithm, and label each detection/event as:

    TP = detection within +/- MATCH_WINDOW_SEC of a real video-labeled drift
    FN = real drift event with no matching detection
    FP = detection that does not match any real drift event

This script is intentionally written with clearly labeled sections so you can edit it.
"""

# =============================================================================
# 0. IMPORTS
# =============================================================================

from pathlib import Path
import re
import csv
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# =============================================================================
# 1. USER SETTINGS — CHANGE THESE FIRST
# =============================================================================

# Root folder containing your extracted CSV folders.
# Example structure:
# DATA_ROOT/
#   3surface_track1_assam/1csv/
#   3surface_track1_assam/2csv/
#   ...
#   3surface_track1_assam/20csv/
#   3s_t2_assam/1/
#   ...
#   3s_t2_assam/16/
#   2surface_csvs/2surface_1/
#   ...
#   2surface_csvs/2surface_20/
DATA_ROOT = Path(".")  # Keep as current directory, but see find_bag_csv_folders below

# Ground truth label CSVs.

# These should contain the true "Time Video" drift times.
LABEL_CSVS = [
    Path("drifting data - Assam Track 1.csv"),
    Path("drifting data - Assam Track 2.csv"),
]

# Set correct columns for your CSVs (Bag Number, Surface, Time (Video))
LABEL_COLS = {
    "bag": 0,
    "surface": 5,
    "time_video": 6,
}

# New 2-surface dataset. This gets its own internal track ID so it does not
# collide with the existing Track 1 and Track 2 datasets above.
TWO_SURFACE_TRACK_ID = 3
TWO_SURFACE_SURFACE_NAME = "2surface"
TWO_SURFACE_BASE_FOLDER = "2surface_csvs"
TWO_SURFACE_FOLDER_TEMPLATE = "2surface_{bag}"
TWO_SURFACE_LABEL_CSV_CANDIDATES = [
    Path("2surfacedrifttiming.csv"),
    Path("2surfacedrifttiming - Sheet1.csv"),
]

# These are the absolute ROS2 output timestamps you provided for the old
# detector's two detections in each 2surface bag. The matching video-relative
# detected timestamps come from 2surfacedrifttiming.csv, and the offset is:
#
#     video_bag_offset = absolute_ros_timestamp - detected_timestamp_video
#
# The two detections in each bag should give the same offset; the code averages
# them defensively in case of tiny floating-point / export differences.
TWO_SURFACE_ABSOLUTE_DETECTED_TIMESTAMPS = {
    1: (1756824532.995317, 1756824536.9953597),
    2: (1756824592.7559733, 1756824596.9560103),
    3: (1756824916.419064, 1756824919.6549032),
    4: (1756825109.7963068, 1756825113.3612063),
    5: (1756825178.2816541, 1756825182.001722),
    6: (1756825212.5754862, 1756825219.7973437),
    7: (1756825269.9376612, 1756825272.677416),
    8: (1756825341.418298, 1756825344.8786516),
    9: (1756825382.4588776, 1756825385.4590747),
    10: (1756825414.2048078, 1756825416.9396582),
    11: (1756825473.4014997, 1756825476.6064627),
    12: (1756825505.447099, 1756825509.5469658),
    13: (1756825577.788553, 1756825580.9032416),
    14: (1756825756.7466297, 1756825759.9666233),
    15: (1756825822.9532084, 1756825827.5533545),
    16: (1756825908.3174968, 1756825911.65037),
    17: (1756825943.6512277, 1756825946.6016338),
    18: (1756825973.456783, 1756825976.6740408),
    19: (1756826023.8563006, 1756826027.0960917),
    20: (1756826068.4125636, 1756826071.5365987),
}

# Filled by load_two_surface_ground_truth_events_and_offsets().
# This is used only when the 2surface CSV export has relative `time` columns
# instead of absolute ROS sec/nanosec timestamps.
TWO_SURFACE_RELATIVE_VIDEO_OFFSETS = {}

# Output folder.
OUTPUT_DIR = Path("event_eval_results")

# K-fold train/test splitting.
# Each fold trains on 4/5 of the labeled bags and leaves 1/5 for testing.
# Across all 5 folds, every labeled bag is used as a test bag exactly once.
K_FOLDS = 5
RANDOM_SEED = 42  # Change this if you want a different deterministic shuffle.

# Event matching rule.
# A detection is a TP if it is within +/- this many seconds of the true video drift time.
MATCH_WINDOW_SEC = 1.0
EVAL_AFTER_LAST_EVENT_SEC = 3.0

# Data muting / repeat suppression.
# After one detection, ignore additional detections for this many seconds.
# This mirrors the ROS node's drift_length = 1.5 behavior.
MUTE_AFTER_DETECTION_SEC = 1.5

# Vehicle parameters from your ROS node.
WHEELBASE_M = 0.32

##### VIDEO TIME OFFSETS
VIDEO_BAG_OFFSETS = {
    # Track 1 - Assam (3surface_track1_assam)
    (1, 1): 1770685224.332076875,
    (1, 2): 1770685252.037954463,
    (1, 3): 1770685276.093463964,
    (1, 4): 1770685297.349485609,
    (1, 5): 1770685323.98266618,
    (1, 6): 1770685345.668551213,
    (1, 7): 1770685368.282913669,
    (1, 8): 1770685416.034880254,
    (1, 9): 1770685436.733885476,
    (1, 10): 1770685458.221413463,
    (1, 11): 1770685478.22223772,
    (1, 12): 1770685514.755985375,
    (1, 13): 1770685538.688959557,
    (1, 14): 1770685560.270502758,
    (1, 15): 1770685581.440852002,
    (1, 16): 1770685601.665632441,
    (1, 17): 1770685622.163354407,
    (1, 18): 1770685648.66353143,
    (1, 19): 1770685680.921996843,
    (1, 20): 1770685700.803396095,
    
    # Track 2 - Assam (3s_t2_assam)
    (2, 1): 1770760642.629366108,
    (2, 2): 1770760670.666420022,
    (2, 3): 1770760695.849173156,
    (2, 4): 1770760719.599157819,
    (2, 5): 1770760744.473020467,
    (2, 6): 1770760805.249555805,
    (2, 7): 1770760877.651752788,
    (2, 8): 1770760916.338523462,
    (2, 9): 1770760993.909104212,
    (2, 10): 1770761024.909621551,
    (2, 11): 1770761046.913041670,
    (2, 12): 1770761068.512913042,
    (2, 13): 1770761093.282338231,
    (2, 14): 1770761144.407258708,
    (2, 15): 1770761168.722219611,
    (2, 16): 1770761209.086526603,
}


# Thresholds to test.
# Put the values you want here.
# You can also use np.linspace(...) if you want to sweep.
LINEAR_THRESHOLDS = [
    1.0,
    1.5,
    2.0,
    2.5,
    3.0,
]

# Epsilons to test in the normalized linear score denominator:
#
#     linear_score = abs(2*|odom_filtered_xy| - 2*odom_raw_vx) / (abs(2*odom_raw_vx) + epsilon)
#
# This is the same score logic as before, except epsilon is swept instead of hard-coded
# to 0.05.
EPSILONS = [
    0.005,
    0.01,
    0.025,
    0.05,
    0.075,
    0.10,
    0.15,
    0.20,
    0.30,
    0.50,
]

# Only used for the per-bag debug score CSVs so you can still see the old fixed
# epsilon score. The sweep itself does not depend on this value.
REFERENCE_DEBUG_EPSILON = 0.05

# If True, the script will generate threshold values automatically from the score ranges.
# If False, it uses LINEAR_THRESHOLDS above.
AUTO_SWEEP_THRESHOLDS = True
N_AUTO_THRESHOLDS = 100

# If True, the script will generate epsilon values automatically.
# If False, it uses EPSILONS above.
AUTO_SWEEP_EPSILONS = True
N_AUTO_EPSILONS = 100
EPSILON_MIN = 0.005
EPSILON_MAX = 5.0


# =============================================================================
# 2. CSV COLUMN MAPPING — EDIT THIS IF YOUR COLUMNS ARE DIFFERENT
# =============================================================================
"""
Important:
----------
Your exported CSVs may not have headers. This script reads with header=None.

You need to set which column contains each value.

For ROS-style CSV exports, common layouts are often:

ackermann_cmd.csv:
    col 0 = header.stamp.sec
    col 1 = header.stamp.nanosec
    some later column = steering_angle
    some later column = speed

imu.csv:
    col 0 = header.stamp.sec
    col 1 = header.stamp.nanosec
    some later column = angular_velocity.z

odometry_filtered.csv:
    col 0 = header.stamp.sec
    col 1 = header.stamp.nanosec
    some later column = twist.twist.linear.x
    some later column = twist.twist.linear.y

You MUST verify these by opening the first few rows of each CSV.
"""

ACKERMANN_COLS = {
    "sec": 0,
    "nanosec": 1,
    "steering_angle": 3,  # Corrected index
    "speed": 5,           # Corrected index
}

IMU_COLS = {
    "sec": 0,
    "nanosec": 1,
    "angular_velocity_z": 18,  # Corrected index
}

ODOM_RAW_COLS = {
    "sec": 0,
    "nanosec": 1,
    "linear_x": 47,
    "linear_y": 48,
}

ODOM_FILTERED_COLS = {
    "sec": 0,
    "nanosec": 1,
    "linear_x": 47,
    "linear_y": 48, 
}


# =============================================================================
# 3. LABEL CSV MAPPING — EDIT THIS IF YOUR LABEL SHEET IS DIFFERENT
# =============================================================================
"""
This script assumes your label CSV has, somewhere in each row:

    Bag number
    Surface
    Time Video

The easiest setup is to tell the script the column numbers.

Example from your screenshots looked roughly like:
    Bag | Surface | Time Video | Time Bag | ...

If your CSV has actual column names, you can modify load_ground_truth_events().
"""



# Optional: Track ID is inferred from the filename:
# "Track 1" -> track = 1
# "Track 2" -> track = 2


# =============================================================================
# 4. HELPER FUNCTIONS
# =============================================================================

def make_time_sec(df: pd.DataFrame, sec_col: int, nanosec_col: int) -> pd.Series:
    """Combine ROS sec and nanosec into one floating-point timestamp."""
    return pd.to_numeric(df.iloc[:, sec_col], errors="coerce") + (
        pd.to_numeric(df.iloc[:, nanosec_col], errors="coerce") * 1e-9
    )


def read_multiline_named_csv(path: Path) -> pd.DataFrame:
    """
    Read newer CSV exports that have a header row and a `time` column.

    Some ROS message fields such as covariance arrays are written across
    multiple physical text lines without CSV quoting. A normal pd.read_csv()
    then treats one ROS message as several dataframe rows. This reader rebuilds
    each logical message row before parsing it.
    """
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        header = f.readline().rstrip("\n")
        columns = [c.strip() for c in header.split(",")]

        # New logical records start with a numeric time followed by a comma.
        # Continuation lines from covariance arrays usually start with spaces.
        record_start = re.compile(
            r"^[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?,"
        )

        records = []
        buffer = []

        for line in f:
            line = line.rstrip("\n")

            if record_start.match(line) and buffer:
                records.append(" ".join(part.strip() for part in buffer))
                buffer = [line]
            else:
                buffer.append(line)

        if buffer:
            records.append(" ".join(part.strip() for part in buffer))

    parsed_rows = []
    for record in records:
        try:
            fields = next(csv.reader([record]))
        except Exception:
            continue

        # Be defensive if a malformed row is shorter/longer than the header.
        if len(fields) < len(columns):
            fields = fields + [np.nan] * (len(columns) - len(fields))
        elif len(fields) > len(columns):
            fields = fields[:len(columns)]

        parsed_rows.append(fields)

    return pd.DataFrame(parsed_rows, columns=columns)


def read_no_header_csv(path: Path) -> pd.DataFrame:
    """
    Read either the old headerless CSV exports or the newer named-column exports.

    Old files:
        sec,nanosec,... with no useful header row.

    New 2surface files:
        time,drive.speed,...
        time,twist.twist.linear.x,...
        time,imu.angular_velocity.z,...

    This function intentionally only changes CSV loading. The detector logic
    downstream stays the same.
    """
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        first_line = f.readline().strip()

    first_cols = [c.strip() for c in first_line.split(",")]
    has_named_time_header = (
        len(first_cols) > 1
        and first_cols[0].lower() == "time"
        and any(not re.fullmatch(r"[+-]?\d+(\.\d+)?", c) for c in first_cols[1:])
    )

    if has_named_time_header:
        return read_multiline_named_csv(path)

    # Old bag CSVs occasionally have one malformed line. Skipping a single bad
    # export row is better than dropping the entire bag.
    return pd.read_csv(
        path,
        header=None,
        engine="python",
        on_bad_lines="skip",
    )


def get_numeric_column(
    df: pd.DataFrame,
    names: str | list[str],
    fallback_index: int,
) -> pd.Series:
    """
    Return a numeric series using named CSV columns when present, otherwise the
    old fixed numeric column index.
    """
    if isinstance(names, str):
        names = [names]

    for name in names:
        if name in df.columns:
            return pd.to_numeric(df[name], errors="coerce")

    return pd.to_numeric(df.iloc[:, fallback_index], errors="coerce")


def get_time_column(
    df: pd.DataFrame,
    sec_index: int,
    nanosec_index: int,
) -> pd.Series:
    """
    Return bag time.

    New 2surface exports use a single relative `time` column.
    Old exports use sec + nanosec columns.
    """
    if "time" in df.columns:
        return pd.to_numeric(df["time"], errors="coerce")

    return make_time_sec(df, sec_index, nanosec_index)


def infer_track_from_filename(path: Path) -> int | None:
    """Infer track number from label CSV filename."""
    name = path.name.lower()
    if "track 1" in name or "t1" in name:
        return 1
    if "track 2" in name or "t2" in name:
        return 2
    if "2surface" in name:
        return TWO_SURFACE_TRACK_ID
    return None


def infer_bag_number_from_folder(folder: Path) -> int | None:
    """
    Infer bag number from folder name.

    This tries to grab the last integer in the folder name.
    Example:
        3surf_t1_a_12 -> 12
    """
    nums = re.findall(r"\d+", folder.name)
    if not nums:
        return None
    return int(nums[-1])


def infer_track_from_folder(folder: Path) -> int | None:
    """
    Infer track number from folder name.
    Edit this if your folder naming is different.
    """
    name = folder.name.lower()
    if "t1" in name or "track1" in name or "track_1" in name:
        return 1
    if "t2" in name or "track2" in name or "track_2" in name:
        return 2
    return None


def first_existing_path(candidates: list[Path]) -> Path | None:
    """Return the first path in candidates that exists, otherwise None."""
    for path in candidates:
        if path.exists():
            return path
    return None


def bag_key(track: int, bag: int) -> tuple[int, int]:
    """Normalize track/bag into the dictionary key format used throughout."""
    return (int(track), int(bag))


def bag_name(track: int, bag: int) -> str:
    """Human-readable bag name for CSV outputs."""
    track = int(track)
    bag = int(bag)
    if track == TWO_SURFACE_TRACK_ID:
        return f"2surface_bag{bag}"
    return f"track{track}_bag{bag}"


def folder_keys(folders_df: pd.DataFrame) -> set[tuple[int, int]]:
    """Return the set of (track, bag) keys represented by a folders dataframe."""
    return {
        bag_key(row["track"], row["bag"])
        for _, row in folders_df.iterrows()
    }


def select_events_for_folders(events: pd.DataFrame, folders_df: pd.DataFrame) -> pd.DataFrame:
    """Return only ground-truth events belonging to the folders in folders_df."""
    keys = folder_keys(folders_df)
    return events[
        events.apply(lambda r: bag_key(r["track"], r["bag"]) in keys, axis=1)
    ].copy()


def select_scores_for_folders(
    all_scores: dict[tuple[int, int], pd.DataFrame],
    folders_df: pd.DataFrame,
) -> dict[tuple[int, int], pd.DataFrame]:
    """Return only score dataframes belonging to the folders in folders_df."""
    keys = folder_keys(folders_df)
    return {key: score_df for key, score_df in all_scores.items() if key in keys}


def load_two_surface_ground_truth_events_and_offsets() -> tuple[pd.DataFrame, dict]:
    """
    Load the new 2surface timing sheet.

    The sheet format is one block per bag:
        row N:   Bag k, Video Timestamp 1, value, Video Timestamp 2, value
        row N+1:       Detected Timestamp 1, value, Detected Timestamp 2, value

    The video timestamps become the true event labels. The detected timestamps
    are only used to compute the ROS-time-to-video-time offset for each bag.
    """
    csv_path = first_existing_path(TWO_SURFACE_LABEL_CSV_CANDIDATES)
    if csv_path is None:
        print(
            "⚠️  No 2surface timing sheet found. Tried: "
            + ", ".join(str(p) for p in TWO_SURFACE_LABEL_CSV_CANDIDATES)
        )
        return pd.DataFrame(), {}

    raw = pd.read_csv(csv_path, header=None)

    global TWO_SURFACE_RELATIVE_VIDEO_OFFSETS

    event_rows = []
    offset_rows = {}
    relative_offset_rows = {}
    offset_debug_rows = []

    for idx, row in raw.iterrows():
        bag_raw = str(row.iloc[0])
        bag_match = re.search(r"bag\s*(\d+)", bag_raw, flags=re.IGNORECASE)
        if not bag_match:
            continue

        bag = int(bag_match.group(1))

        # Ground-truth video timestamps are in columns 2 and 4 on the bag row.
        video_times = []
        for col in [2, 4]:
            time_video = pd.to_numeric(row.iloc[col], errors="coerce")
            if not pd.isna(time_video):
                video_times.append(float(time_video))

        # The new 2surface bags have two slips; do not try to read a third event.
        for event_number, time_video in enumerate(video_times[:2], start=1):
            event_rows.append({
                "track": TWO_SURFACE_TRACK_ID,
                "bag": bag,
                "surface": TWO_SURFACE_SURFACE_NAME,
                "event_time_video": float(time_video),
                "event_number_in_bag": event_number,
            })

        # Detected timestamps are on the next row in columns 2 and 4.
        detected_video_times = []
        if idx + 1 < len(raw):
            det_row = raw.iloc[idx + 1]
            for col in [2, 4]:
                detected_video = pd.to_numeric(det_row.iloc[col], errors="coerce")
                if not pd.isna(detected_video):
                    detected_video_times.append(float(detected_video))

        # Relative offset for the new named-column CSV export:
        #     t_video = t_bag - mean(detected_relative_time - video_timestamp)
        #
        # This is used when the actual CSV `time` column is relative seconds.
        if detected_video_times and video_times:
            n_rel = min(len(detected_video_times), len(video_times))
            relative_offset_estimates = [
                float(detected_video_times[i]) - float(video_times[i])
                for i in range(n_rel)
            ]
            relative_offset_rows[(TWO_SURFACE_TRACK_ID, bag)] = float(
                np.mean(relative_offset_estimates)
            )
        else:
            relative_offset_estimates = []

        # Absolute offset for CSV exports that contain absolute ROS timestamps:
        #     t_video = t_bag - mean(absolute_ros_detection - detected_relative_time)
        absolute_detected_times = TWO_SURFACE_ABSOLUTE_DETECTED_TIMESTAMPS.get(bag)
        if absolute_detected_times is not None and detected_video_times:
            n = min(len(absolute_detected_times), len(detected_video_times))
            offset_estimates = [
                float(absolute_detected_times[i]) - float(detected_video_times[i])
                for i in range(n)
            ]
            offset_rows[(TWO_SURFACE_TRACK_ID, bag)] = float(np.mean(offset_estimates))

            offset_debug_rows.append({
                "track": TWO_SURFACE_TRACK_ID,
                "bag": bag,
                "absolute_offset_used": float(np.mean(offset_estimates)),
                "absolute_offset_1": offset_estimates[0] if len(offset_estimates) > 0 else np.nan,
                "absolute_offset_2": offset_estimates[1] if len(offset_estimates) > 1 else np.nan,
                "absolute_offset_difference": (
                    offset_estimates[1] - offset_estimates[0]
                    if len(offset_estimates) > 1 else np.nan
                ),
                "relative_offset_used": (
                    float(np.mean(relative_offset_estimates))
                    if relative_offset_estimates else np.nan
                ),
                "relative_offset_1": (
                    relative_offset_estimates[0]
                    if len(relative_offset_estimates) > 0 else np.nan
                ),
                "relative_offset_2": (
                    relative_offset_estimates[1]
                    if len(relative_offset_estimates) > 1 else np.nan
                ),
                "relative_offset_difference": (
                    relative_offset_estimates[1] - relative_offset_estimates[0]
                    if len(relative_offset_estimates) > 1 else np.nan
                ),
            })

    events = pd.DataFrame(event_rows)

    if not events.empty:
        # Save this later into OUTPUT_DIR if main has already created it.
        events = events.sort_values(["track", "bag", "event_time_video"]).reset_index(drop=True)

    TWO_SURFACE_RELATIVE_VIDEO_OFFSETS = relative_offset_rows

    if offset_debug_rows:
        # This global is only for debugging/export in main.
        load_two_surface_ground_truth_events_and_offsets.offset_debug_df = pd.DataFrame(offset_debug_rows)
    else:
        load_two_surface_ground_truth_events_and_offsets.offset_debug_df = pd.DataFrame()

    print(
        f"Loaded {len(events)} 2surface ground-truth events "
        f"from {csv_path} and computed {len(offset_rows)} offsets."
    )

    return events, offset_rows


# =============================================================================
# 5. LOAD GROUND TRUTH VIDEO DRIFT TIMES
# =============================================================================

def load_ground_truth_events(label_csvs: list[Path]) -> pd.DataFrame:
    """
    Load manually labeled drift events from your label CSVs.

    Output columns:
        track
        bag
        surface
        event_time_video
        event_id
    """
    all_events = []

    # -------------------------------------------------------------------------
    # Existing Track 1 / Track 2 label loader
    # -------------------------------------------------------------------------
    for csv_path in label_csvs:
        if not csv_path.exists():
            print(f"⚠️  Label CSV not found, skipping: {csv_path}")
            continue

        track = infer_track_from_filename(csv_path)
        if track is None:
            print(f"⚠️  Could not infer track from label CSV name, skipping: {csv_path}")
            continue

        raw = pd.read_csv(csv_path, header=None)

        # Forward-fill Bag Number column so all rows for a bag have the correct bag number
        bag_col = raw.iloc[:, LABEL_COLS["bag"]].replace('', np.nan)
        bag_col = bag_col.ffill()
        raw.iloc[:, LABEL_COLS["bag"]] = bag_col

        valid_count = 0
        for idx, row in raw.iterrows():
            bag_raw = str(row.iloc[LABEL_COLS["bag"]])
            # Extract integer from 'Bag 1' style string
            bag_match = re.search(r"(\d+)", bag_raw)
            bag = int(bag_match.group(1)) if bag_match else np.nan
            time_video = pd.to_numeric(row.iloc[LABEL_COLS["time_video"]], errors="coerce")

            if pd.isna(bag) or pd.isna(time_video):
                continue

            surface = str(row.iloc[LABEL_COLS["surface"]]).strip()

            all_events.append({
                "track": int(track),
                "bag": int(bag),
                "surface": surface,
                "event_time_video": float(time_video),
            })
            valid_count += 1

        print(f"Loaded {valid_count} events from {csv_path}.")

    # -------------------------------------------------------------------------
    # New 2surface label loader + offset builder
    # -------------------------------------------------------------------------
    two_surface_events, two_surface_offsets = load_two_surface_ground_truth_events_and_offsets()
    if not two_surface_events.empty:
        all_events.extend(two_surface_events.to_dict("records"))

    # Make the dynamically computed 2surface offsets available to compute_scores_for_bag().
    VIDEO_BAG_OFFSETS.update(two_surface_offsets)

    events = pd.DataFrame(all_events)

    if events.empty:
        raise RuntimeError(
            "No ground truth events loaded. Check LABEL_CSVS, LABEL_COLS, and the 2surface timing sheet."
        )

    events = events.sort_values(["track", "bag", "event_time_video"]).reset_index(drop=True)
    events["event_id"] = np.arange(len(events))

    return events


# =============================================================================
# 6. FIND BAG CSV FOLDERS
# =============================================================================

def find_bag_csv_folders(data_root: Path) -> pd.DataFrame:
    """
    Find folders that contain the CSV files needed for offline detector reconstruction.

    Required per bag:
        ackermann_cmd.csv
        odom.csv
        odometry_filtered.csv
        imu.csv or sensors_imu.csv
    """
    rows = []

    # Existing datasets
    base1 = data_root / "3surface_track1_assam"
    base2 = data_root / "3s_t2_assam"

    # New 2surface dataset from the screenshot:
    #   2surface_csvs/2surface_1/ ... 2surface_csvs/2surface_20/
    base3 = data_root / TWO_SURFACE_BASE_FOLDER

    # Track 1: 1csv to 20csv
    for i in range(1, 21):
        folder = base1 / f"{i}csv"
        ack = folder / "ackermann_cmd.csv"
        imu = first_existing_path([folder / "imu.csv", folder / "sensors_imu.csv"])
        odom_raw = folder / "odom.csv"
        odomf = folder / "odometry_filtered.csv"
        if ack.exists() and imu is not None and odom_raw.exists() and odomf.exists():
            rows.append({
                "track": 1,
                "bag": i,
                "dataset": "3surface_track1_assam",
                "folder": folder,
                "ackermann_csv": ack,
                "imu_csv": imu,
                "odom_csv": odom_raw,
                "odom_filtered_csv": odomf,
            })

    # Track 2: 1 to 16
    for i in range(1, 17):
        folder = base2 / f"{i}"
        ack = folder / "ackermann_cmd.csv"
        imu = first_existing_path([folder / "imu.csv", folder / "sensors_imu.csv"])
        odom_raw = folder / "odom.csv"
        odomf = folder / "odometry_filtered.csv"
        if ack.exists() and imu is not None and odom_raw.exists() and odomf.exists():
            rows.append({
                "track": 2,
                "bag": i,
                "dataset": "3s_t2_assam",
                "folder": folder,
                "ackermann_csv": ack,
                "imu_csv": imu,
                "odom_csv": odom_raw,
                "odom_filtered_csv": odomf,
            })

    # 2surface: 2surface_1 to 2surface_20
    for i in range(1, 21):
        folder = base3 / TWO_SURFACE_FOLDER_TEMPLATE.format(bag=i)
        ack = folder / "ackermann_cmd.csv"
        imu = first_existing_path([folder / "sensors_imu.csv", folder / "imu.csv"])
        odom_raw = folder / "odom.csv"
        odomf = folder / "odometry_filtered.csv"
        if ack.exists() and imu is not None and odom_raw.exists() and odomf.exists():
            rows.append({
                "track": TWO_SURFACE_TRACK_ID,
                "bag": i,
                "dataset": TWO_SURFACE_SURFACE_NAME,
                "folder": folder,
                "ackermann_csv": ack,
                "imu_csv": imu,
                "odom_csv": odom_raw,
                "odom_filtered_csv": odomf,
            })

    folders = pd.DataFrame(rows)

    if folders.empty:
        raise RuntimeError(
            "No CSV folders found. Check DATA_ROOT and expected folder structure."
        )

    return folders.sort_values(["track", "bag"]).reset_index(drop=True)


# =============================================================================
# 7. RECONSTRUCT DETECTOR SCORES OFFLINE
# =============================================================================

def compute_scores_for_bag(folder_row: pd.Series) -> pd.DataFrame:
    """Compute the reusable linear-score pieces for one bag folder."""
    ack = read_no_header_csv(folder_row["ackermann_csv"])
    imu = read_no_header_csv(folder_row["imu_csv"])
    odom_filtered = read_no_header_csv(folder_row["odom_filtered_csv"])
    odom_raw = read_no_header_csv(folder_row["odom_csv"])
    
    # -------------------------------------------------------------------------
    # 7A. Build clean Ackermann dataframe
    # -------------------------------------------------------------------------
    ack_clean = pd.DataFrame({
        "t_bag": get_time_column(ack, ACKERMANN_COLS["sec"], ACKERMANN_COLS["nanosec"]),
        "speed": get_numeric_column(ack, "drive.speed", ACKERMANN_COLS["speed"]),
        "steering_angle": get_numeric_column(
            ack,
            "drive.steering_angle",
            ACKERMANN_COLS["steering_angle"],
        ),
    }).dropna()
    
    # -------------------------------------------------------------------------
    # 7B. Build clean IMU dataframe
    # -------------------------------------------------------------------------
    imu_clean = pd.DataFrame({
        "t_bag": get_time_column(imu, IMU_COLS["sec"], IMU_COLS["nanosec"]),
        "angular_velocity_z": get_numeric_column(
            imu,
            ["imu.angular_velocity.z", "angular_velocity.z"],
            IMU_COLS["angular_velocity_z"],
        ),
    }).dropna()
    
    # -------------------------------------------------------------------------
    # 7C. Build clean FILTERED odometry dataframe
    # -------------------------------------------------------------------------
    odom_filtered_clean = pd.DataFrame({
        "t_bag": get_time_column(
            odom_filtered,
            ODOM_FILTERED_COLS["sec"],
            ODOM_FILTERED_COLS["nanosec"],
        ),
        "vx_filt": get_numeric_column(
            odom_filtered,
            "twist.twist.linear.x",
            ODOM_FILTERED_COLS["linear_x"],
        ),
        "vy_filt": get_numeric_column(
            odom_filtered,
            "twist.twist.linear.y",
            ODOM_FILTERED_COLS["linear_y"],
        ),
    }).dropna()
    
    # -------------------------------------------------------------------------
    # 7D. Build clean RAW odometry dataframe
    # -------------------------------------------------------------------------
    odom_raw_clean = pd.DataFrame({
        "t_bag": get_time_column(odom_raw, ODOM_RAW_COLS["sec"], ODOM_RAW_COLS["nanosec"]),
        "vx_raw": get_numeric_column(
            odom_raw,
            "twist.twist.linear.x",
            ODOM_RAW_COLS["linear_x"],
        ),
        "vy_raw": get_numeric_column(
            odom_raw,
            "twist.twist.linear.y",
            ODOM_RAW_COLS["linear_y"],
        ),
    }).dropna()
    
    # Sort for merge_asof
    ack_clean = ack_clean.sort_values("t_bag")
    imu_clean = imu_clean.sort_values("t_bag")
    odom_filtered_clean = odom_filtered_clean.sort_values("t_bag")
    odom_raw_clean = odom_raw_clean.sort_values("t_bag")
    
    # -------------------------------------------------------------------------
    # 7E. Synchronize streams by nearest timestamp
    # -------------------------------------------------------------------------
    merged = pd.merge_asof(
        odom_filtered_clean,
        ack_clean,
        on="t_bag",
        direction="nearest",
        tolerance=0.05,
    )
    merged = pd.merge_asof(
        merged.sort_values("t_bag"),
        imu_clean,
        on="t_bag",
        direction="nearest",
        tolerance=0.05,
    )
    merged = pd.merge_asof(
        merged.sort_values("t_bag"),
        odom_raw_clean,
        on="t_bag",
        direction="nearest",
        tolerance=0.05,
    )
    merged = merged.dropna().reset_index(drop=True)
    
    if merged.empty:
        return merged
    
    # -------------------------------------------------------------------------
    # 7F. Convert bag time to video-relative time
    # -------------------------------------------------------------------------
    track = int(folder_row["track"])
    bag = int(folder_row["bag"])
    offset_key = (track, bag)

    if offset_key in VIDEO_BAG_OFFSETS:
        # Old Track 1 / Track 2 CSVs use absolute ROS timestamps, so subtract
        # the absolute video-bag offset.
        #
        # The new 2surface CSVs you uploaded use a relative `time` column
        # starting near zero. For those, subtract only the small relative offset
        # from the timing sheet. This keeps the timestamp alignment correct
        # without touching the detector score/muting/matching logic.
        is_relative_time_export = merged["t_bag"].dropna().median() < 1_000_000

        if (
            track == TWO_SURFACE_TRACK_ID
            and is_relative_time_export
            and offset_key in TWO_SURFACE_RELATIVE_VIDEO_OFFSETS
        ):
            video_t0 = TWO_SURFACE_RELATIVE_VIDEO_OFFSETS[offset_key]
        else:
            video_t0 = VIDEO_BAG_OFFSETS[offset_key]

        merged["t_video"] = merged["t_bag"] - video_t0
    else:
        # Fallback for bags without offset data
        print(f"⚠️  No offset for Track {track} Bag {bag}, using first ackermann")
        t0 = ack_clean["t_bag"].iloc[0]
        merged["t_video"] = merged["t_bag"] - t0

    MAX_VIDEO_DURATION = 30.0  # Adjust if your videos are longer
    merged = merged[merged["t_video"] <= MAX_VIDEO_DURATION].copy()

    if merged.empty:
        print(f"⚠️  Warning: No data within first {MAX_VIDEO_DURATION}s for Track {track} Bag {bag}")
        return merged
    
    # -------------------------------------------------------------------------
    # 7G. Compute reusable linear-score pieces
    # -------------------------------------------------------------------------
    odomfil_comb = merged["vx_filt"]
    odom_raw_comb = merged["vx_raw"]
    merged["linear_abs_error"] = np.abs(odomfil_comb - odom_raw_comb)
    merged["odom_raw_abs"] = np.abs(odomfil_comb)
    merged["linear_score_eps_0p05"] = (
        merged["linear_abs_error"] / (merged["odom_raw_abs"] + REFERENCE_DEBUG_EPSILON)
    )

    return merged[[
        "t_bag",
        "t_video",
        "linear_abs_error",
        "odom_raw_abs",
        "linear_score_eps_0p05",
    ]].dropna()

# =============================================================================
# 8. DATA MUTING / REPEAT DETECTION SUPPRESSION
# =============================================================================

def get_detection_times_with_muting(
    score_df: pd.DataFrame,
    score_col: str,
    threshold: float,
    mute_sec: float,
) -> list[float]:
    """
    Convert continuous scores into event detection timestamps.

    Matches the ROS behavior more closely:
    - detect whenever score is above threshold
    - suppress repeat detections for mute_sec seconds
    - no rising-edge requirement
    """
    detection_times = []
    last_detection_time = -np.inf

    for _, row in score_df.iterrows():
        t = float(row["t_video"])
        score = float(row[score_col])

        if score > threshold and (t - last_detection_time) > mute_sec:
            detection_times.append(t)
            last_detection_time = t

    return detection_times


# =============================================================================
# 9. EVENT MATCHING: TP / FP / FN
# =============================================================================

def match_detections_to_events(
    detections: list[float],
    events_for_bag: pd.DataFrame,
    match_window_sec: float,
) -> tuple[pd.DataFrame, pd.DataFrame, dict]:

    detections = sorted(detections)

    event_rows = []
    detection_rows = []

    used_detection_indices = set()

    # -------------------------------------------------------------------------
    # 9A. Match each true event to the closest unused detection inside window
    # -------------------------------------------------------------------------
    for _, event in events_for_bag.iterrows():
        event_time = float(event["event_time_video"])

        candidates = []
        for i, det_time in enumerate(detections):
            if i in used_detection_indices:
                continue

            dt = det_time - event_time
            if abs(dt) <= match_window_sec:
                candidates.append((abs(dt), i, det_time, dt))

        if candidates:
            candidates.sort()
            _, best_i, best_det_time, best_dt = candidates[0]
            used_detection_indices.add(best_i)

            event_rows.append({
                "event_id": event["event_id"],
                "surface": event["surface"],
                "event_time_video": event_time,
                "matched_detection_time": best_det_time,
                "latency_sec": best_dt,
                "event_label": "TP",
            })
        else:
            event_rows.append({
                "event_id": event["event_id"],
                "surface": event["surface"],
                "event_time_video": event_time,
                "matched_detection_time": np.nan,
                "latency_sec": np.nan,
                "event_label": "FN",
            })

    # -------------------------------------------------------------------------
    # 9B. Label every detection as TP or FP, plus nearest true event info
    # -------------------------------------------------------------------------
    event_times = events_for_bag["event_time_video"].astype(float).to_numpy()

    for i, det_time in enumerate(detections):

        if len(event_times) > 0:
            signed_dts = det_time - event_times
            nearest_idx = int(np.argmin(np.abs(signed_dts)))
            nearest_event_time = float(event_times[nearest_idx])
            nearest_event_signed_dt = float(signed_dts[nearest_idx])
            nearest_event_distance = abs(nearest_event_signed_dt)
        else:
            nearest_event_time = np.nan
            nearest_event_signed_dt = np.nan
            nearest_event_distance = np.nan

        detection_rows.append({
            "detection_time_video": det_time,
            "detection_label": "TP" if i in used_detection_indices else "FP",
            "nearest_event_time": nearest_event_time,
            "nearest_event_signed_dt": nearest_event_signed_dt,
            "nearest_event_distance": nearest_event_distance,
        })

    event_df = pd.DataFrame(event_rows)
    detection_df = pd.DataFrame(detection_rows)

    TP = int((event_df["event_label"] == "TP").sum()) if not event_df.empty else 0
    FN = int((event_df["event_label"] == "FN").sum()) if not event_df.empty else 0
    FP = int((detection_df["detection_label"] == "FP").sum()) if not detection_df.empty else 0

    precision = TP / (TP + FP) if (TP + FP) > 0 else 0.0
    recall = TP / (TP + FN) if (TP + FN) > 0 else 0.0
    f1 = 2 * precision * recall / (precision + recall) if (precision + recall) > 0 else 0.0

    metrics = {
        "TP": TP,
        "FP": FP,
        "FN": FN,
        "precision": precision,
        "recall": recall,
        "f1": f1,
    }

    return event_df, detection_df, metrics


# =============================================================================
# 10. EVALUATE LINEAR DETECTOR AT ONE THRESHOLD/EPSILON COMBO
# =============================================================================

def add_linear_score_for_epsilon(score_df: pd.DataFrame, epsilon: float) -> pd.DataFrame:
    """Create the normalized linear score for a specific epsilon value."""
    out = score_df.copy()
    out["linear_score"] = out["linear_abs_error"] / (out["odom_raw_abs"] + epsilon)
    return out


def evaluate_linear_at_threshold_epsilon(
    all_scores: dict,
    events: pd.DataFrame,
    threshold: float,
    epsilon: float,
) -> tuple[pd.DataFrame, pd.DataFrame, dict]:
    """
    Evaluate the linear detector at one threshold and one epsilon value.
    """
    all_event_results = []
    all_detection_results = []
    total = {"TP": 0, "FP": 0, "FN": 0}

    for key, score_df in all_scores.items():
        track, bag = key

        events_for_bag = events[
            (events["track"] == track) &
            (events["bag"] == bag)
        ].copy()

        if events_for_bag.empty:
            continue

        # --------------------------------------------------
        # Only evaluate a few seconds past the final label
        # --------------------------------------------------
        last_event_time = events_for_bag["event_time_video"].max()

        score_df = score_df[
            score_df["t_video"] <= (last_event_time + EVAL_AFTER_LAST_EVENT_SEC)
        ].copy()

        score_df = add_linear_score_for_epsilon(score_df, epsilon=epsilon)

        detections = get_detection_times_with_muting(
            score_df=score_df,
            score_col="linear_score",
            threshold=threshold,
            mute_sec=MUTE_AFTER_DETECTION_SEC,
        )

        event_df, detection_df, metrics = match_detections_to_events(
            detections=detections,
            events_for_bag=events_for_bag,
            match_window_sec=MATCH_WINDOW_SEC,
        )

        for df in [event_df, detection_df]:
            df["track"] = track
            df["bag"] = bag
            df["detector"] = "linear"
            df["threshold"] = threshold
            df["epsilon"] = epsilon

        all_event_results.append(event_df)
        all_detection_results.append(detection_df)

        total["TP"] += metrics["TP"]
        total["FP"] += metrics["FP"]
        total["FN"] += metrics["FN"]

    TP, FP, FN = total["TP"], total["FP"], total["FN"]
    precision = TP / (TP + FP) if (TP + FP) > 0 else 0.0
    recall = TP / (TP + FN) if (TP + FN) > 0 else 0.0
    f1 = 2 * precision * recall / (precision + recall) if (precision + recall) > 0 else 0.0

    summary = {
        "detector": "linear",
        "threshold": threshold,
        "epsilon": epsilon,
        "TP": TP,
        "FP": FP,
        "FN": FN,
        "precision": precision,
        "recall": recall,
        "f1": f1,
    }

    events_out = pd.concat(all_event_results, ignore_index=True) if all_event_results else pd.DataFrame()
    detections_out = pd.concat(all_detection_results, ignore_index=True) if all_detection_results else pd.DataFrame()

    return events_out, detections_out, summary



# =============================================================================
# 11. PRECISION-RECALL / F1 THRESHOLD ANALYSIS
# =============================================================================

def plot_threshold_epsilon_sweep(summary_df, detector_name, output_dir):
    """
    For event-level detection, use precision/recall/F1 instead of ROC/AUROC.
    There is no natural TN count, so ROC/FPR is not well-defined.
    """
    summary_df = summary_df.copy().dropna()

    # Best threshold/epsilon combo by F1
    best_f1 = summary_df.loc[summary_df["f1"].idxmax()]

    pr_df = summary_df.sort_values("recall")

    # For the threshold-vs-metric plot, show the slice at the best epsilon so the
    # lines stay readable instead of plotting every epsilon on top of each other.
    best_epsilon = float(best_f1["epsilon"])
    best_eps_df = summary_df[np.isclose(summary_df["epsilon"], best_epsilon)].sort_values("threshold")

    # Plot threshold vs precision/recall/F1 for the best epsilon slice
    plt.figure(figsize=(12, 8))
    plt.plot(best_eps_df["threshold"], best_eps_df["precision"], label="Precision")
    plt.plot(best_eps_df["threshold"], best_eps_df["recall"], label="Recall")
    plt.plot(best_eps_df["threshold"], best_eps_df["f1"], label="F1")

    plt.axvline(
        best_f1["threshold"],
        linestyle="--",
        label=(
            f'Best F1 threshold = {best_f1["threshold"]:.4f}, '
            f'epsilon = {best_f1["epsilon"]:.4f}'
        )
    )

    plt.xlabel("Threshold")
    plt.ylabel("Metric")
    plt.title(f"{detector_name.upper()} Detector Threshold Sweep at Best Epsilon")
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(output_dir / f"{detector_name}_threshold_sweep.png", dpi=300)
    plt.close()

    # Plot all tested threshold/epsilon combos as a 2D F1 search map.
    plt.figure(figsize=(10, 8))
    scatter = plt.scatter(
        summary_df["threshold"],
        summary_df["epsilon"],
        c=summary_df["f1"],
        s=12,
    )
    plt.scatter([best_f1["threshold"]], [best_f1["epsilon"]], marker="x", s=100)
    plt.xlabel("Threshold")
    plt.ylabel("Epsilon")
    plt.title(f"{detector_name.upper()} Detector F1 by Threshold/Epsilon")
    plt.colorbar(scatter, label="F1")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(output_dir / f"{detector_name}_threshold_epsilon_f1_map.png", dpi=300)
    plt.close()

    # Plot Precision-Recall curve
    plt.figure(figsize=(8, 8))
    plt.plot(pr_df["recall"], pr_df["precision"], marker="o", markersize=3, linewidth=0)    
    plt.xlabel("Recall")
    plt.ylabel("Precision")
    plt.title(f"{detector_name.upper()} Detector Precision-Recall Curve")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(output_dir / f"{detector_name}_precision_recall_curve.png", dpi=300)
    plt.close()

    # Save all threshold/epsilon metrics
    summary_df.to_csv(output_dir / f"{detector_name}_all_threshold_epsilon_combos.csv", index=False)

    return best_f1


# =============================================================================
# 11. MAIN SCRIPT
# =============================================================================

def make_kfold_splits(
    folders: pd.DataFrame,
    k_folds: int,
    random_seed: int,
) -> list[tuple[int, pd.DataFrame, pd.DataFrame]]:
    """
    Build deterministic K-fold splits over labeled bag folders.

    Returns a list of:
        (fold_number, train_folders, test_folders)
    """
    if len(folders) < k_folds:
        raise RuntimeError(
            f"Need at least {k_folds} labeled bags for {k_folds}-fold CV, "
            f"but only found {len(folders)}."
        )

    rng = np.random.default_rng(random_seed)
    shuffled_indices = folders.index.to_numpy().copy()
    rng.shuffle(shuffled_indices)

    test_index_groups = np.array_split(shuffled_indices, k_folds)
    splits = []

    for fold_number, test_indices in enumerate(test_index_groups, start=1):
        test_set = set(int(i) for i in test_indices)
        train_indices = [int(i) for i in shuffled_indices if int(i) not in test_set]

        train_folders = folders.loc[train_indices].sort_values(["track", "bag"]).reset_index(drop=True)
        test_folders = folders.loc[list(test_indices)].sort_values(["track", "bag"]).reset_index(drop=True)
        splits.append((fold_number, train_folders, test_folders))

    return splits


def build_threshold_epsilon_grid(
    all_scores: dict[tuple[int, int], pd.DataFrame],
) -> tuple[np.ndarray, dict[float, np.ndarray], pd.DataFrame]:
    """Build the same threshold/epsilon sweep grid, but for one fold's training bags."""
    if AUTO_SWEEP_EPSILONS:
        epsilon_values = np.linspace(
            EPSILON_MIN,
            EPSILON_MAX,
            N_AUTO_EPSILONS,
        )
    else:
        epsilon_values = np.array(EPSILONS, dtype=float)

    epsilon_values = np.array(sorted(set(float(eps) for eps in epsilon_values)))

    thresholds_by_epsilon = {}

    if AUTO_SWEEP_THRESHOLDS:
        for eps in epsilon_values:
            score_arrays = []
            for df in all_scores.values():
                vals = (
                    df["linear_abs_error"].to_numpy()
                    / (df["odom_raw_abs"].to_numpy() + float(eps))
                )
                vals = vals[np.isfinite(vals)]
                if len(vals) > 0:
                    score_arrays.append(vals)

            if not score_arrays:
                thresholds_by_epsilon[float(eps)] = np.array(LINEAR_THRESHOLDS, dtype=float)
                continue

            all_linear = np.concatenate(score_arrays)
            score_min = float(np.nanmin(all_linear))
            score_max = float(np.nanmax(all_linear))

            if np.isclose(score_min, score_max):
                thresholds_by_epsilon[float(eps)] = np.array([score_min], dtype=float)
            else:
                thresholds_by_epsilon[float(eps)] = np.linspace(
                    score_min,
                    score_max,
                    N_AUTO_THRESHOLDS,
                )
    else:
        for eps in epsilon_values:
            thresholds_by_epsilon[float(eps)] = np.array(LINEAR_THRESHOLDS, dtype=float)

    sweep_grid_rows = []
    for eps, thresholds in thresholds_by_epsilon.items():
        for th in thresholds:
            sweep_grid_rows.append({
                "epsilon": float(eps),
                "threshold": float(th),
            })

    return epsilon_values, thresholds_by_epsilon, pd.DataFrame(sweep_grid_rows)


def run_one_training_fold(
    fold_number: int,
    train_folders: pd.DataFrame,
    test_folders: pd.DataFrame,
    events: pd.DataFrame,
    all_scores_all_bags: dict[tuple[int, int], pd.DataFrame],
) -> tuple[dict, list[dict]]:
    """
    Train/sweep thresholds on this fold's training bags and return the best
    threshold/epsilon combo plus the held-out bags it should be tested on.
    """
    fold_output_dir = OUTPUT_DIR / f"fold_{fold_number}"
    fold_output_dir.mkdir(parents=True, exist_ok=True)

    train_folders.to_csv(fold_output_dir / "train_bags_used.csv", index=False)
    test_folders.to_csv(fold_output_dir / "test_bags_to_use.csv", index=False)

    train_events = select_events_for_folders(events, train_folders)
    train_events.to_csv(fold_output_dir / "training_ground_truth_events.csv", index=False)

    all_scores = select_scores_for_folders(all_scores_all_bags, train_folders)

    if not all_scores:
        raise RuntimeError(f"Fold {fold_number}: no training bag scores available.")

    epsilon_values, thresholds_by_epsilon, sweep_grid = build_threshold_epsilon_grid(all_scores)
    sweep_grid.to_csv(fold_output_dir / "linear_threshold_epsilon_sweep_grid.csv", index=False)

    print(
        f"\nFold {fold_number}/{K_FOLDS}: "
        f"training on {len(train_folders)} bags, testing on {len(test_folders)} bags."
    )
    print(
        f"  Testing {len(epsilon_values)} epsilon values and "
        f"{sum(len(v) for v in thresholds_by_epsilon.values())} total "
        "threshold/epsilon combos."
    )

    # -------------------------------------------------------------------------
    # Evaluate LINEAR detector across threshold/epsilon combos on TRAINING bags
    # -------------------------------------------------------------------------
    linear_summaries = []

    for eps in epsilon_values:
        eps = float(eps)
        print(f"  Fold {fold_number} epsilon = {eps:.6f}")

        for th in thresholds_by_epsilon[eps]:
            _, _, summary = evaluate_linear_at_threshold_epsilon(
                all_scores=all_scores,
                events=train_events,
                threshold=float(th),
                epsilon=eps,
            )
            summary["fold"] = fold_number
            linear_summaries.append(summary)

    lin_sum = pd.DataFrame(linear_summaries)
    lin_sum.to_csv(
        fold_output_dir / "linear_threshold_epsilon_summary.csv",
        index=False,
    )

    # -------------------------------------------------------------------------
    # Generate fold plots and choose best threshold/epsilon by training F1
    # -------------------------------------------------------------------------
    lin_best_f1 = plot_threshold_epsilon_sweep(lin_sum, "linear", fold_output_dir)

    best_linear_threshold = float(lin_best_f1["threshold"])
    best_linear_epsilon = float(lin_best_f1["epsilon"])

    ev, det, summary = evaluate_linear_at_threshold_epsilon(
        all_scores=all_scores,
        events=train_events,
        threshold=best_linear_threshold,
        epsilon=best_linear_epsilon,
    )

    ev.to_csv(
        fold_output_dir / "linear_best_threshold_epsilon_event_labels.csv",
        index=False,
    )

    det.to_csv(
        fold_output_dir / "linear_best_threshold_epsilon_detection_labels.csv",
        index=False,
    )

    linear_fp = det[det["detection_label"] == "FP"] if not det.empty else pd.DataFrame()
    linear_fp.to_csv(
        fold_output_dir / "linear_best_threshold_epsilon_false_positives.csv",
        index=False,
    )

    tp_latencies = ev[ev["event_label"] == "TP"]["latency_sec"] if not ev.empty else pd.Series(dtype=float)

    print(f"\nFold {fold_number} Linear latency statistics:")
    print(tp_latencies.describe())

    print(f"  Fold {fold_number} Best F1 Threshold: {best_linear_threshold:.4f}")
    print(f"  Fold {fold_number} Best F1 Epsilon:   {best_linear_epsilon:.4f}")
    print(f"    - TP: {int(summary['TP'])}")
    print(f"    - FP: {int(summary['FP'])}")
    print(f"    - FN: {int(summary['FN'])}")
    print(f"    - Precision: {summary['precision']:.3f}")
    print(f"    - Recall: {summary['recall']:.3f}")
    print(f"    - F1: {summary['f1']:.3f}")

    test_bag_names = [
        bag_name(row["track"], row["bag"])
        for _, row in test_folders.iterrows()
    ]
    train_bag_names = [
        bag_name(row["track"], row["bag"])
        for _, row in train_folders.iterrows()
    ]

    best_row = {
        "fold": fold_number,
        "detector": "linear",
        "criterion": "max_f1_on_training_bags",
        "threshold": best_linear_threshold,
        "epsilon": best_linear_epsilon,
        "TP_train": int(summary["TP"]),
        "FP_train": int(summary["FP"]),
        "FN_train": int(summary["FN"]),
        "precision_train": float(summary["precision"]),
        "recall_train": float(summary["recall"]),
        "f1_train": float(summary["f1"]),
        "train_bag_count": int(len(train_folders)),
        "test_bag_count": int(len(test_folders)),
        "train_bags": "; ".join(train_bag_names),
        "test_bags": "; ".join(test_bag_names),
    }

    threshold_test_rows = []
    for _, row in test_folders.iterrows():
        threshold_test_rows.append({
            "fold": fold_number,
            "threshold": best_linear_threshold,
            "epsilon": best_linear_epsilon,
            "test_track": int(row["track"]),
            "test_bag": int(row["bag"]),
            "test_bag_name": bag_name(row["track"], row["bag"]),
            "test_folder": str(row["folder"]),
            "all_test_bags_for_fold": "; ".join(test_bag_names),
            "train_bag_count": int(len(train_folders)),
            "test_bag_count": int(len(test_folders)),
            "TP_train": int(summary["TP"]),
            "FP_train": int(summary["FP"]),
            "FN_train": int(summary["FN"]),
            "precision_train": float(summary["precision"]),
            "recall_train": float(summary["recall"]),
            "f1_train": float(summary["f1"]),
        })

    return best_row, threshold_test_rows


def main():
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    # -------------------------------------------------------------------------
    # 11A. Load ground truth event labels and computed offsets
    # -------------------------------------------------------------------------
    print("Loading ground truth events...")
    events = load_ground_truth_events(LABEL_CSVS)
    events.to_csv(OUTPUT_DIR / "loaded_ground_truth_events.csv", index=False)
    print(f"Loaded {len(events)} ground truth events.")

    # Save 2surface offsets used for video-time alignment.
    offset_debug_df = getattr(
        load_two_surface_ground_truth_events_and_offsets,
        "offset_debug_df",
        pd.DataFrame(),
    )
    if not offset_debug_df.empty:
        offset_debug_df.to_csv(OUTPUT_DIR / "two_surface_video_bag_offsets.csv", index=False)

    # -------------------------------------------------------------------------
    # 11B. Find bag CSV folders
    # -------------------------------------------------------------------------
    print("Finding bag CSV folders...")
    folders = find_bag_csv_folders(DATA_ROOT)
    folders.to_csv(OUTPUT_DIR / "found_csv_folders.csv", index=False)
    print(f"Found {len(folders)} CSV folders.")

    # Only put bags with ground-truth labels into the K-fold split.
    label_keys = set(zip(events["track"].astype(int), events["bag"].astype(int)))
    folders["has_ground_truth"] = folders.apply(
        lambda r: bag_key(r["track"], r["bag"]) in label_keys,
        axis=1,
    )

    folders_without_ground_truth = folders[~folders["has_ground_truth"]].copy()
    folders_without_ground_truth.to_csv(
        OUTPUT_DIR / "folders_without_ground_truth.csv",
        index=False,
    )

    folders = folders[folders["has_ground_truth"]].copy().reset_index(drop=True)
    folders.to_csv(OUTPUT_DIR / "labeled_csv_folders_used_for_kfold.csv", index=False)
    print(f"Using {len(folders)} labeled CSV folders for {K_FOLDS}-fold training/testing.")

    if len(folders) == 0:
        raise RuntimeError("No labeled CSV folders found after matching folders to ground-truth labels.")

    # -------------------------------------------------------------------------
    # 11C. Compute detector scores once for every labeled bag
    # -------------------------------------------------------------------------
    print("Computing detector scores for all labeled bags...")
    all_scores_all_bags = {}
    failed = []

    for _, row in folders.iterrows():
        track, bag = row["track"], row["bag"]

        if pd.isna(track) or pd.isna(bag):
            failed.append({
                "folder": str(row["folder"]),
                "reason": "Could not infer track or bag from folder name.",
            })
            continue

        key = bag_key(track, bag)

        try:
            score_df = compute_scores_for_bag(row)
            if score_df.empty:
                failed.append({
                    "folder": str(row["folder"]),
                    "reason": "Score dataframe empty after synchronization.",
                })
                continue

            all_scores_all_bags[key] = score_df

            # Save per-bag scores for debugging.
            score_df.to_csv(
                OUTPUT_DIR / f"scores_{bag_name(key[0], key[1])}.csv",
                index=False,
            )

        except Exception as e:
            failed.append({
                "folder": str(row["folder"]),
                "reason": repr(e),
            })

    pd.DataFrame(failed).to_csv(OUTPUT_DIR / "failed_bags.csv", index=False)
    print(f"Computed scores for {len(all_scores_all_bags)} bags.")
    print(f"Failed bags: {len(failed)}")

    if not all_scores_all_bags:
        raise RuntimeError("No bag scores computed. Check CSV column mappings.")

    scored_keys = set(all_scores_all_bags.keys())
    folders = folders[
        folders.apply(lambda r: bag_key(r["track"], r["bag"]) in scored_keys, axis=1)
    ].copy().reset_index(drop=True)

    # -------------------------------------------------------------------------
    # 11D. Build and save the 5 K-fold splits
    # -------------------------------------------------------------------------
    splits = make_kfold_splits(folders, K_FOLDS, RANDOM_SEED)

    split_rows = []
    for fold_number, train_folders, test_folders in splits:
        for _, row in train_folders.iterrows():
            split_rows.append({
                "fold": fold_number,
                "split": "train",
                "track": int(row["track"]),
                "bag": int(row["bag"]),
                "bag_name": bag_name(row["track"], row["bag"]),
                "folder": str(row["folder"]),
            })
        for _, row in test_folders.iterrows():
            split_rows.append({
                "fold": fold_number,
                "split": "test",
                "track": int(row["track"]),
                "bag": int(row["bag"]),
                "bag_name": bag_name(row["track"], row["bag"]),
                "folder": str(row["folder"]),
            })

    pd.DataFrame(split_rows).to_csv(OUTPUT_DIR / "kfold_bag_split_summary.csv", index=False)

    # -------------------------------------------------------------------------
    # 11E. Run threshold/epsilon training for each fold
    # -------------------------------------------------------------------------
    print("\n" + "="*60)
    print("RUNNING 5-FOLD THRESHOLD/EPSILON TRAINING")
    print("="*60)

    best_rows = []
    threshold_and_test_rows = []

    for fold_number, train_folders, test_folders in splits:
        best_row, test_rows = run_one_training_fold(
            fold_number=fold_number,
            train_folders=train_folders,
            test_folders=test_folders,
            events=events,
            all_scores_all_bags=all_scores_all_bags,
        )
        best_rows.append(best_row)
        threshold_and_test_rows.extend(test_rows)

    best_thresholds = pd.DataFrame(best_rows)
    best_thresholds.to_csv(OUTPUT_DIR / "kfold_best_threshold_epsilon_summary.csv", index=False)

    threshold_and_test_df = pd.DataFrame(threshold_and_test_rows)
    threshold_and_test_df.to_csv(
        OUTPUT_DIR / "Threshold and corresponding bags to test on.csv",
        index=False,
    )

    print("\n" + "="*60)
    print(f"Done! Results saved to: {OUTPUT_DIR.resolve()}")
    print("="*60)
    print("\nGenerated files:")
    print("  - loaded_ground_truth_events.csv")
    print("  - two_surface_video_bag_offsets.csv")
    print("  - found_csv_folders.csv")
    print("  - labeled_csv_folders_used_for_kfold.csv")
    print("  - kfold_bag_split_summary.csv")
    print("  - kfold_best_threshold_epsilon_summary.csv")
    print("  - Threshold and corresponding bags to test on.csv")
    print("  - fold_1/ ... fold_5/ per-fold sweep summaries and best-label outputs")


if __name__ == "__main__":
    main()
