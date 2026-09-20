import os
import subprocess
import tempfile
import time
from collections import deque

import cv2
import numpy as np
from libcamera import controls
from picamera2 import MappedArray, Picamera2
from ultralytics import YOLO


def env_flag(name, default=False):
    value = os.environ.get(name)
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


MODEL_PATH = os.environ.get(
    "TRIPGUARD_DETECT_MODEL",
    "models/yolo11n_256_ncnn_model"
)
POSE_MODEL_PATH = os.environ.get(
    "TRIPGUARD_POSE_MODEL",
    "models/yolo11n-pose_ncnn_model"
)

# The IMX708 1536x864 sensor mode crops both sides. Force the full-FOV
# 2304x1296 sensor mode, then let the ISP create two synchronised streams.
SENSOR_WIDTH = 2304
SENSOR_HEIGHT = 1296
HIGH_WIDTH = 1536
HIGH_HEIGHT = 864
DISPLAY_WIDTH = 640
DISPLAY_HEIGHT = 360
CAMERA_FPS = float(os.environ.get("TRIPGUARD_CAMERA_FPS", "15"))

DETECT_IMG_SIZE = 256
POSE_IMG_SIZE = 288
IOU_THRESHOLD = 0.45
GLOBAL_MIN_CONF = float(
    os.environ.get("TRIPGUARD_GLOBAL_MIN_CONF", "0.20")
)
GLOBAL_STRONG_CONF = float(
    os.environ.get("TRIPGUARD_GLOBAL_STRONG_CONF", "0.40")
)
ROI_MIN_CONF = float(
    os.environ.get("TRIPGUARD_ROI_MIN_CONF", "0.20")
)
ROI_STRONG_CONF = float(
    os.environ.get("TRIPGUARD_ROI_STRONG_CONF", "0.36")
)
POSE_MIN_CONF = float(
    os.environ.get("TRIPGUARD_POSE_MIN_CONF", "0.20")
)
KEYPOINT_CONF = float(
    os.environ.get("TRIPGUARD_KEYPOINT_CONF", "0.20")
)

CAMERA_WARMUP_SECONDS = float(
    os.environ.get("TRIPGUARD_CAMERA_WARMUP_SECONDS", "2.0")
)
CAMERA_MAX_WARMUP_SECONDS = float(
    os.environ.get("TRIPGUARD_CAMERA_MAX_WARMUP_SECONDS", "5.0")
)
ROI_INTERVAL_SECONDS = float(
    os.environ.get("TRIPGUARD_ROI_INTERVAL_SECONDS", "1.7")
)
ROI_RETRY_SECONDS = float(
    os.environ.get("TRIPGUARD_ROI_RETRY_SECONDS", "0.40")
)
ROI_CACHE_SECONDS = float(
    os.environ.get("TRIPGUARD_ROI_CACHE_SECONDS", "2.5")
)
MIN_GLOBAL_FRAMES = int(
    os.environ.get("TRIPGUARD_MIN_GLOBAL_FRAMES", "60")
)
MIN_SCANS_PER_ROI = int(
    os.environ.get("TRIPGUARD_MIN_SCANS_PER_ROI", "3")
)
WEAK_CONFIRM_HITS = int(
    os.environ.get("TRIPGUARD_WEAK_CONFIRM_HITS", "3")
)
WEAK_CONFIRM_SECONDS = float(
    os.environ.get("TRIPGUARD_WEAK_CONFIRM_SECONDS", "0.40")
)
TRACK_TTL_SECONDS = float(
    os.environ.get("TRIPGUARD_TRACK_TTL_SECONDS", "10.0")
)
HEALTH_INTERVAL_SECONDS = float(
    os.environ.get("TRIPGUARD_HEALTH_INTERVAL_SECONDS", "5.0")
)
PERF_WINDOW = int(os.environ.get("TRIPGUARD_PERF_WINDOW", "30"))

MIN_BRIGHTNESS = float(
    os.environ.get("TRIPGUARD_MIN_BRIGHTNESS", "8")
)
MAX_BRIGHTNESS = float(
    os.environ.get("TRIPGUARD_MAX_BRIGHTNESS", "248")
)
MIN_BLUR_SCORE = float(
    os.environ.get("TRIPGUARD_MIN_BLUR_SCORE", "5")
)
MAX_INVALID_FRAME_RATIO = float(
    os.environ.get("TRIPGUARD_MAX_INVALID_FRAME_RATIO", "0.50")
)
MAX_SAFE_TEMPERATURE_C = float(
    os.environ.get("TRIPGUARD_MAX_SAFE_TEMPERATURE_C", "75")
)
EXPECTED_SCALER_CROP = (0, 0, 4608, 2592)

# Focus on textured seat/curtain edges at the rear, not the bright windows or
# the low-detail ceiling. Rectangle format is x, y, width, height in sensor
# coordinates (approximately x=.27-.63 and y=.24-.58).
AF_WINDOW = (1244, 622, 1659, 881)

HEADLESS = env_flag("TRIPGUARD_HEADLESS")
AUTO_START = env_flag("TRIPGUARD_AUTO_START")
EXIT_AFTER_SCAN = env_flag("TRIPGUARD_EXIT_AFTER_SCAN")
MAX_PROGRAM_SECONDS = float(
    os.environ.get("TRIPGUARD_MAX_SECONDS", "0")
)
SCAN_SESSION_SECONDS = float(
    os.environ.get("TRIPGUARD_SCAN_SESSION_SECONDS", "900")
)
SCAN_SESSION_ID = int(os.environ.get("TRIPGUARD_SCAN_SESSION_ID", "0"))
if not 1.0 <= SCAN_SESSION_SECONDS <= 3600.0:
    raise ValueError("TRIPGUARD_SCAN_SESSION_SECONDS must be 1..3600")
EVIDENCE_JPEG_QUALITY = int(
    os.environ.get("TRIPGUARD_EVIDENCE_JPEG_QUALITY", "55")
)
EVIDENCE_MAX_BYTES = int(
    os.environ.get("TRIPGUARD_EVIDENCE_MAX_BYTES", str(75 * 1024))
)

KEYPOINT_NAMES = (
    "HEAD", "EYE-L", "EYE-R", "EAR-L", "EAR-R",
    "SHOULDER-L", "SHOULDER-R", "ELBOW-L", "ELBOW-R",
    "WRIST-L", "WRIST-R", "HIP-L", "HIP-R", "KNEE-L",
    "KNEE-R", "ANKLE-L", "ANKLE-R"
)

SKELETON_EDGES = (
    (0, 1), (0, 2), (1, 3), (2, 4),
    (5, 6), (5, 7), (7, 9), (6, 8), (8, 10),
    (5, 11), (6, 12), (11, 12), (11, 13),
    (13, 15), (12, 14), (14, 16)
)

# Rear-cabin preset derived from the supplied 29-seat bus photograph. Three
# near-square crops enlarge partially hidden passengers without wasting the
# 256px model input on the ceiling, close seat backs, or bright side windows.
# The full 640x360 view still monitors the whole cabin between ROI frames.
# Fine-tune these rectangles once a reference frame is captured by the Pi
# Camera from its final mount.
ROI_SPECS = {
    "REAR_LEFT": (0.18, 0.20, 0.47, 0.72),
    "REAR_CENTER": (0.30, 0.20, 0.59, 0.72),
    "REAR_RIGHT": (0.43, 0.20, 0.72, 0.72),
}
ROI_SCHEDULE = (
    "REAR_LEFT",
    "REAR_CENTER",
    "REAR_RIGHT",
)


def build_roi_boxes():
    boxes = {}
    for name, (x1, y1, x2, y2) in ROI_SPECS.items():
        boxes[name] = (
            int(round(x1 * HIGH_WIDTH)),
            int(round(y1 * HIGH_HEIGHT)),
            int(round(x2 * HIGH_WIDTH)),
            int(round(y2 * HIGH_HEIGHT)),
        )
    return boxes


ROI_BOXES = build_roi_boxes()


def read_temperature_c():
    try:
        completed = subprocess.run(
            ["vcgencmd", "measure_temp"],
            check=True,
            capture_output=True,
            text=True,
            timeout=2,
        )
        value = completed.stdout.strip().split("=", 1)[-1]
        return float(value.split("'", 1)[0])
    except (OSError, ValueError, subprocess.SubprocessError):
        pass

    try:
        with open(
            "/sys/class/thermal/thermal_zone0/temp",
            "r",
            encoding="utf-8",
        ) as temperature_file:
            return float(temperature_file.read().strip()) / 1000.0
    except (OSError, ValueError):
        return None


def read_throttled_status():
    try:
        completed = subprocess.run(
            ["vcgencmd", "get_throttled"],
            check=True,
            capture_output=True,
            text=True,
            timeout=2,
        )
        return completed.stdout.strip().split("=", 1)[-1]
    except (OSError, subprocess.SubprocessError):
        return "unavailable"


def has_active_power_or_throttle_fault(status):
    try:
        # Current-condition bits: undervoltage, capped frequency, throttling,
        # and soft temperature limit. Upper bits only record past events.
        return (int(status, 16) & 0xF) != 0
    except (TypeError, ValueError):
        return True


def print_health(elapsed_seconds):
    temperature = read_temperature_c()
    temperature_text = (
        f"{temperature:.1f}C" if temperature is not None else "unavailable"
    )
    throttled = read_throttled_status()
    print(
        f"[HEALTH] elapsed={elapsed_seconds:.1f}s "
        f"temp={temperature_text} throttled={throttled}"
    )
    return temperature, throttled


def make_candidate(box, confidence, source, region, points=None):
    return {
        "box": tuple(float(value) for value in box),
        "confidence": float(confidence),
        "source_scores": {source: float(confidence)},
        "region": region,
        "points": points or {},
    }


def detect_people(frame, confidence, source, region):
    result = detector.predict(
        frame,
        imgsz=DETECT_IMG_SIZE,
        conf=confidence,
        iou=IOU_THRESHOLD,
        classes=[0],
        verbose=False,
    )[0]

    candidates = []
    if result.boxes is not None:
        for box, score in zip(result.boxes.xyxy, result.boxes.conf):
            candidates.append(
                make_candidate(
                    box.tolist(),
                    score.item(),
                    source,
                    region,
                )
            )

    speed = result.speed
    timing = {
        "preprocess_ms": float(speed.get("preprocess", 0.0)),
        "inference_ms": float(speed.get("inference", 0.0)),
        "postprocess_ms": float(speed.get("postprocess", 0.0)),
    }
    return candidates, timing


def detect_pose(frame, region):
    started = time.perf_counter()
    result = pose_detector.predict(
        frame,
        imgsz=POSE_IMG_SIZE,
        conf=POSE_MIN_CONF,
        iou=IOU_THRESHOLD,
        verbose=False,
    )[0]
    elapsed_ms = (time.perf_counter() - started) * 1000.0

    candidates = []
    if (
        result.boxes is None
        or result.keypoints is None
        or result.keypoints.conf is None
    ):
        return candidates, elapsed_ms

    for box, score, points_xy, points_conf in zip(
        result.boxes.xyxy.tolist(),
        result.boxes.conf.tolist(),
        result.keypoints.xy.tolist(),
        result.keypoints.conf.tolist(),
    ):
        visible_points = {}
        for point_id, ((x, y), point_confidence) in enumerate(
            zip(points_xy, points_conf)
        ):
            if point_confidence >= KEYPOINT_CONF:
                visible_points[point_id] = (
                    float(x),
                    float(y),
                    float(point_confidence),
                )

        candidates.append(
            make_candidate(
                box,
                score,
                "roi-pose",
                region,
                visible_points,
            )
        )

    return candidates, elapsed_ms


def box_iou(first_box, second_box):
    first_x1, first_y1, first_x2, first_y2 = first_box
    second_x1, second_y1, second_x2, second_y2 = second_box
    intersection_width = max(
        0.0, min(first_x2, second_x2) - max(first_x1, second_x1)
    )
    intersection_height = max(
        0.0, min(first_y2, second_y2) - max(first_y1, second_y1)
    )
    intersection = intersection_width * intersection_height
    first_area = max(0.0, first_x2 - first_x1) * max(
        0.0, first_y2 - first_y1
    )
    second_area = max(0.0, second_x2 - second_x1) * max(
        0.0, second_y2 - second_y1
    )
    union = first_area + second_area - intersection
    return intersection / union if union > 0.0 else 0.0


def box_center_distance(first_box, second_box):
    first_x = (first_box[0] + first_box[2]) / 2.0
    first_y = (first_box[1] + first_box[3]) / 2.0
    second_x = (second_box[0] + second_box[2]) / 2.0
    second_y = (second_box[1] + second_box[3]) / 2.0
    return ((first_x - second_x) ** 2 + (first_y - second_y) ** 2) ** 0.5


def merge_candidates(candidates, iou_threshold=0.30):
    ordered = sorted(
        candidates,
        key=lambda candidate: candidate["confidence"],
        reverse=True,
    )
    merged = []
    for candidate in ordered:
        matching = None
        for kept in merged:
            if box_iou(candidate["box"], kept["box"]) >= iou_threshold:
                matching = kept
                break

        if matching is None:
            copied = dict(candidate)
            copied["source_scores"] = dict(candidate["source_scores"])
            copied["points"] = dict(candidate["points"])
            merged.append(copied)
            continue

        for source, score in candidate["source_scores"].items():
            matching["source_scores"][source] = max(
                matching["source_scores"].get(source, 0.0),
                score,
            )
        if len(candidate["points"]) > len(matching["points"]):
            matching["points"] = dict(candidate["points"])
        if candidate["confidence"] > matching["confidence"]:
            matching["confidence"] = candidate["confidence"]
            matching["box"] = candidate["box"]
            matching["region"] = candidate["region"]

    return merged


def map_roi_candidate_to_display(candidate, roi_box, crop_shape):
    roi_x1, roi_y1, roi_x2, roi_y2 = roi_box
    crop_height, crop_width = crop_shape[:2]
    roi_width = roi_x2 - roi_x1
    roi_height = roi_y2 - roi_y1
    scale_from_crop_x = roi_width / float(crop_width)
    scale_from_crop_y = roi_height / float(crop_height)
    scale_to_display_x = DISPLAY_WIDTH / float(HIGH_WIDTH)
    scale_to_display_y = DISPLAY_HEIGHT / float(HIGH_HEIGHT)

    x1, y1, x2, y2 = candidate["box"]
    mapped_box = (
        (roi_x1 + x1 * scale_from_crop_x) * scale_to_display_x,
        (roi_y1 + y1 * scale_from_crop_y) * scale_to_display_y,
        (roi_x1 + x2 * scale_from_crop_x) * scale_to_display_x,
        (roi_y1 + y2 * scale_from_crop_y) * scale_to_display_y,
    )
    mapped_points = {}
    for point_id, (x, y, confidence) in candidate["points"].items():
        mapped_points[point_id] = (
            (roi_x1 + x * scale_from_crop_x) * scale_to_display_x,
            (roi_y1 + y * scale_from_crop_y) * scale_to_display_y,
            confidence,
        )

    mapped = dict(candidate)
    mapped["box"] = mapped_box
    mapped["points"] = mapped_points
    return mapped


def roi_box_to_display(roi_box):
    scale_x = DISPLAY_WIDTH / float(HIGH_WIDTH)
    scale_y = DISPLAY_HEIGHT / float(HIGH_HEIGHT)
    return tuple(
        int(round(value * (scale_x if index % 2 == 0 else scale_y)))
        for index, value in enumerate(roi_box)
    )


def draw_roi_guides(frame, active_roi=None):
    """Overlay the calibrated rear zones so camera alignment is visible."""
    short_names = {
        "REAR_LEFT": "REAR-L",
        "REAR_CENTER": "REAR-C",
        "REAR_RIGHT": "REAR-R",
    }
    for name, roi_box in ROI_BOXES.items():
        x1, y1, x2, y2 = roi_box_to_display(roi_box)
        is_active = name == active_roi
        color = (255, 0, 255) if is_active else (160, 160, 160)
        thickness = 2 if is_active else 1
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)
        cv2.putText(
            frame,
            short_names.get(name, name),
            (x1 + 3, min(DISPLAY_HEIGHT - 4, y1 + 15)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.36,
            color,
            1,
        )


def candidate_level(candidate):
    scores = candidate["source_scores"]
    point_count = len(candidate["points"])
    global_score = scores.get("global-detect", 0.0)
    roi_score = scores.get("roi-detect", 0.0)
    pose_score = scores.get("roi-pose", 0.0)

    if global_score >= GLOBAL_STRONG_CONF:
        return "strong", f"global person {global_score:.2f}"
    if roi_score >= ROI_STRONG_CONF:
        return "strong", f"high-res person {roi_score:.2f}"
    if (
        roi_score >= ROI_MIN_CONF
        and pose_score >= POSE_MIN_CONF
        and point_count >= 3
    ):
        return "strong", (
            f"detect+pose {max(roi_score, pose_score):.2f}/"
            f"{point_count} points"
        )

    # A chair with clothing produced a stable 16-keypoint false pose in the
    # empty-room test. Pose-only evidence therefore requests a recheck but
    # cannot latch OCCUPIED without detector support.
    if global_score >= GLOBAL_MIN_CONF or roi_score >= ROI_MIN_CONF:
        return "weak", f"weak person {max(global_score, roi_score):.2f}"
    if pose_score >= POSE_MIN_CONF and point_count >= 3:
        return "weak", (
            f"pose-only candidate {pose_score:.2f}/{point_count} points"
        )
    return "ignore", "below evidence threshold"


def update_evidence_tracks(tracks, candidates, now):
    newly_confirmed_reason = None
    updated_track_ids = set()
    for candidate in candidates:
        level, reason = candidate_level(candidate)
        candidate["level"] = level
        candidate["reason"] = reason
        if level == "ignore":
            continue
        if level == "strong":
            newly_confirmed_reason = reason

        best_track = None
        best_match_score = -1.0
        for track in tracks:
            if id(track) in updated_track_ids:
                continue
            if now - track["last_seen"] > TRACK_TTL_SECONDS:
                continue
            overlap = box_iou(candidate["box"], track["box"])
            distance = box_center_distance(candidate["box"], track["box"])
            match_score = overlap
            if overlap >= 0.10 or distance <= 45.0:
                if match_score > best_match_score:
                    best_match_score = match_score
                    best_track = track

        if best_track is None:
            best_track = {
                "box": candidate["box"],
                "first_seen": now,
                "last_seen": now,
                "hits": 1,
                "max_confidence": candidate["confidence"],
                "max_points": len(candidate["points"]),
                "sources": set(candidate["source_scores"]),
                "region": candidate["region"],
                "reason": reason,
            }
            tracks.append(best_track)
        else:
            best_track["box"] = candidate["box"]
            best_track["last_seen"] = now
            best_track["hits"] += 1
            best_track["max_confidence"] = max(
                best_track["max_confidence"], candidate["confidence"]
            )
            best_track["max_points"] = max(
                best_track["max_points"], len(candidate["points"])
            )
            best_track["sources"].update(candidate["source_scores"])
            best_track["region"] = candidate["region"]
            best_track["reason"] = reason

        updated_track_ids.add(id(best_track))

        elapsed = now - best_track["first_seen"]
        has_pose = "roi-pose" in best_track["sources"]
        has_detector = bool(
            {"global-detect", "roi-detect"} & best_track["sources"]
        )
        if (
            best_track["hits"] >= 2
            and has_pose
            and has_detector
            and best_track["max_points"] >= 3
        ):
            newly_confirmed_reason = (
                f"repeated detect+pose in {best_track['region']}"
            )
        elif (
            best_track["hits"] >= WEAK_CONFIRM_HITS
            and has_detector
            and elapsed >= WEAK_CONFIRM_SECONDS
        ):
            newly_confirmed_reason = (
                f"repeated weak person in {best_track['region']}"
            )

    tracks[:] = [
        track
        for track in tracks
        if now - track["last_seen"] <= TRACK_TTL_SECONDS
    ]
    return newly_confirmed_reason


def analyse_frame_quality(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    brightness = float(gray.mean())
    blur_score = float(cv2.Laplacian(gray, cv2.CV_64F).var())
    clipped_ratio = float(np.mean((gray <= 3) | (gray >= 252)))
    valid = (
        MIN_BRIGHTNESS <= brightness <= MAX_BRIGHTNESS
        and blur_score >= MIN_BLUR_SCORE
        and clipped_ratio < 0.95
    )
    return valid, brightness, blur_score, clipped_ratio


def exposure_is_stable(samples):
    if len(samples) < samples.maxlen:
        return False
    exposure_values = [sample[0] for sample in samples]
    gain_values = [sample[1] for sample in samples]
    if min(exposure_values) <= 0.0 or min(gain_values) <= 0.0:
        return False
    min_exposure = max(1.0, min(exposure_values))
    min_gain = max(0.01, min(gain_values))
    return (
        max(exposure_values) / min_exposure <= 1.20
        and max(gain_values) / min_gain <= 1.20
    )


def start_camera():
    camera = Picamera2()
    try:
        frame_duration = max(1, int(round(1_000_000.0 / CAMERA_FPS)))
        config = camera.create_preview_configuration(
            main={"size": (HIGH_WIDTH, HIGH_HEIGHT), "format": "RGB888"},
            lores={
                "size": (DISPLAY_WIDTH, DISPLAY_HEIGHT),
                "format": "YUV420",
            },
            raw=None,
            sensor={
                "output_size": (SENSOR_WIDTH, SENSOR_HEIGHT),
                "bit_depth": 10,
            },
            controls={
                "FrameDurationLimits": (frame_duration, frame_duration),
                "AfMode": controls.AfModeEnum.Continuous,
                "AfMetering": controls.AfMeteringEnum.Windows,
                "AfWindows": [AF_WINDOW],
            },
            buffer_count=4,
            queue=True,
            display=None,
            encode=None,
        )
        camera.configure(config)
        applied = camera.camera_configuration()
        applied_sensor = applied.get("sensor") or {}
        if tuple(applied_sensor.get("output_size", ())) != (
            SENSOR_WIDTH,
            SENSOR_HEIGHT,
        ):
            raise RuntimeError(
                f"wrong sensor mode applied: {applied_sensor}"
            )
        print(
            f"[CAMERA] main={applied['main']['size']} "
            f"lores={applied['lores']['size']} "
            f"sensor={applied.get('sensor')}"
        )
        camera.start()
        return camera
    except Exception:
        camera.close()
        raise


def stop_camera(camera):
    if camera is None:
        return
    try:
        camera.stop()
    finally:
        camera.close()


def capture_camera_frame(camera, roi_name=None):
    request = camera.capture_request()
    try:
        metadata = request.get_metadata()
        with MappedArray(request, "lores") as mapped:
            yuv_height = DISPLAY_HEIGHT * 3 // 2
            yuv = mapped.array[:yuv_height, :DISPLAY_WIDTH]
            display_frame = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420)

        roi_frame = None
        if roi_name is not None:
            roi_x1, roi_y1, roi_x2, roi_y2 = ROI_BOXES[roi_name]
            with MappedArray(request, "main") as mapped:
                roi_frame = np.ascontiguousarray(
                    mapped.array[roi_y1:roi_y2, roi_x1:roi_x2]
                )
    finally:
        request.release()
    return display_frame, roi_frame, metadata


def draw_candidate(frame, candidate):
    x1, y1, x2, y2 = (
        int(round(value)) for value in candidate["box"]
    )
    level, reason = candidate_level(candidate)
    if level == "strong":
        color = (0, 0, 255)
    elif level == "weak":
        color = (0, 215, 255)
    else:
        color = (120, 120, 120)
    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
    cv2.putText(
        frame,
        f"{candidate['region']} {reason}",
        (x1, max(18, y1 - 6)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.42,
        color,
        1,
    )

    points = candidate["points"]
    for first_id, second_id in SKELETON_EDGES:
        if first_id in points and second_id in points:
            first = tuple(int(round(v)) for v in points[first_id][:2])
            second = tuple(int(round(v)) for v in points[second_id][:2])
            cv2.line(frame, first, second, (255, 180, 0), 1)
    for point_id, (x, y, confidence) in points.items():
        center = (int(round(x)), int(round(y)))
        cv2.circle(frame, center, 3, (0, 200, 255), -1)
        if point_id == 0:
            cv2.putText(
                frame,
                f"{KEYPOINT_NAMES[point_id]} {confidence:.2f}",
                (center[0] + 4, max(12, center[1] - 4)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.32,
                (0, 200, 255),
                1,
            )


def draw_status_panel(
    frame,
    phase,
    phase_time,
    result,
    occupied_latched,
    active_roi,
    roi_counts,
    valid_frames,
    invalid_frames,
    recent_fps,
    quality,
):
    overlay = frame.copy()
    cv2.rectangle(overlay, (0, 0), (DISPLAY_WIDTH, 132), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.62, frame, 0.38, 0, frame)

    if phase == "WARMUP":
        state_text = f"WARMUP {phase_time:.1f}s"
        state_color = (0, 215, 255)
    elif phase == "SCANNING":
        state_text = f"MONITORING {phase_time:.1f}s"
        state_color = (0, 255, 0)
    else:
        state_text = phase
        state_color = (255, 255, 255)

    cv2.putText(
        frame,
        f"PHASE: {state_text}",
        (12, 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.58,
        state_color,
        2,
    )
    evidence_text = (
        "OCCUPIED LATCHED" if occupied_latched else "NO CONFIRMED PERSON"
    )
    evidence_color = (0, 0, 255) if occupied_latched else (255, 255, 255)
    cv2.putText(
        frame,
        f"EVIDENCE: {evidence_text}",
        (12, 49),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.52,
        evidence_color,
        2,
    )
    if result:
        result_colors = {
            "VERIFYING": (255, 255, 255),
            "CAMERA_PASS_ONLY": (0, 215, 255),
            "RECHECK_REQUIRED": (0, 165, 255),
            "OCCUPIED": (0, 0, 255),
        }
        cv2.putText(
            frame,
            f"RESULT: {result}",
            (12, 74),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.56,
            result_colors.get(result, (0, 0, 255)),
            2,
        )
    else:
        cv2.putText(
            frame,
            f"ROI: {active_roi}",
            (12, 74),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            (255, 0, 255),
            1,
        )

    brightness, blur_score = quality
    cv2.putText(
        frame,
        (
            f"VALID {valid_frames} BAD {invalid_frames}  "
            f"FPS {recent_fps:.1f}  LUMA {brightness:.0f} BLUR {blur_score:.0f}"
        ),
        (12, 98),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.42,
        (255, 255, 255),
        1,
    )
    coverage = (
        f"ACTIVE {active_roi}  "
        f"L:{roi_counts.get('REAR_LEFT', 0)} "
        f"C:{roi_counts.get('REAR_CENTER', 0)} "
        f"R:{roi_counts.get('REAR_RIGHT', 0)}"
    )
    cv2.putText(
        frame,
        coverage,
        (12, 120),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.34,
        (255, 255, 255),
        1,
    )
    cv2.putText(
        frame,
        "RADAR: NOT CONNECTED",
        (DISPLAY_WIDTH - 196, DISPLAY_HEIGHT - 14),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.40,
        (0, 215, 255),
        1,
    )


def make_waiting_frame(result=None, detail=None):
    frame = np.zeros((DISPLAY_HEIGHT, DISPLAY_WIDTH, 3), dtype=np.uint8)
    cv2.putText(
        frame,
        "TRIPGUARD AI - END-TRIP SCANNER",
        (65, 62),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.70,
        (255, 255, 255),
        2,
    )
    if result is None:
        cv2.putText(
            frame,
            "WAITING FOR DRIVER END-TRIP BUTTON",
            (84, 137),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (0, 215, 255),
            2,
        )
    else:
        color = (
            (0, 215, 255) if result == "CAMERA_PASS_ONLY" else (0, 0, 255)
        )
        cv2.putText(
            frame,
            f"FINAL RESULT: {result}",
            (100, 130),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.68,
            color,
            2,
        )
        if detail:
            cv2.putText(
                frame,
                detail[:70],
                (28, 170),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                (255, 255, 255),
                1,
            )
    cv2.putText(
        frame,
        "S = simulate END_TRIP / scan again",
        (126, 252),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.52,
        (255, 255, 255),
        1,
    )
    cv2.putText(
        frame,
        "Q = quit",
        (276, 286),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.52,
        (255, 255, 255),
        1,
    )
    cv2.putText(
        frame,
        "Camera and inference are OFF while waiting",
        (141, 331),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.42,
        (130, 130, 130),
        1,
    )
    return frame
UART_BRIDGE_PYTHON = "/usr/bin/python3"
UART_BRIDGE_SCRIPT = "/opt/tripguard/tripguard_uart.py"


def save_evidence_frame(frame, candidates, confidence):
    """Create one bounded JPEG for a newly confirmed person event."""
    evidence_frame = frame.copy()
    for candidate in candidates:
        draw_candidate(evidence_frame, candidate)
    cv2.putText(
        evidence_frame,
        f"PERSON {confidence:.2f} {time.strftime('%Y-%m-%d %H:%M:%S')}",
        (12, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.58,
        (0, 0, 255),
        2,
        cv2.LINE_AA,
    )

    encoded_bytes = None
    start_quality = max(30, min(85, EVIDENCE_JPEG_QUALITY))
    for quality in range(start_quality, 24, -5):
        encoded_ok, encoded = cv2.imencode(
            ".jpg",
            evidence_frame,
            [cv2.IMWRITE_JPEG_QUALITY, quality],
        )
        if not encoded_ok:
            continue
        candidate_bytes = encoded.tobytes()
        if len(candidate_bytes) <= EVIDENCE_MAX_BYTES:
            encoded_bytes = candidate_bytes
            break
    if encoded_bytes is None:
        raise RuntimeError(
            f"cannot compress evidence below {EVIDENCE_MAX_BYTES} bytes"
        )

    descriptor, image_path = tempfile.mkstemp(
        prefix="tripguard-person-",
        suffix=".jpg",
    )
    try:
        with os.fdopen(descriptor, "wb") as image_file:
            image_file.write(encoded_bytes)
    except BaseException:
        try:
            os.unlink(image_path)
        except FileNotFoundError:
            pass
        raise
    return image_path, len(encoded_bytes)


def notify_stm32(action, confidence=1.0, image_path=None):
    """Gui READY, PERSON hoac CLEAR qua tripguard-uart.service."""
    command = [
        UART_BRIDGE_PYTHON,
        UART_BRIDGE_SCRIPT,
        action,
    ]

    if action == "person":
        confidence = max(0.0, min(1.0, float(confidence)))
        command.extend([
            "--confidence",
            f"{confidence:.3f}",
        ])
        if image_path:
            command.extend(["--image", image_path])

    try:
        completed = subprocess.run(
            command,
            capture_output=True,
            text=True,
            timeout=3.0,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        print(f"[UART ERROR] {action}: {error}")
        return False

    response = (completed.stdout or completed.stderr).strip()
    if completed.returncode == 0 and "OK queued" in response:
        print(f"[UART OK] STM32 <- {action.upper()} {response}")
        return True

    print(
        f"[UART ERROR] action={action} "
        f"returncode={completed.returncode} response={response}"
    )
    return False


def notify_scan_complete(scan_result):
    """Persist and report the terminal result for the commanded scan session."""
    command = [
        UART_BRIDGE_PYTHON,
        UART_BRIDGE_SCRIPT,
        "scan-complete",
        "--session-id",
        str(SCAN_SESSION_ID),
        "--result",
        scan_result,
    ]
    try:
        completed = subprocess.run(
            command,
            capture_output=True,
            text=True,
            timeout=3.0,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        print(f"[UART ERROR] scan-complete: {error}")
        return False
    response = (completed.stdout or completed.stderr).strip()
    if completed.returncode == 0 and "OK queued" in response:
        print(
            f"[UART OK] SCAN_COMPLETE session={SCAN_SESSION_ID} "
            f"result={scan_result}"
        )
        return True
    print(
        f"[UART ERROR] scan-complete returncode={completed.returncode} "
        f"response={response}"
    )
    return False

def print_performance(samples, label="PERF"):
    if not samples:
        return 0.0

    def average(key):
        return sum(sample[key] for sample in samples) / len(samples)

    total_ms = average("total_ms")
    fps = 1000.0 / total_ms if total_ms > 0.0 else 0.0
    print(
        f"[{label}] frames={len(samples)} "
        f"capture={average('capture_ms'):.1f}ms "
        f"global={average('global_ms'):.1f}ms "
        f"roi-detect/frame={average('roi_detect_ms'):.1f}ms "
        f"roi-pose/frame={average('roi_pose_ms'):.1f}ms "
        f"display={average('display_ms'):.1f}ms "
        f"total={total_ms:.1f}ms fps={fps:.2f}"
    )
    return fps


print("=" * 56)
print(" TRIPGUARD AI - BUTTON-TRIGGERED HIGH-RES CABIN SCAN")
print("=" * 56)
if HEADLESS and not AUTO_START:
    raise RuntimeError(
        "headless mode needs TRIPGUARD_AUTO_START=1 until UART/GPIO is added"
    )
print("[1] Loading YOLO11n detect and YOLO11n pose...")
model_load_started = time.perf_counter()
detector = YOLO(MODEL_PATH, task="detect")
pose_detector = YOLO(POSE_MODEL_PATH, task="pose")
print(
    f"[OK] Model descriptors loaded in "
    f"{(time.perf_counter() - model_load_started) * 1000:.1f}ms"
)
print(f"[MODEL] global/ROI={MODEL_PATH} at {DETECT_IMG_SIZE}px")
print(f"[MODEL] partial-body pose={POSE_MODEL_PATH} at {POSE_IMG_SIZE}px")

print("[2] Warming model runtimes with synthetic images...")
warmup_started = time.perf_counter()
detector.predict(
    np.zeros((DETECT_IMG_SIZE, DETECT_IMG_SIZE, 3), dtype=np.uint8),
    imgsz=DETECT_IMG_SIZE,
    conf=0.99,
    classes=[0],
    verbose=False,
)
pose_detector.predict(
    np.zeros((POSE_IMG_SIZE, POSE_IMG_SIZE, 3), dtype=np.uint8),
    imgsz=POSE_IMG_SIZE,
    conf=0.99,
    verbose=False,
)
print(
    f"[OK] Model runtimes ready in "
    f"{(time.perf_counter() - warmup_started) * 1000:.1f}ms"
)
notify_stm32("ready")
print("[READY] Press S in the Pi window to simulate END_TRIP; Q quits.")
uart_person_state = None
no_person_since = None
program_started = time.monotonic()
camera = None
phase = "WAITING"
result = None
result_detail = None
auto_start_pending = AUTO_START
exit_requested = False
maximum_temperature = read_temperature_c()
last_display_frame = make_waiting_frame()

scan_started_at = None
session_started_at = None
session_deadline = None
warmup_min_ends_at = None
warmup_deadline = None
focus_good_frames = 0
focus_locked = False
exposure_samples = deque(maxlen=5)
last_af_state = None
last_lens_position = None
next_roi_at = None
next_health_at = None
roi_schedule_index = 0
active_roi = "WAITING"
roi_counts = {name: 0 for name in ROI_SPECS}
roi_invalid_counts = {name: 0 for name in ROI_SPECS}
roi_cache = []
evidence_tracks = []
uncertain_latched = False
uncertain_reasons = set()
occupied_latched = False
occupied_reason = None
valid_global_frames = 0
invalid_global_frames = 0
last_sensor_timestamp = None
last_quality = (0.0, 0.0)
performance_samples = deque(maxlen=PERF_WINDOW)
last_monitor_status = None
scan_completion_sent = False


def reset_scan_state(now):
    global scan_started_at, session_started_at, session_deadline
    global warmup_min_ends_at, warmup_deadline
    global focus_good_frames, focus_locked, exposure_samples, last_af_state
    global last_lens_position, next_roi_at, next_health_at
    global roi_schedule_index, active_roi, roi_counts, roi_invalid_counts
    global roi_cache, evidence_tracks, uncertain_latched, uncertain_reasons
    global occupied_latched, occupied_reason
    global valid_global_frames, invalid_global_frames
    global last_sensor_timestamp, last_quality, performance_samples
    global last_monitor_status, scan_completion_sent
    scan_started_at = None
    session_started_at = now
    session_deadline = now + SCAN_SESSION_SECONDS
    warmup_min_ends_at = now + CAMERA_WARMUP_SECONDS
    warmup_deadline = now + CAMERA_MAX_WARMUP_SECONDS
    focus_good_frames = 0
    focus_locked = False
    exposure_samples = deque(maxlen=5)
    last_af_state = None
    last_lens_position = None
    next_roi_at = None
    next_health_at = now + HEALTH_INTERVAL_SECONDS
    roi_schedule_index = 0
    active_roi = "WAITING"
    roi_counts = {name: 0 for name in ROI_SPECS}
    roi_invalid_counts = {name: 0 for name in ROI_SPECS}
    roi_cache = []
    evidence_tracks = []
    uncertain_latched = False
    uncertain_reasons = set()
    occupied_latched = False
    occupied_reason = None
    valid_global_frames = 0
    invalid_global_frames = 0
    last_sensor_timestamp = None
    last_quality = (0.0, 0.0)
    performance_samples = deque(maxlen=PERF_WINDOW)
    last_monitor_status = None
    scan_completion_sent = False


def minimum_coverage_ready():
    """Return True after every view has enough valid camera observations."""
    total_frames = valid_global_frames + invalid_global_frames
    invalid_ratio = (
        invalid_global_frames / total_frames if total_frames else 1.0
    )
    return (
        valid_global_frames >= MIN_GLOBAL_FRAMES
        and invalid_ratio <= MAX_INVALID_FRAME_RATIO
        and all(
            count >= MIN_SCANS_PER_ROI
            for count in roi_counts.values()
        )
    )


def current_monitor_status():
    """Live safety status; monitoring continues for every returned value."""
    if occupied_latched:
        return "OCCUPIED"
    if uncertain_latched:
        return "RECHECK_REQUIRED"
    if minimum_coverage_ready():
        return "CAMERA_PASS_ONLY"
    return "VERIFYING"


def begin_scan():
    global camera, phase, result, result_detail
    now = time.monotonic()
    result = None
    result_detail = None
    reset_scan_state(now)
    print(
        f"[TRIGGER] SCAN_START session={SCAN_SESSION_ID} "
        f"duration={SCAN_SESSION_SECONDS:.0f}s"
    )
    try:
        camera = start_camera()
    except Exception as error:
        camera = None
        finish_scan("FAULT", f"camera start failed: {error}")
        return
    phase = "WARMUP"
    print(f"[STATE] WARMUP for {CAMERA_WARMUP_SECONDS:.1f}s")


def finish_scan(forced_result=None, detail=None):
    global camera, phase, result, result_detail, scan_completion_sent
    now = time.monotonic()
    invalid_total = valid_global_frames + invalid_global_frames
    invalid_ratio = (
        invalid_global_frames / invalid_total if invalid_total else 1.0
    )
    missing_regions = [
        name
        for name, count in roi_counts.items()
        if count < MIN_SCANS_PER_ROI
    ]
    final_temperature = read_temperature_c()
    final_throttled = read_throttled_status()
    if forced_result is not None:
        result = forced_result
        result_detail = detail or "forced result"
    elif occupied_latched:
        result = "OCCUPIED"
        result_detail = occupied_reason or "confirmed camera evidence"
    elif has_active_power_or_throttle_fault(final_throttled):
        result = "FAULT"
        result_detail = f"active power/throttle fault: {final_throttled}"
    elif (
        final_temperature is None
        or final_temperature > MAX_SAFE_TEMPERATURE_C
    ):
        result = "FAULT"
        result_detail = f"unsafe/unavailable temperature: {final_temperature}"
    elif valid_global_frames < MIN_GLOBAL_FRAMES:
        result = "FAULT"
        result_detail = (
            f"only {valid_global_frames}/{MIN_GLOBAL_FRAMES} valid global frames"
        )
    elif invalid_ratio > MAX_INVALID_FRAME_RATIO:
        result = "FAULT"
        result_detail = f"invalid image ratio {invalid_ratio:.0%}"
    elif missing_regions:
        result = "FAULT"
        result_detail = "ROI coverage missing: " + ",".join(missing_regions)
    elif uncertain_latched:
        result = "RECHECK_REQUIRED"
        result_detail = "weak evidence latched: " + "; ".join(
            sorted(uncertain_reasons)[:3]
        )
    else:
        result = "CAMERA_PASS_ONLY"
        result_detail = "minimum camera coverage passed; radar not connected"

    print_performance(performance_samples, label="FINAL PERF")
    print(
        f"[FINAL] result={result} detail={result_detail} "
        f"valid={valid_global_frames} invalid={invalid_global_frames} "
        f"roi_valid={roi_counts} roi_invalid={roi_invalid_counts}"
    )
    print_health(time.monotonic() - program_started)
    stop_camera(camera)
    camera = None
    phase = "RESULT"
    if not scan_completion_sent:
        if result == "OCCUPIED":
            completion = "occupied"
        elif result == "CAMERA_PASS_ONLY":
            completion = "clear"
            notify_stm32("clear")
        else:
            completion = "fault"
        scan_completion_sent = notify_scan_complete(completion)


try:
    while not exit_requested:
        now = time.monotonic()
        key = 255

        if phase in {"WAITING", "RESULT"}:
            last_display_frame = make_waiting_frame(
                result if phase == "RESULT" else None,
                result_detail,
            )
            if not HEADLESS:
                cv2.imshow("TripGuard AI - End-Trip Scan", last_display_frame)
                # Five UI refreshes per second are enough for a physical
                # button prompt and avoid burning a CPU core while idle.
                key = cv2.waitKey(200) & 0xFF
            else:
                time.sleep(0.02)

            if key == ord("q"):
                break
            if auto_start_pending or key in {ord("s"), ord("r")}:
                auto_start_pending = False
                begin_scan()
                continue
            if phase == "RESULT" and EXIT_AFTER_SCAN:
                break

        else:
            loop_started = time.perf_counter()
            roi_due = phase == "SCANNING" and now >= next_roi_at
            roi_name = None
            if roi_due:
                roi_name = ROI_SCHEDULE[roi_schedule_index]

            capture_started = time.perf_counter()
            try:
                frame, roi_frame, metadata = capture_camera_frame(
                    camera,
                    roi_name,
                )
            except Exception as error:
                finish_scan("FAULT", f"camera capture failed: {error}")
                continue
            capture_ms = (time.perf_counter() - capture_started) * 1000.0

            sensor_timestamp = metadata.get("SensorTimestamp")
            if (
                sensor_timestamp is None
                or (
                    last_sensor_timestamp is not None
                    and sensor_timestamp <= last_sensor_timestamp
                )
            ):
                finish_scan("FAULT", "camera timestamp did not advance")
                continue
            last_sensor_timestamp = sensor_timestamp

            quality_valid, brightness, blur_score, clipped_ratio = (
                analyse_frame_quality(frame)
            )
            last_quality = (brightness, blur_score)
            scaler_crop = tuple(metadata.get("ScalerCrop", ()))
            if scaler_crop != EXPECTED_SCALER_CROP:
                finish_scan(
                    "FAULT",
                    f"full-FOV sensor crop lost: {scaler_crop}",
                )
                continue

            last_af_state = metadata.get("AfState")
            last_lens_position = metadata.get("LensPosition")
            focus_valid = (
                last_af_state == controls.AfStateEnum.Focused
                or (phase == "SCANNING" and focus_locked)
            )
            exposure_samples.append((
                float(metadata.get("ExposureTime", 0.0)),
                float(metadata.get("AnalogueGain", 0.0)),
            ))

            if phase == "WARMUP":
                if quality_valid and focus_valid:
                    focus_good_frames += 1
                else:
                    focus_good_frames = 0
                exposure_ready = exposure_is_stable(exposure_samples)
                remaining = max(0.0, warmup_deadline - now)
                draw_roi_guides(frame)
                draw_status_panel(
                    frame,
                    phase,
                    remaining,
                    None,
                    False,
                    (
                        f"AF={last_af_state} LENS={last_lens_position} "
                        f"AE={'OK' if exposure_ready else 'WAIT'}"
                    ),
                    roi_counts,
                    0,
                    0,
                    0.0,
                    last_quality,
                )
                last_display_frame = frame
                if (
                    now >= warmup_min_ends_at
                    and focus_good_frames >= 3
                    and exposure_ready
                ):
                    camera.set_controls({
                        "AfMode": controls.AfModeEnum.Manual,
                        "LensPosition": float(last_lens_position),
                    })
                    focus_locked = True
                    phase = "SCANNING"
                    scan_started_at = time.monotonic()
                    next_roi_at = scan_started_at
                    print(
                        "[STATE] focus/exposure ready; continuous MONITORING "
                        f"lens={last_lens_position}"
                    )
                elif now >= warmup_deadline:
                    try:
                        fallback_lens_position = float(last_lens_position)
                        fallback_focus_valid = (
                            np.isfinite(fallback_lens_position)
                            and fallback_lens_position >= 0.0
                        )
                    except (TypeError, ValueError):
                        fallback_lens_position = None
                        fallback_focus_valid = False

                    # Camera Module 3 can report AfState=Failed in a low-detail
                    # scene even though the captured frame is demonstrably
                    # usable.  In that case, keep the last physical lens
                    # position and rely on the existing brightness/blur and AE
                    # gates.  A genuinely bad frame still remains a hard fault.
                    if (
                        quality_valid
                        and exposure_ready
                        and fallback_focus_valid
                    ):
                        camera.set_controls({
                            "AfMode": controls.AfModeEnum.Manual,
                            "LensPosition": fallback_lens_position,
                        })
                        focus_locked = True
                        phase = "SCANNING"
                        scan_started_at = time.monotonic()
                        next_roi_at = scan_started_at
                        print(
                            "[WARN] autofocus did not lock; using verified "
                            "frame-quality fallback "
                            f"AF={last_af_state} lens={fallback_lens_position} "
                            f"brightness={brightness:.1f} blur={blur_score:.1f}"
                        )
                    else:
                        finish_scan(
                            "FAULT",
                            (
                                "camera did not stabilise: "
                                f"AF={last_af_state} "
                                f"lens={last_lens_position} "
                                f"quality={quality_valid} "
                                f"AE={exposure_ready} "
                                f"brightness={brightness:.1f} "
                                f"blur={blur_score:.1f}"
                            ),
                        )
                        continue
            else:
                if quality_valid and focus_valid:
                    valid_global_frames += 1
                else:
                    invalid_global_frames += 1

                # On a scheduled high-resolution ROI frame, the same detector
                # is used on the crop instead of redundantly running it on the
                # full frame too. The next global observation is about 0.1s
                # later, while this removes a complete NCNN call.
                global_candidates = []
                global_ms = 0.0
                if not roi_due:
                    global_started = time.perf_counter()
                    global_candidates, _ = detect_people(
                        frame,
                        GLOBAL_MIN_CONF,
                        "global-detect",
                        "FULL_WIDE",
                    )
                    global_ms = (
                        time.perf_counter() - global_started
                    ) * 1000.0

                roi_detect_ms = 0.0
                roi_pose_ms = 0.0
                current_candidates = list(global_candidates)
                if roi_due and roi_frame is not None:
                    roi_quality_valid, _, _, _ = analyse_frame_quality(
                        roi_frame
                    )
                    roi_scan_valid = roi_quality_valid and focus_valid
                    mapped_roi = []
                    if roi_scan_valid:
                        roi_detect_started = time.perf_counter()
                        roi_detected, _ = detect_people(
                            roi_frame,
                            ROI_MIN_CONF,
                            "roi-detect",
                            roi_name,
                        )
                        roi_detect_ms = (
                            time.perf_counter() - roi_detect_started
                        ) * 1000.0
                        roi_pose, roi_pose_ms = detect_pose(
                            roi_frame,
                            roi_name,
                        )
                        mapped_roi = [
                            map_roi_candidate_to_display(
                                candidate,
                                ROI_BOXES[roi_name],
                                roi_frame.shape,
                            )
                            for candidate in roi_detected + roi_pose
                        ]
                        mapped_roi = merge_candidates(mapped_roi)
                        current_candidates.extend(mapped_roi)
                        roi_cache = [
                            {"candidate": candidate, "updated_at": now}
                            for candidate in mapped_roi
                        ]
                        roi_counts[roi_name] += 1
                    else:
                        roi_invalid_counts[roi_name] += 1
                    active_roi = roi_name
                    if roi_scan_valid:
                        roi_schedule_index = (
                            roi_schedule_index + 1
                        ) % len(ROI_SCHEDULE)
                        while next_roi_at <= now:
                            next_roi_at += ROI_INTERVAL_SECONDS
                    else:
                        next_roi_at = now + ROI_RETRY_SECONDS

                current_candidates = merge_candidates(current_candidates)
                for candidate in current_candidates:
                    level, reason = candidate_level(candidate)
                    if level == "weak":
                        uncertain_latched = True
                        uncertain_reasons.add(
                            f"{candidate['region']} {reason}"
                        )
                confirmed_reason = update_evidence_tracks(
                    evidence_tracks,
                    current_candidates,
                    now,
                )
                newly_occupied = (
                    confirmed_reason is not None and not occupied_latched
                )
                if newly_occupied:
                    occupied_latched = True
                    occupied_reason = confirmed_reason
                    print(
                        f"[OCCUPIED] latched at "
                        f"{now - scan_started_at:.1f}s: {occupied_reason}"
                    )
                # Cap nhat trang thai nguoi gui sang STM32.
                if not quality_valid or not focus_valid:
                    no_person_since = None

                elif current_candidates:
                    no_person_since = None

                    # OCCUPIED is a safety latch for the current end-trip
                    # scan.  Only its first edge creates evidence and notifies
                    # STM32; intermittent detector misses must not manufacture
                    # a new event for the same occupant.
                    if newly_occupied:
                        if uart_person_state != 1:
                            event_confidence = max(
                                candidate["confidence"]
                                for candidate in current_candidates
                            )
                            evidence_path = None
                            try:
                                evidence_path, evidence_size = (
                                    save_evidence_frame(
                                        frame,
                                        current_candidates,
                                        event_confidence,
                                    )
                                )
                                print(
                                    f"[EVIDENCE] JPEG ready "
                                    f"bytes={evidence_size}"
                                )
                            except Exception as error:
                                print(f"[EVIDENCE ERROR] {error}")

                            try:
                                if notify_stm32(
                                    "person",
                                    confidence=event_confidence,
                                    image_path=evidence_path,
                                ):
                                    uart_person_state = 1
                                    print("[LIVE] PERSON = 1")
                            finally:
                                if evidence_path:
                                    try:
                                        os.unlink(evidence_path)
                                    except FileNotFoundError:
                                        pass

                elif not roi_due and not occupied_latched:
                    # Chi dung khung hinh toan canh de xac dinh vang nguoi.
                    if no_person_since is None:
                        no_person_since = time.monotonic()

                    absent_seconds = (
                        time.monotonic() - no_person_since
                    )

                    if absent_seconds >= 3.0:
                        if uart_person_state != 0:
                            if notify_stm32("clear"):
                                uart_person_state = 0
                                print("[LIVE] PERSON = 0")

                roi_cache = [
                    item
                    for item in roi_cache
                    if now - item["updated_at"] <= ROI_CACHE_SECONDS
                ]
                for candidate in merge_candidates(
                    global_candidates
                    + [item["candidate"] for item in roi_cache]
                ):
                    draw_candidate(frame, candidate)

                draw_roi_guides(frame, active_roi)

                scan_elapsed = now - scan_started_at
                recent_fps = 0.0
                if performance_samples:
                    average_total = sum(
                        sample["total_ms"] for sample in performance_samples
                    ) / len(performance_samples)
                    recent_fps = (
                        1000.0 / average_total if average_total > 0.0 else 0.0
                    )
                monitor_status = current_monitor_status()
                if monitor_status != last_monitor_status:
                    print(
                        f"[STATUS] {monitor_status} at {scan_elapsed:.1f}s; "
                        "monitoring continues"
                    )
                    last_monitor_status = monitor_status
                draw_status_panel(
                    frame,
                    phase,
                    scan_elapsed,
                    monitor_status,
                    occupied_latched,
                    active_roi,
                    roi_counts,
                    valid_global_frames,
                    invalid_global_frames,
                    recent_fps,
                    last_quality,
                )
                last_display_frame = frame

                display_started = time.perf_counter()
                if not HEADLESS and phase != "RESULT":
                    cv2.imshow("TripGuard AI - End-Trip Scan", frame)
                    key = cv2.waitKey(1) & 0xFF
                display_ms = (
                    time.perf_counter() - display_started
                ) * 1000.0
                total_ms = (time.perf_counter() - loop_started) * 1000.0
                performance_samples.append({
                    "capture_ms": capture_ms,
                    "global_ms": global_ms,
                    "roi_detect_ms": roi_detect_ms,
                    "roi_pose_ms": roi_pose_ms,
                    "display_ms": display_ms,
                    "total_ms": total_ms,
                })
                if (
                    len(performance_samples) == PERF_WINDOW
                    and valid_global_frames % PERF_WINDOW == 0
                ):
                    print_performance(performance_samples)
                    print(
                        f"[STATE] elapsed={scan_elapsed:.1f}s "
                        f"valid={valid_global_frames} "
                        f"invalid={invalid_global_frames} "
                        f"occupied={occupied_latched} roi={roi_counts}"
                    )

            if not HEADLESS and phase == "WARMUP":
                cv2.imshow("TripGuard AI - End-Trip Scan", last_display_frame)
                key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                if phase in {"WARMUP", "SCANNING"}:
                    finish_scan("FAULT", "operator aborted active scan")
                exit_requested = True

            now = time.monotonic()
            if (
                phase in {"WARMUP", "SCANNING"}
                and session_deadline is not None
                and now >= session_deadline
            ):
                print(
                    f"[INFO] Scan session {SCAN_SESSION_ID} reached "
                    f"{SCAN_SESSION_SECONDS:.0f}s"
                )
                finish_scan()
                continue
            if next_health_at is not None and now >= next_health_at:
                temperature, throttled = print_health(
                    now - program_started
                )
                if temperature is not None:
                    maximum_temperature = max(
                        maximum_temperature or temperature,
                        temperature,
                    )
                if phase in {"WARMUP", "SCANNING"} and (
                    has_active_power_or_throttle_fault(throttled)
                    or temperature is None
                    or temperature > MAX_SAFE_TEMPERATURE_C
                ):
                    finish_scan(
                        "FAULT",
                        (
                            f"health fault: temp={temperature} "
                            f"throttled={throttled}"
                        ),
                    )
                    continue
                while next_health_at <= now:
                    next_health_at += HEALTH_INTERVAL_SECONDS

        if (
            MAX_PROGRAM_SECONDS > 0.0
            and time.monotonic() - program_started >= MAX_PROGRAM_SECONDS
        ):
            print("[INFO] Maximum program duration reached")
            if phase in {"WARMUP", "SCANNING"}:
                finish_scan("FAULT", "maximum program duration interrupted scan")
            break

except KeyboardInterrupt:
    print("[INFO] Interrupted by user")
    if phase in {"WARMUP", "SCANNING"}:
        finish_scan("FAULT", "keyboard interrupt during active scan")
except Exception as error:
    result = "FAULT"
    result_detail = f"unhandled error: {error}"
    print(f"[FAULT] {result_detail}")
    raise
finally:
    stop_camera(camera)
    cv2.destroyAllWindows()
    final_temperature, final_throttled = print_health(
        time.monotonic() - program_started
    )
    if final_temperature is not None:
        maximum_temperature = max(
            maximum_temperature or final_temperature,
            final_temperature,
        )
    if maximum_temperature is not None:
        print(f"[FINAL HEALTH] max_temp={maximum_temperature:.1f}C")
    print(f"[FINAL POWER] throttled={final_throttled}")
    print("[INFO] TripGuard AI stopped")
