import os
import argparse
import yaml
import cv2
import numpy as np
from scipy.ndimage import distance_transform_edt
from skimage.metrics import structural_similarity as ssim


def load_map_and_yaml(yaml_path):
    """Loads a ROS map YAML file and its corresponding image (PGM/PNG)."""
    if not os.path.exists(yaml_path):
        raise FileNotFoundError(f"YAML file not found: {yaml_path}")

    with open(yaml_path, 'r') as f:
        info = yaml.safe_load(f)

    yaml_dir = os.path.dirname(yaml_path)
    img_rel_path = info['image']
    img_full_path = os.path.normpath(os.path.join(yaml_dir, img_rel_path))

    if not os.path.exists(img_full_path):
        raise FileNotFoundError(f"Map image not found: {img_full_path}")

    img = cv2.imread(img_full_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise ValueError(f"Failed to decode image at: {img_full_path}")

    return img, info


def align_maps_feature_based(map1_img, map1_info, map2_img, map2_info, unknown_val=205):
    """Aligns map2 onto map1's canvas using ORB feature matching and rigid RANSAC."""
    res1 = map1_info['resolution']
    res2 = map2_info['resolution']

    # 1. Rescale Map 2 if spatial resolutions differ
    if not np.isclose(res1, res2):
        scale_factor = res2 / res1
        map2_img = cv2.resize(map2_img, None, fx=scale_factor, fy=scale_factor, interpolation=cv2.INTER_NEAREST)

    h1, w1 = map1_img.shape[:2]

    # 2. Extract wall structural features using Canny edge detection
    edges1 = cv2.Canny(map1_img, 50, 150)
    edges2 = cv2.Canny(map2_img, 50, 150)

    # 3. Feature Detection via ORB
    orb = cv2.ORB_create(nfeatures=5000)
    kp1, des1 = orb.detectAndCompute(edges1, None)
    kp2, des2 = orb.detectAndCompute(edges2, None)

    if des1 is None or des2 is None or len(des1) < 4 or len(des2) < 4:
        raise ValueError("Insufficient structural features detected for visual map alignment.")

    # 4. Match features using Hamming distance
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(des1, des2)
    matches = sorted(matches, key=lambda x: x.distance)

    good_matches = matches[:max(10, int(len(matches) * 0.20))]

    src_pts = np.float32([kp2[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)
    dst_pts = np.float32([kp1[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)

    # 5. Compute Rigid Transformation Matrix (Rotation + Translation)
    M, inliers = cv2.estimateAffinePartial2D(
        src_pts, 
        dst_pts, 
        method=cv2.RANSAC, 
        ransacReprojThreshold=3.0
    )

    if M is None:
        raise RuntimeError("RANSAC failed to estimate a valid alignment transformation matrix.")

    aligned_map2 = cv2.warpAffine(map2_img, M, (w1, h1), borderValue=unknown_val)
    return aligned_map2, M


def compute_chamfer_distance(walls1, walls2, resolution):
    """Computes symmetric mean Chamfer Distance Error in meters between wall boundaries."""
    if not np.any(walls1) or not np.any(walls2):
        return float('nan')

    dist_map2 = distance_transform_edt(~walls2) * resolution
    dist_map1 = distance_transform_edt(~walls1) * resolution

    chamfer_1_to_2 = np.mean(dist_map2[walls1])
    chamfer_2_to_1 = np.mean(dist_map1[walls2])

    return (chamfer_1_to_2 + chamfer_2_to_1) / 2.0


def add_info_banner(diff_canvas, metrics, map1_path, map2_path):
    """
    Creates an informational top panel containing numerical stats and color legend,
    then stacks it onto the difference canvas.
    """
    h, w, _ = diff_canvas.shape
    panel_h = 160
    min_width = 720
    target_w = max(w, min_width)

    map1_name = os.path.basename(map1_path)
    map2_name = os.path.basename(map2_path)

    # Create dark banner background
    panel = np.zeros((panel_h, target_w, 3), dtype=np.uint8) + 30

    font = cv2.FONT_HERSHEY_SIMPLEX
    COLOR_TITLE = (0, 215, 255)  # Gold/Yellow
    COLOR_TEXT = (240, 240, 240)
    COLOR_GREEN = (0, 200, 0)
    COLOR_RED = (0, 0, 240)
    COLOR_BLUE = (240, 0, 0)
    COLOR_WHITE = (255, 255, 255)

    # Title
    cv2.putText(panel, "SLAM MAP EVALUATION REPORT", (15, 28), font, 0.65, COLOR_TITLE, 2, cv2.LINE_AA)

    # Left Column: Quantitative Metrics
    col1_x = 15
    y_start = 58
    y_step = 24

    wall_iou_str = f"Wall IoU (Overlap):  {metrics['wall_iou']*100:.2f}%"
    free_iou_str = f"Free Space IoU:      {metrics['free_iou']*100:.2f}%"
    ssim_str = f"SSIM Score:          {metrics['ssim']:.4f}"
    
    if not np.isnan(metrics['chamfer_error_m']):
        chamfer_str = f"Mean Wall Error:     {metrics['chamfer_error_m']*100:.2f} cm"
    else:
        chamfer_str = "Mean Wall Error:     N/A"

    cv2.putText(panel, wall_iou_str, (col1_x, y_start), font, 0.48, COLOR_TEXT, 1, cv2.LINE_AA)
    cv2.putText(panel, free_iou_str, (col1_x, y_start + y_step), font, 0.48, COLOR_TEXT, 1, cv2.LINE_AA)
    cv2.putText(panel, ssim_str, (col1_x, y_start + y_step*2), font, 0.48, COLOR_TEXT, 1, cv2.LINE_AA)
    cv2.putText(panel, chamfer_str, (col1_x, y_start + y_step*3), font, 0.48, COLOR_TEXT, 1, cv2.LINE_AA)

    # Right Column: Color Legend
    col2_x = target_w // 2 + 10
    cv2.putText(panel, "COLOR LEGEND:", (col2_x, 28), font, 0.55, COLOR_TITLE, 1, cv2.LINE_AA)

    legend_items = [
        (COLOR_GREEN, "Wall Match (Both Maps)"),
        (COLOR_RED, f"Map 1 Only ({map1_name})"),
        (COLOR_BLUE, f"Map 2 Only ({map2_name})"),
        (COLOR_WHITE, "Agreed Free Space")
    ]

    y_leg = 48
    for bgr, label in legend_items:
        if len(label) > 36:
            label = label[:33] + "..."
        # Draw color sample box
        cv2.rectangle(panel, (col2_x, y_leg), (col2_x + 18, y_leg + 14), bgr, -1)
        cv2.rectangle(panel, (col2_x, y_leg), (col2_x + 18, y_leg + 14), (180, 180, 180), 1)
        # Draw label
        cv2.putText(panel, label, (col2_x + 26, y_leg + 12), font, 0.45, COLOR_TEXT, 1, cv2.LINE_AA)
        y_leg += 24

    # Pad map horizontally if banner minimum width exceeds map image width
    if target_w > w:
        pad_right = target_w - w
        diff_canvas = cv2.copyMakeBorder(diff_canvas, 0, 0, 0, pad_right, cv2.BORDER_CONSTANT, value=[230, 230, 230])

    # Stack header banner on top of map canvas
    combined_image = np.vstack((panel, diff_canvas))
    return combined_image


def evaluate_slam_maps(map1_img, aligned_map2_img, map1_info, map1_path, map2_path, output_diff_path="map_diff.png"):
    """Calculates quantitative metrics and exports annotated image."""
    resolution = map1_info.get('resolution', 0.05)
    occ_thresh = map1_info.get('occupied_thresh', 0.65)
    free_thresh = map1_info.get('free_thresh', 0.196)

    # 1. Binary segmentation based on ROS occupancy standard
    walls1 = map1_img < int((1.0 - occ_thresh) * 255)
    walls2 = aligned_map2_img < int((1.0 - occ_thresh) * 255)

    free1 = map1_img > int((1.0 - free_thresh) * 255)
    free2 = aligned_map2_img > int((1.0 - free_thresh) * 255)

    # 2. Compute IoU Metrics
    wall_intersection = np.logical_and(walls1, walls2).sum()
    wall_union = np.logical_or(walls1, walls2).sum()
    wall_iou = wall_intersection / wall_union if wall_union > 0 else 0.0

    free_intersection = np.logical_and(free1, free2).sum()
    free_union = np.logical_or(free1, free2).sum()
    free_iou = free_intersection / free_union if free_union > 0 else 0.0

    # 3. Structural Similarity Index (SSIM)
    score_ssim, _ = ssim(map1_img, aligned_map2_img, full=True)

    # 4. Chamfer Wall Error
    mean_chamfer_error = compute_chamfer_distance(walls1, walls2, resolution)

    metrics = {
        "wall_iou": wall_iou,
        "free_iou": free_iou,
        "ssim": score_ssim,
        "chamfer_error_m": mean_chamfer_error
    }

    # Print Terminal Report
    print("\n================ SLAM EVALUATION REPORT ================")
    print(f" Wall IoU (Structural Overlap):  {wall_iou * 100:.2f}%")
    print(f" Free Space IoU:                 {free_iou * 100:.2f}%")
    print(f" Structural Similarity (SSIM):   {score_ssim:.4f}")
    if not np.isnan(mean_chamfer_error):
        print(f" Mean Wall Distance Error:       {mean_chamfer_error * 100:.2f} cm")
    else:
        print(f" Mean Wall Distance Error:       N/A")
    print("========================================================\n")

    # 5. Generate Base Visual Diff
    h, w = map1_img.shape[:2]
    diff_canvas = np.full((h, w, 3), 230, dtype=np.uint8)

    diff_canvas[np.logical_and(free1, free2)] = [255, 255, 255] # White = Agreed Free Space
    diff_canvas[np.logical_and(walls1, walls2)] = [0, 200, 0]   # Green = Wall Match
    diff_canvas[np.logical_and(walls1, ~walls2)] = [0, 0, 240]   # Red   = Map 1 Only
    diff_canvas[np.logical_and(~walls1, walls2)] = [240, 0, 0]   # Blue  = Map 2 Only

    # 6. Append Information Banner
    final_image = add_info_banner(diff_canvas, metrics, map1_path, map2_path)

    cv2.imwrite(output_diff_path, final_image)
    print(f"Annotated visual diff saved to: {output_diff_path}")


def main():
    parser = argparse.ArgumentParser(description="Align and evaluate two 2D SLAM occupancy grid maps.")
    parser.add_argument("--map1", required=True, help="Path to reference map YAML file")
    parser.add_argument("--map2", required=True, help="Path to evaluated map YAML file")
    parser.add_argument("--output_aligned", default="aligned_map2.pgm", help="Path to save aligned Map 2 image")
    parser.add_argument("--output_diff", default="map_diff.png", help="Path to save visual difference image")

    args = parser.parse_args()

    print(f"Loading Map 1: {args.map1}")
    map1_img, map1_info = load_map_and_yaml(args.map1)

    print(f"Loading Map 2: {args.map2}")
    map2_img, map2_info = load_map_and_yaml(args.map2)

    print("Aligning Map 2 onto Map 1 canvas...")
    aligned_map2, _ = align_maps_feature_based(map1_img, map1_info, map2_img, map2_info)
    cv2.imwrite(args.output_aligned, aligned_map2)

    print("Evaluating SLAM map quality and rendering output banner...")
    evaluate_slam_maps(
        map1_img, 
        aligned_map2, 
        map1_info, 
        args.map1, 
        args.map2, 
        output_diff_path=args.output_diff
    )


if __name__ == "__main__":
    main()