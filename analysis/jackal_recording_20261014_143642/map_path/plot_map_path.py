"""Plot the last recorded SLAM grid and time-aligned online map-frame trajectory.

Input: extracted.npz / extracted.json in this directory (offline rosbag extraction).
Trajectory is online localization history, not a final pose-graph reoptimization.
"""
from pathlib import Path
import json
import csv
import re
from datetime import datetime
from zoneinfo import ZoneInfo
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap, Normalize
from matplotlib.lines import Line2D
from matplotlib.transforms import Affine2D

OUT = Path(__file__).resolve().parent
data = np.load(OUT / "extracted.npz")
info = json.loads((OUT / "extracted.json").read_text())
plt.rcParams.update({
    "font.family": "DejaVu Sans", "axes.unicode_minus": False,
    "font.size": 11, "axes.titlesize": 17, "axes.labelsize": 11,
    "axes.spines.top": False, "axes.spines.right": False,
    "savefig.facecolor": "white",
})


def clean_tf(series):
    series = series[np.argsort(series[:, 0], kind="stable")]
    # Repeated stamps occur in this recording; retain the last received update.
    return series[np.r_[np.diff(series[:, 0]) > 0, True]]


def interpolate(series, t):
    return np.column_stack([
        np.interp(t, series[:, 0], series[:, 2]),
        np.interp(t, series[:, 0], series[:, 3]),
        np.interp(t, series[:, 0], np.unwrap(series[:, 4])),
    ])


def compose(a, b):
    c, s = np.cos(a[..., 2]), np.sin(a[..., 2])
    return np.stack([
        a[..., 0] + c * b[..., 0] - s * b[..., 1],
        a[..., 1] + s * b[..., 0] + c * b[..., 1],
        a[..., 2] + b[..., 2],
    ], axis=-1)


def tf_series(parent, child):
    item = next(x for x in info["dynamic"] if (x["parent"], x["child"]) == (parent, child))
    return clean_tf(data[item["key"]])


odom = tf_series("odom", "base_link")
localizer = tf_series("map", "odom_start")
offset = np.array(next(x["pose"] for x in info["static"]
                      if (x["parent"], x["child"]) == ("odom_start", "odom")))
lo = max(odom[0, 0], localizer[0, 0])
hi = min(odom[-1, 0], localizer[-1, 0])
valid = (odom[:, 0] >= lo) & (odom[:, 0] <= hi)
times = odom[valid, 0]
poses = compose(interpolate(localizer, times), compose(offset, interpolate(odom, times)))
poses[:, 2] = np.arctan2(np.sin(poses[:, 2]), np.cos(poses[:, 2]))
elapsed = times - times[0]
steps = np.linalg.norm(np.diff(poses[:, :2], axis=0), axis=1)
continuous = (np.diff(times) <= 0.5) & (steps <= 0.5)
assert continuous.all(), "Trajectory gaps/jumps require explicit segmented plotting"
assert np.max(np.diff(localizer[:, 0])) < 0.5
assert info["map"]["frame"] == "map" and info["pose_frames"] == {"map": 406}

# Independent check against the recorded SLAM pose topic at its header timestamp.
reference = data["poses"]
reference = reference[(reference[:, 0] >= lo) & (reference[:, 0] <= hi)]
reconstructed = compose(interpolate(localizer, reference[:, 0]),
                        compose(offset, interpolate(odom, reference[:, 0])))
residual = np.linalg.norm(reconstructed[:, :2] - reference[:, 2:4], axis=1)

np.savetxt(OUT / "trajectory_map.csv",
           np.column_stack([times, elapsed, poses[:, :2], np.rad2deg(poses[:, 2])]),
           delimiter=",", header="stamp_unix_s,elapsed_s,map_x_m,map_y_m,yaw_deg",
           comments="", fmt=["%.9f", "%.6f", "%.6f", "%.6f", "%.6f"])


# The bag has no action/goal topics. RViz provides request timestamps on the
# same Host clock as bag receipt timestamps, not accepted-goal boundaries.
source = json.loads((OUT / "goal_request_source.json").read_text())
log_lines = (OUT / source["copy"]).read_text().splitlines()
receipts = odom[valid, 1]
assert np.all(np.diff(receipts) >= 0)
goal_events = []
for line_number, line in enumerate(log_lines, 1):
    if "NavigateToPose will be called using" not in line:
        continue
    match = re.search(r"\[(\d+\.\d+)\]", line)
    if not match:
        continue
    request = float(match.group(1))
    if not receipts[0] <= request <= receipts[-1]:
        continue
    nearest = int(np.argmin(np.abs(receipts - request)))
    goal_events.append({
        "goal_id": f"G{len(goal_events)+1}",
        "request_host_unix_s": request,
        "request_host_local": datetime.fromtimestamp(
            request, ZoneInfo("America/New_York")).isoformat(timespec="milliseconds"),
        "elapsed_host_s": request - receipts[0],
        "trajectory_index": nearest,
        "matched_tf_header_unix_s": float(times[nearest]),
        "matched_tf_receipt_unix_s": float(receipts[nearest]),
        "receipt_match_error_s": float(receipts[nearest] - request),
        "robot_map_x_m": float(poses[nearest, 0]),
        "robot_map_y_m": float(poses[nearest, 1]),
        "source_line": line_number,
    })
assert len(goal_events) == 11
request_times = np.array([g["request_host_unix_s"] for g in goal_events])
assert np.all(np.diff(request_times) > 0)
assert max(abs(g["receipt_match_error_s"]) for g in goal_events) < 0.1
goal_ids = np.searchsorted(request_times, receipts, side="right")
with (OUT / "goal_requests.csv").open("w", newline="") as stream:
    writer = csv.DictWriter(stream, fieldnames=list(goal_events[0]))
    writer.writeheader()
    writer.writerows(goal_events)
with (OUT / "trajectory_by_goal.csv").open("w", newline="") as stream:
    writer = csv.writer(stream)
    writer.writerow(["tf_header_unix_s", "host_receipt_unix_s", "map_x_m",
                     "map_y_m", "yaw_deg", "goal_request_interval"])
    writer.writerows(
        [f"{t:.9f}", f"{r:.9f}", f"{p[0]:.6f}", f"{p[1]:.6f}",
         f"{np.rad2deg(p[2]):.6f}", f"G{g}" if g else "Before G1"]
        for t, r, p, g in zip(times, receipts, poses, goal_ids)
    )

# Preserve the complete recorded grid and origin; unknown != free.
grid = data["map"]
shown = np.where(grid < 0, 0, np.where(grid >= 65, 2, 1))
map_cmap = ListedColormap(["#e6eaf0", "#ffffff", "#263746"])
res = info["map"]["resolution"]
ox, oy, oyaw = info["map"]["origin"]
width, height = info["map"]["width"] * res, info["map"]["height"] * res
map_transform = Affine2D().rotate(oyaw).translate(ox, oy)
corners = map_transform.transform([[0, 0], [width, 0], [0, height], [width, height]])
full_bounds = (corners[:, 0].min(), corners[:, 0].max(),
               corners[:, 1].min(), corners[:, 1].max())


# Include transition samples so display downsampling preserves goal boundaries.
boundaries = np.flatnonzero(np.diff(goal_ids) != 0) + 1
chosen = np.unique(np.r_[np.arange(0, len(times), 5), boundaries-1,
                         boundaries, len(times)-1])
xy = poses[chosen, :2]
minutes = (receipts[chosen] - receipts[0]) / 60
segments = np.stack([xy[:-1], xy[1:]], axis=1)
norm = Normalize(0, minutes[-1])
cmap = plt.get_cmap("plasma")
chain = "map -> odom_start -> odom -> base_link"
palette = ["#96a0aa", "#256aa5", "#e17c25", "#338d57", "#c84b45",
           "#895ba9", "#8a6349", "#cc6097", "#1593a3", "#a58b12",
           "#3d438b", "#384c5c"]
label_offsets = [(8, 16), (0, -24), (0, 17), (0, -24), (0, 18), (0, -25),
                 (-8, -25), (15, -13), (15, 2), (15, 15), (-43, 12)]


def make_figure(name, title, route=True, detail=False, by_goal=True):
    fig, ax = plt.subplots(figsize=(15, 7.8))
    fig.subplots_adjust(left=0.065, right=0.975, top=0.85,
                        bottom=0.24 if route else 0.13)
    ax.imshow(shown, origin="lower", interpolation="nearest", cmap=map_cmap,
              vmin=0, vmax=2, extent=[0, width, 0, height],
              transform=map_transform + ax.transData, zorder=0)
    if detail:
        xmin, ymin = poses[:, :2].min(axis=0) - [2, 2.5]
        xmax, ymax = poses[:, :2].max(axis=0) + [2.5, 2.5]
        ax.set(xlim=(xmin, xmax), ylim=(ymin, ymax))
    else:
        ax.set(xlim=full_bounds[:2], ylim=full_bounds[2:])
    ax.set_aspect("equal")
    ax.set(xlabel="Map X (m)", ylabel="Map Y (m)")
    ax.grid(color="#597181", alpha=0.13, linewidth=0.7)
    for spine in ax.spines.values():
        spine.set_color("#9aabb8")
    fig.suptitle(title, x=0.065, ha="left", fontsize=21, fontweight="bold", y=0.97)
    fig.text(0.065, 0.92,
             "jackal_20261014_143642  |  Oct 14, 2026  14:36-14:54 EDT"
             "  |  Map resolution: 0.05 m",
             color="#506376", fontsize=10.5)
    if route:
        ax.plot(xy[:, 0], xy[:, 1], color="#87929c", alpha=0.65, lw=4.1, zorder=2)
        if by_goal:
            # One polyline per interval preserves dash phase on retraced paths.
            for goal_number in range(12):
                selected = np.flatnonzero(goal_ids[chosen] == goal_number)
                if not len(selected):
                    continue
                first, last = selected[0], min(selected[-1] + 1, len(xy)-1)
                path = xy[first:last+1]
                style = "solid" if goal_number < 10 else "--" if goal_number == 10 else ":"
                ax.plot(path[:, 0], path[:, 1], color=palette[goal_number],
                        linestyle=style, linewidth=2.7, zorder=3)
        else:
            lines = LineCollection(segments, cmap=cmap, norm=norm, linewidths=2.5, zorder=3)
            lines.set_array(minutes[:-1])
            ax.add_collection(lines)
        if by_goal:
            # Mark approximate robot positions at request times, not target poses.
            for i, g in enumerate(goal_events):
                point = [g["robot_map_x_m"], g["robot_map_y_m"]]
                color = palette[i+1]
                ax.scatter(*point, facecolor="white", edgecolor=color, s=65,
                           linewidth=1.8, zorder=7)
                ax.annotate(g["goal_id"], xy=point, xytext=label_offsets[i],
                            textcoords="offset points", fontsize=10, fontweight="bold",
                            color=color, ha="left" if label_offsets[i][0] > 0 else "center",
                            bbox=dict(boxstyle="round,pad=0.22", facecolor="white",
                                      edgecolor="none", alpha=0.94),
                            arrowprops=dict(arrowstyle="-", color=color, lw=0.8),
                            zorder=8)
            handles = [Line2D([0], [0], color=palette[g], lw=3,
                              linestyle="solid" if g < 10 else "--" if g == 10 else ":",
                              label="Before G1" if g == 0 else f"G{g}")
                       for g in range(12)]
            handles = [handles[i] for i in [0, 6, 1, 7, 2, 8, 3, 9, 4, 10, 5, 11]]
            fig.legend(handles=handles, loc="lower center", bbox_to_anchor=(0.52, 0.105),
                       ncol=6, frameon=False, fontsize=10,
                       columnspacing=1.7, handlelength=2.3)
            fig.text(0.065, 0.065,
                     "G1-G11: goal-request intervals. Labels mark where the robot was estimated "
                     "to be when each RViz request was issued.",
                     fontsize=10, color="#364b60")
            fig.text(0.065, 0.035,
                     "Request positions are approximate. Target coordinates and Nav2 acceptance/"
                     "completion times are unavailable.",
                     fontsize=9, color="#657587")
        else:
            cax = fig.add_axes([0.25, 0.11, 0.5, 0.023])
            cb = fig.colorbar(lines, cax=cax, orientation="horizontal")
            cb.set_label("Elapsed Host receipt time (min)", fontsize=10)
            cb.ax.tick_params(labelsize=9)
            cb.outline.set_visible(False)
            fig.text(0.065, 0.035,
                     "Recorded online localization history over the last SLAM grid."
                     "  Gray: unknown  |  White: free  |  Dark: occupied",
                     fontsize=9, color="#506376")
        ax.scatter(*xy[0], c="#00a875", edgecolor="white", s=110, linewidth=1.6, zorder=9)
        ax.scatter(*xy[-1], c="#e24845", edgecolor="white", s=180, linewidth=1.1,
                   marker="*", zorder=9)
        ax.annotate("Start" if by_goal else f"Start ({xy[0,0]:.2f}, {xy[0,1]:.2f})",
                    xy=xy[0], xytext=(-7, -23), textcoords="offset points",
                    fontsize=10, color="#006a50", ha="right",
                    bbox=dict(boxstyle="round,pad=0.25", facecolor="white",
                              edgecolor="none", alpha=0.92), zorder=10)
        ax.annotate("End" if by_goal else f"End ({xy[-1,0]:.2f}, {xy[-1,1]:.2f})",
                    xy=xy[-1], xytext=(-16, 17), textcoords="offset points",
                    fontsize=10, color="#a52d2d", ha="right",
                    bbox=dict(boxstyle="round,pad=0.25", facecolor="white",
                              edgecolor="none", alpha=0.92), zorder=10)
    else:
        fig.text(0.065, 0.045,
                 "Last recorded SLAM occupancy grid."
                 "  Gray: unknown  |  White: free  |  Dark: occupied",
                 fontsize=10, color="#506376")
    fig.savefig(OUT / f"{name}.png", dpi=180)
    fig.savefig(OUT / f"{name}.pdf")
    plt.close(fig)


make_figure("map_and_path", "Jackal map and trajectory by goal request")
make_figure("path_detail", "Jackal trajectory by goal request - detail", detail=True)
make_figure("map_only", "Jackal SLAM map", route=False)
make_figure("map_and_path_time", "Jackal map and trajectory over time", by_goal=False)
make_figure("path_detail_time", "Jackal trajectory over time - detail", detail=True, by_goal=False)

summary = {
    "bag": info["bag"], "map": info["map"],
    "trajectory_source": chain,
    "method": "Planar composition of time-interpolated TF; latest receipt for duplicate stamps; no extrapolation",
    "meaning": "Recorded online localization history over the last recorded SLAM map; not ground truth, not a planned path, not final pose-graph reoptimization",
    "samples": len(times), "duration_s": float(elapsed[-1]),
    "start_map_xy_m": poses[0, :2].tolist(), "end_map_xy_m": poses[-1, :2].tolist(),
    "trajectory_length_m": float(steps.sum()),
    "length_note": "Includes online SLAM corrections and localization noise",
    "max_sample_step_m": float(steps.max()),
    "max_odom_tf_gap_s": float(np.diff(odom[:, 0]).max()),
    "max_map_tf_gap_s": float(np.diff(localizer[:, 0]).max()),
    "slam_pose_crosscheck_count": len(reference),
    "slam_pose_crosscheck_median_p95_max_m": np.quantile(residual, [.5, .95, 1]).tolist(),
    "goal_request_count": len(goal_events),
    "goal_source": source,
    "goal_boundary_meaning": "RViz request intervals, including any waiting after completion; not confirmed active-goal intervals",
    "goal_positions": "Approximate: nearest TF Host receipt to RViz log time; not target coordinates",
    "goal_alignment_max_abs_receipt_error_s": max(abs(g["receipt_match_error_s"]) for g in goal_events),
    "tf_receipt_minus_header_s_median_p95": np.quantile(receipts-times, [.5,.95]).tolist(),
    "goal_events": goal_events,
    "outputs": ["map_and_path.png", "map_and_path.pdf", "path_detail.png",
                "path_detail.pdf", "map_only.png", "map_only.pdf", "trajectory_map.csv",
                "map_and_path_time.png", "map_and_path_time.pdf", "path_detail_time.png",
                "path_detail_time.pdf", "goal_requests.csv", "trajectory_by_goal.csv", "README.md"],
}
(OUT / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n")
print(f"Saved English figures with {len(goal_events)} goal-request intervals and {len(times)} trajectory samples.")
