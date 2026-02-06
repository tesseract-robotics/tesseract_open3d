# This Python file uses the following encoding: utf-8
# All frames at once, colored by frame (downsample to keep it light)
# python3 viz_sensor_frames.py --pattern "/tmp/orbit_spin_*.pcd" --mode accumulate --down 0.02

# Or play them as an animation (space-saving, only one in memory at a time)
# python3 viz_sensor_frames.py --pattern "/tmp/orbit_spin_*.pcd" --mode animate --delay 0.03

#!/usr/bin/env python3
import argparse, glob, os, time
import numpy as np
import open3d as o3d

def color_from_idx(i, n):
    # simple colormap: hue sweep
    h = i / max(1, n-1)
    # tiny HSV->RGB
    k = int(h * 6)
    f = h*6 - k
    p, q, t = 0, 1-f, f
    r,g,b = [(1,t,0),(q,1,0),(0,1,t),(0,q,1),(t,0,1),(1,0,q)][k%6]
    return np.array([r,g,b], dtype=np.float64)

def load_pcd(path, down=None, color=None):
    pcd = o3d.io.read_point_cloud(path)
    if down:
        pcd = pcd.voxel_down_sample(down)
    if color is not None:
        if len(pcd.points) > 0:
            pcd.colors = o3d.utility.Vector3dVector(np.tile(color, (len(pcd.points),1)))
    return pcd

def visualize_accumulate(files, down=None):
    geoms = []
    n = len(files)
    for i, f in enumerate(files):
        c = color_from_idx(i, n)
        geoms.append(load_pcd(f, down=down, color=c))
    o3d.visualization.draw_geometries(geoms, window_name="All frames (accumulated)")

def visualize_animate(files, down=None, delay=0.05):
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Frames (animate)")
    base = load_pcd(files[0], down=down)
    vis.add_geometry(base)
    for f in files[1:]:
        pc = load_pcd(f, down=down)
        base.points = pc.points
        base.colors = pc.colors if pc.has_colors() else o3d.utility.Vector3dVector()
        base.normals = pc.normals if pc.has_normals() else o3d.utility.Vector3dVector()
        vis.update_geometry(base)
        vis.poll_events(); vis.update_renderer()
        time.sleep(delay)
    print("Animation finished — close the window to exit.")
    # Keep last frame on screen
    while True:
        vis.poll_events(); vis.update_renderer()
        time.sleep(0.02)

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--pattern", required=True,
                    help="Glob for a single sensor’s frames, e.g. /tmp/orbit_spin_*.pcd")
    ap.add_argument("--mode", choices=["accumulate","animate"], default="accumulate")
    ap.add_argument("--down", type=float, default=None, help="Voxel size to downsample (meters)")
    ap.add_argument("--delay", type=float, default=0.05, help="Animate delay (s) between frames")
    args = ap.parse_args()

    files = sorted(glob.glob(args.pattern))
    if not files:
        raise SystemExit(f"No files match: {args.pattern}")

    print(f"Found {len(files)} frames")
    if args.mode == "accumulate":
        visualize_accumulate(files, down=args.down)
    else:
        visualize_animate(files, down=args.down, delay=args.delay)
