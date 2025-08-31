# Automated Driving with MATLAB — Custom App (by Zeinab Eid)

This package contains a lightweight MATLAB app for **lane detection**, **vehicle detection**, and simple **decision hints**. It is designed as a custom project for the "Automated Driving with MATLAB" learning path.

## Files
- `AutomatedDrivingApp.m` — Main app (GUI with uifigure) and processing pipeline
- `demo_startup.m` — Tiny launcher script

## How to Run
1. Open MATLAB (R2021b or later recommended).
2. Add this folder to the path: `addpath(genpath(pwd))`
3. Run the launcher:
   ```matlab
   demo_startup
   ```
4. In the app, click **Load Video** and select a driving video (MP4/AVI). Press **Play**.

## Features
- Lane detection via Canny + Hough transform
- Vehicle detection via `vehicleDetectorACF()` if available, otherwise a cascade detector fallback
- Adjustable edge sensitivity
- FPS display and status messages
- Snapshot saving

## Notes
- For best results, install **Computer Vision Toolbox** and **Automated Driving Toolbox**.
- You can replace the detection modules with pretrained YOLO/SSD models if you have them.