# Robotics Lab

Build robots part by part, look inside them, then operate them — in the browser, no hardware.

Open `robotics-lab.html` (single self-contained file, works offline). The hub lists 10 robots; four are fully built:

1. Six-axis industrial arm — joint sliders, drag-the-tip IK, gravity torque, crashes, teach & playback
2. SCARA arm — planar IK, elbow flip, workspace, pick & place
3. Differential-drive rover — joystick / waypoints, encoders, odometry drift, lidar
4. Hexapod — leg IK, tripod / ripple / wave gaits, support polygon, leg lift

## Source

`src/` holds the code: `kit.js` (transforms, primitives, depth sort), `lab-ui.js` (page shell, panel, hub), one file per robot, `main.js`.
Rebuild the single page with:

```sh
./src/make.sh
```

Drawing engine: `src/core.js` and `src/build.py` from Colorfigures (MIT, see `src/COLORFIGURES-LICENSE`).
