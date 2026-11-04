# Robotics Lab

Build robots part by part, look inside them, then operate them — in the browser, no hardware.

Open `robotics-lab.html` (single self-contained file, works offline). The hub lists 10 robots, all fully built:

1. Six-axis industrial arm — joint sliders, drag-the-tip IK, gravity torque, crashes, teach & playback
2. SCARA arm — planar IK, elbow flip, workspace, pick & place
3. Differential-drive rover — joystick / waypoints, encoders, odometry drift, lidar
4. Hexapod — leg IK, tripod / ripple / wave gaits, support polygon, leg lift
5. Desktop hobby arm — bus servos, leader / follower teleop, record & replay, servo load and heat
6. Delta picker — parallel-robot IK, pick & place cycles, speed and g-forces
7. Three-finger gripper — tendon-driven underactuated fingers, force limit, grasp types, lift test, crushable egg
8. CNC gantry plotter — G-code programs, steps per mm, microstepping, lost steps from over-driving
9. Mobile manipulator — whole-body reach: the base drives and parks, the arm picks and delivers to a bin
10. Humanoid upper body — two 4-DOF arms with elbow-swivel redundancy, gaze tracking, hand speed limits

## Source

`src/` holds the code: `kit.js` (transforms, primitives, depth sort), `lab-ui.js` (page shell, panel, hub), one file per robot, `main.js`.
Rebuild the single page with:

```sh
./src/make.sh
```

Drawing engine: `src/core.js` and `src/build.py` from Colorfigures (MIT, see `src/COLORFIGURES-LICENSE`).
