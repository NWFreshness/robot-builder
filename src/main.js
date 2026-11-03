/* ---------- the rest of the lineup: spec sheets on the hub, built next ---------- */
ROBOT_ORDER.push(
  { id: 'hobbyarm', name: 'Desktop hobby arm', blurb: 'A 5-DOF + gripper arm of serial bus servos on 3D-printed links, the kind used for learning AI teleoperation with a leader arm.', chips: ['5 DOF', 'Bus servos', 'Leader / follower'] },
  { id: 'delta', name: 'Delta robot', blurb: 'Three motors on a fixed top plate drive three parallelogram arms down to one small platform: extremely fast, always level.', chips: ['3 DOF', 'Parallel', 'Closed chain'] },
  { id: 'gripper', name: 'Three-finger gripper', blurb: 'Tendon-driven fingers with one motor each, compliant pads and force sensing: how robot hands wrap around odd shapes.', chips: ['Underactuated', 'Tendons', 'Force sensing'] },
  { id: 'gantry', name: 'CNC gantry / plotter', blurb: 'Steppers, lead screws and timing belts on linear rails: Cartesian motion, microstepping and G-code paths.', chips: ['3 axes', 'Steppers', 'Lead screw + belts'] },
  { id: 'mobman', name: 'Mobile manipulator', blurb: 'The rover with an arm on top: base and arm coordinate so the gripper reaches what the arm alone cannot.', chips: ['Base + arm', '8 DOF', 'Whole-body'] },
  { id: 'humanoid', name: 'Humanoid upper body', blurb: 'Two 4-DOF arms, a 2-DOF neck and a camera head on a torso: redundancy, reachability and safe human-like motion.', chips: ['10 DOF', 'Redundant', 'Vision'] });
(() => { const q = new URLSearchParams(location.search), R = ROBOTS[q.get('robot')];
  if (R) mountLab(R); else renderHub(); })();
