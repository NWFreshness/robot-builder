#!/bin/sh
# concatenate the lab sources into one figure page, then let the Colorfigures builder inline the engine
set -e
cd "$(dirname "$0")"
python3 - <<'PY'
src = open('lab.src.html').read()
js = '\n'.join(open(f).read() for f in ['kit.js', 'lab-ui.js', 'arm6.js', 'scara.js', 'rover.js', 'hexapod.js', 'main.js'])
open('lab.html', 'w').write(src.replace('/*@@LAB@@*/', js))
PY
python3 build.py lab.html ../robotics-lab.html
