(() => { const q = new URLSearchParams(location.search), R = ROBOTS[q.get('robot')];
  if (R) mountLab(R); else renderHub(); })();
