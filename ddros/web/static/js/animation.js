/* Drone playback (UI-5).

   The plan gives each assignment a departure and arrival time and an ordered
   route. Together those form a timeline per drone; at any clock value the
   drone is either flying a leg -- interpolated along the polyline by elapsed
   fraction of that leg -- or parked at wherever it last landed. */
(function () {
  const D = window.DDROS || (window.DDROS = {});

  let timeline = {};
  let horizon = 0;
  let clock = 0;
  let playing = false;
  let raf = null;
  let lastFrame = 0;
  const SPEED = 4;                 // simulated minutes per real second

  function build(plan) {
    timeline = {};
    horizon = plan.makespan_min || 0;
    plan.assignments.forEach(a => {
      const pts = a.route.path.map(D.map.latlng).filter(Boolean);
      if (pts.length < 2) return;
      (timeline[a.drone_id] || (timeline[a.drone_id] = [])).push({
        pts: pts,
        cum: cumulative(pts),
        start: a.depart_min,
        end: Math.max(a.arrive_min, a.depart_min + 0.001)
      });
    });
    Object.keys(timeline).forEach(id => timeline[id].sort((x, y) => x.start - y.start));
    reset();
  }

  function cumulative(pts) {
    const out = [0];
    for (let i = 1; i < pts.length; i++) {
      out.push(out[i - 1] + Math.hypot(pts[i][0] - pts[i - 1][0],
                                       pts[i][1] - pts[i - 1][1]));
    }
    return out;
  }

  function pointAlong(leg, fraction) {
    const cum = leg.cum;
    const total = cum[cum.length - 1];
    if (total <= 0) return leg.pts[0];
    const want = Math.max(0, Math.min(1, fraction)) * total;
    for (let i = 1; i < cum.length; i++) {
      if (cum[i] >= want) {
        const span = cum[i] - cum[i - 1];
        const t = span > 0 ? (want - cum[i - 1]) / span : 0;
        const a = leg.pts[i - 1], b = leg.pts[i];
        return [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t];
      }
    }
    return leg.pts[leg.pts.length - 1];
  }

  function positionsAt(t) {
    const out = {};
    Object.keys(timeline).forEach(function (id) {
      const legs = timeline[id];
      let position = legs.length ? legs[0].pts[0] : null;
      for (let i = 0; i < legs.length; i++) {
        const leg = legs[i];
        if (t >= leg.end) {
          position = leg.pts[leg.pts.length - 1];       // landed, waiting
        } else if (t >= leg.start) {
          position = pointAlong(leg, (t - leg.start) / (leg.end - leg.start));
          break;
        } else {
          break;                                         // not departed yet
        }
      }
      out[id] = position ? { at: position } : null;
    });
    return out;
  }

  function render() {
    D.map.setDronePositions(positionsAt(clock));
    const slider = document.getElementById('clock');
    const label = document.getElementById('clock-label');
    if (slider) { slider.max = Math.max(horizon, 0.1); slider.value = clock; }
    if (label) label.textContent = clock.toFixed(1) + ' min';
  }

  function frame(now) {
    if (!playing) return;
    const dt = (now - lastFrame) / 1000;
    lastFrame = now;
    clock += dt * SPEED;
    if (clock >= horizon) { clock = horizon; playing = false; }
    render();
    if (playing) raf = requestAnimationFrame(frame);
  }

  function play() {
    if (playing || horizon <= 0) return;
    if (clock >= horizon) clock = 0;
    playing = true;
    lastFrame = performance.now();
    raf = requestAnimationFrame(frame);
  }

  function pause() {
    playing = false;
    if (raf) cancelAnimationFrame(raf);
  }

  function reset() {
    pause();
    clock = 0;
    render();
  }

  function seek(value) {
    pause();
    clock = Math.max(0, Math.min(horizon, Number(value) || 0));
    render();
  }

  D.animation = { build, play, pause, reset, seek };
})();
