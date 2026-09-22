/* Leaflet rendering: city graph, restricted airspace, routes, drones.
   Colours are the Okabe-Ito palette, which stays distinguishable under the
   common forms of colour-vision deficiency, and every route also carries its
   own dash pattern so colour is never the only channel (NFR-21). */
(function () {
  const D = window.DDROS || (window.DDROS = {});

  D.COLORS = ['#0072B2', '#D55E00', '#009E73', '#CC79A7', '#E69F00', '#56B4E9'];
  D.DASHES = [null, '10 6', '2 7', '14 4 3 4', '7 4', '1 7'];

  const NODE_STYLE = {
    warehouse:        { kind: 'icon',   cls: 'n-warehouse', size: 16 },
    charging_station: { kind: 'icon',   cls: 'n-charge',    size: 14 },
    customer:         { kind: 'circle', radius: 6, color: '#16202b', fill: '#ffffff', weight: 2 },
    waypoint:         { kind: 'circle', radius: 3, color: '#9aa8b5', fill: '#9aa8b5', weight: 1 }
  };

  let map = null;
  let layers = {};
  let nodesById = {};
  let droneColor = {};
  let tileWarned = false;

  function init() {
    // scrollWheelZoom is off on purpose: with it on, a wheel gesture over the
    // map zooms Leaflet instead of scrolling the page, which traps the reader
    // above the results table. Zoom via the +/- control, double-click or pinch.
    map = L.map('map', {
      zoomControl: true, preferCanvas: false, scrollWheelZoom: false
    });
    map.setView([24.875, 67.01], 13);

    // Ctrl/Cmd + wheel still zooms, for anyone who wants it.
    map.getContainer().addEventListener('wheel', function (ev) {
      if (!ev.ctrlKey && !ev.metaKey) return;
      ev.preventDefault();
      map.setZoom(map.getZoom() + (ev.deltaY < 0 ? 1 : -1));
    }, { passive: false });

    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
      maxZoom: 19, opacity: 0.55,
      attribution: '&copy; OpenStreetMap contributors'
    }).on('tileerror', function () {
      // Graceful degradation: the graph stays legible without a base map (C-4).
      if (tileWarned) return;
      tileWarned = true;
      const el = document.getElementById('tile-notice');
      if (el) el.hidden = false;
    }).addTo(map);

    layers = {
      edges:  L.layerGroup().addTo(map),
      zones:  L.layerGroup().addTo(map),
      routes: L.layerGroup().addTo(map),
      nodes:  L.layerGroup().addTo(map),
      drones: L.layerGroup().addTo(map)
    };
    return map;
  }

  function latlng(id) {
    const n = nodesById[id];
    return n ? [n.lat, n.lon] : null;
  }

  function nodeMarker(node) {
    const style = NODE_STYLE[node.type] || NODE_STYLE.waypoint;
    const label = `<strong>${node.name}</strong><br><span class="mono">${node.id}</span>
                   &middot; ${node.type.replace('_', ' ')}`;
    if (style.kind === 'icon') {
      return L.marker([node.lat, node.lon], {
        icon: L.divIcon({
          className: 'node-icon ' + style.cls,
          iconSize: [style.size, style.size],
          iconAnchor: [style.size / 2, style.size / 2]
        }),
        title: node.name
      }).bindTooltip(label);
    }
    return L.circleMarker([node.lat, node.lon], {
      radius: style.radius, color: style.color, fillColor: style.fill,
      fillOpacity: 1, weight: style.weight
    }).bindTooltip(label);
  }

  function drawScenario(scenario) {
    nodesById = {};
    scenario.nodes.forEach(n => { nodesById[n.id] = n; });

    layers.edges.clearLayers();
    layers.nodes.clearLayers();
    scenario.edges.forEach(e => {
      const a = latlng(e.u), b = latlng(e.v);
      if (!a || !b) return;
      L.polyline([a, b], { color: '#b6c3d0', weight: 1.4, opacity: 0.85 })
        .bindTooltip(`${e.u} &ndash; ${e.v} &middot; ${e.distance_km} km`)
        .addTo(layers.edges);
    });
    scenario.nodes.forEach(n => nodeMarker(n).addTo(layers.nodes));

    scenario.drones.forEach((d, i) => {
      droneColor[d.id] = D.COLORS[i % D.COLORS.length];
    });

    // invalidateSize first: Leaflet caches the container dimensions, and fitting
    // against a stale size leaves the graph a speck in the middle of the map.
    const bounds = L.latLngBounds(scenario.nodes.map(n => [n.lat, n.lon]));
    const fit = () => {
      map.invalidateSize(false);
      map.fitBounds(bounds, { padding: [34, 34] });
    };
    fit();
    setTimeout(fit, 120);
    window.addEventListener('resize', () => setTimeout(fit, 80));
  }

  function drawZones(zones, activeIds) {
    layers.zones.clearLayers();
    zones.forEach(z => {
      if (activeIds.indexOf(z.id) === -1) return;
      const style = { color: '#D55E00', weight: 2, dashArray: '6 4',
                      fillColor: '#D55E00', fillOpacity: 0.13 };
      let shape = null;
      if (z.shape === 'circle' && z.centre) {
        shape = L.circle(z.centre, Object.assign({ radius: z.radius_km * 1000 }, style));
      } else if (z.shape === 'polygon' && z.vertices) {
        shape = L.polygon(z.vertices, style);
      }
      if (shape) shape.bindTooltip(`No-fly: ${z.name}`).addTo(layers.zones);
    });
  }

  function colorFor(droneId, index) {
    if (!droneColor[droneId]) {
      droneColor[droneId] = D.COLORS[(index || 0) % D.COLORS.length];
    }
    return droneColor[droneId];
  }

  function dashFor(droneId, order) {
    return D.DASHES[order % D.DASHES.length];
  }

  function drawPlan(plan) {
    layers.routes.clearLayers();
    const order = {};
    plan.drones.forEach((d, i) => { order[d.id] = i; });

    plan.assignments.forEach(a => {
      const pts = a.route.path.map(latlng).filter(Boolean);
      if (pts.length < 2) return;
      const idx = order[a.drone_id] || 0;
      L.polyline(pts, {
        color: colorFor(a.drone_id, idx),
        weight: 3.6, opacity: 0.85,
        dashArray: dashFor(a.drone_id, idx)
      }).bindTooltip(
        `<strong>${a.delivery_id}</strong> &middot; ${a.drone_id}<br>` +
        `${a.route.distance_km} km &middot; ${a.route.energy_pct}% &middot; ` +
        `arrives ${a.arrive_min} min`
      ).addTo(layers.routes);

      if (a.charging_stop) {
        L.circleMarker(latlng(a.charging_stop.station_id), {
          radius: 10, color: '#E69F00', weight: 3, fill: false, dashArray: '3 3'
        }).bindTooltip(`Recharge stop: ${a.charging_stop.recharge_min} min`)
          .addTo(layers.routes);
      }
    });
    renderLegend(plan, order);
  }

  function renderLegend(plan, order) {
    const el = document.getElementById('legend');
    if (!el) return;
    const rows = plan.drones.map(d => {
      const i = order[d.id] || 0;
      const dash = dashFor(d.id, i);
      const bg = dash
        ? `repeating-linear-gradient(90deg, ${colorFor(d.id, i)} 0 5px, transparent 5px 9px)`
        : colorFor(d.id, i);
      return `<div class="row"><span class="swatch" style="background:${bg}"></span>
              <span class="mono">${d.id}</span>
              <span style="color:var(--muted)">${d.deliveries} pkg</span></div>`;
    }).join('');
    el.innerHTML = rows +
      `<div class="row" style="margin-top:6px;border-top:1px solid var(--line);padding-top:6px">
         <span class="dot" style="background:#16202b;border-radius:2px"></span>warehouse</div>
       <div class="row"><span class="dot" style="background:#E69F00;transform:rotate(45deg)"></span>charging</div>
       <div class="row"><span class="dot" style="background:#fff;border:2px solid #16202b;border-radius:50%"></span>customer</div>`;
  }

  function setDronePositions(positions) {
    layers.drones.clearLayers();
    Object.keys(positions).forEach(function (id) {
      const p = positions[id];
      if (!p) return;
      L.marker(p.at, {
        icon: L.divIcon({
          className: 'drone-icon',
          html: `<span style="background:${colorFor(id)}"></span><b>${id}</b>`,
          iconSize: [40, 16], iconAnchor: [20, 8]
        }),
        interactive: false, zIndexOffset: 1000
      }).addTo(layers.drones);
    });
  }

  function clearDrones() { layers.drones.clearLayers(); }

  D.map = { init, drawScenario, drawZones, drawPlan, latlng,
            colorFor, setDronePositions, clearDrones,
            nodes: () => nodesById };
})();
