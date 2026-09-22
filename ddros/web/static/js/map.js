/* Leaflet rendering: city graph, restricted airspace, routes, drones.
   Colours are the Okabe-Ito palette, which stays distinguishable under the
   common forms of colour-vision deficiency, and every route also carries its
   own dash pattern so colour is never the only channel (NFR-21). */
(function () {
  const D = window.DDROS || (window.DDROS = {});

  D.COLORS = ['#0072B2', '#D55E00', '#009E73', '#CC79A7', '#E69F00', '#56B4E9'];
  D.DASHES = [null, '10 6', '2 7', '14 4 3 4', '7 4', '1 7'];

  const NODE_STYLE = {
    warehouse:        { kind: 'icon',   cls: 'n-warehouse', size: 20 },
    charging_station: { kind: 'icon',   cls: 'n-charge',    size: 17 },
    customer:         { kind: 'circle', radius: 7.5, color: '#16202b', fill: '#ffffff', weight: 2.4 },
    waypoint:         { kind: 'circle', radius: 4, color: '#9aa8b5', fill: '#9aa8b5', weight: 1.2 }
  };

  let map = null;
  let layers = {};
  let refit = function () {};
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

    // Kestrel Bay is fictional, so there is no real base map to tile. The
    // geography is drawn from data/city_backdrop.json instead, which also
    // leaves the application with no network dependency at run time (C-4).
    layers = {
      water:  L.layerGroup().addTo(map),
      parks:  L.layerGroup().addTo(map),
      river:  L.layerGroup().addTo(map),
      labels: L.layerGroup().addTo(map),
      edges:  L.layerGroup().addTo(map),
      zones:  L.layerGroup().addTo(map),
      routes: L.layerGroup().addTo(map),
      nodes:  L.layerGroup().addTo(map),
      drops:  L.layerGroup().addTo(map),
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
                   &middot; ${node.type.replace('_', ' ')}` +
                  (node.district ? `<br><span style="color:#5b6b7c">${node.district}</span>` : '');
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

  function drawBackdrop(backdrop) {
    ['water', 'parks', 'river', 'labels'].forEach(k => layers[k].clearLayers());
    if (!backdrop) return;

    (backdrop.water || []).forEach(w => {
      L.polygon(w.points, { color: '#8fb8d4', weight: 1, fillColor: '#bcd9ea',
                            fillOpacity: 1, interactive: false })
        .addTo(layers.water);
      const centre = L.polygon(w.points).getBounds().getCenter();
      L.marker(centre, { interactive: false, icon: L.divIcon({
        className: 'geo-label water-label', html: w.name, iconSize: [120, 16]
      })}).addTo(layers.labels);
    });

    (backdrop.parks || []).forEach(g => {
      L.polygon(g.points, { color: '#9dc79d', weight: 1, fillColor: '#cfe6c9',
                            fillOpacity: 1, interactive: false })
        .addTo(layers.parks);
    });

    if (backdrop.river) {
      L.polyline(backdrop.river.points, {
        color: '#bcd9ea', weight: 9, opacity: 1, lineJoin: 'round',
        lineCap: 'round', interactive: false
      }).addTo(layers.river);
    }

    (backdrop.districts || []).forEach(d => {
      L.marker(d.at, { interactive: false, icon: L.divIcon({
        className: 'geo-label district-label', html: d.name, iconSize: [130, 16]
      })}).addTo(layers.labels);
    });
  }

  function drawScenario(scenario) {
    nodesById = {};
    scenario.nodes.forEach(n => { nodesById[n.id] = n; });

    layers.edges.clearLayers();
    layers.nodes.clearLayers();
    scenario.edges.forEach(e => {
      const a = latlng(e.u), b = latlng(e.v);
      if (!a || !b) return;
      L.polyline([a, b], { color: '#b6c3d0', weight: 1.8, opacity: 0.9 })
        .bindTooltip(`${e.u} &ndash; ${e.v} &middot; ${e.distance_km} km`)
        .addTo(layers.edges);
    });
    scenario.nodes.forEach(n => nodeMarker(n).addTo(layers.nodes));

    scenario.drones.forEach((d, i) => {
      droneColor[d.id] = D.COLORS[i % D.COLORS.length];
    });

    // invalidateSize first: Leaflet caches the container dimensions, and fitting
    // against a stale size leaves the graph a speck in the middle of the map.
    // The city is roughly square, so on a wide short panel the fit is limited by
    // height; the padding is kept tight so the graph uses what height there is.
    const bounds = L.latLngBounds(scenario.nodes.map(n => [n.lat, n.lon]));
    refit = function () {
      map.invalidateSize(false);
      map.fitBounds(bounds, { padding: [18, 18] });
    };
    refit();
    setTimeout(refit, 120);
    window.addEventListener('resize', () => setTimeout(refit, 80));
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
        weight: 4.4, opacity: 0.9,
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

  function droneSvg(color, heading, flying) {
    // Top-view quadcopter. Drawn nose-up, then rotated to the flight heading,
    // so a viewer can read direction of travel from the aircraft itself.
    const rotor = (cx, cy) => `
      <g class="rotor" style="transform-origin:${cx}px ${cy}px">
        <circle cx="${cx}" cy="${cy}" r="8.2" fill="${color}" fill-opacity="0.16"/>
        <circle cx="${cx}" cy="${cy}" r="8.2" fill="none" stroke="${color}"
                stroke-width="1.1" stroke-opacity="0.55"/>
        <path d="M${cx - 7.4} ${cy} H${cx + 7.4}" stroke="${color}"
              stroke-width="2.1" stroke-linecap="round"/>
      </g>`;
    return `<svg viewBox="0 0 48 48" width="42" height="42"
                 class="drone-svg${flying ? ' flying' : ''}"
                 style="transform:rotate(${heading}deg)">
      <g stroke="${color}" stroke-width="3" stroke-linecap="round">
        <path d="M14 14 L34 34"/><path d="M34 14 L14 34"/>
      </g>
      ${rotor(14, 14)}${rotor(34, 14)}${rotor(14, 34)}${rotor(34, 34)}
      <rect x="18" y="17" width="12" height="15" rx="4.5"
            fill="${color}" stroke="#ffffff" stroke-width="1.6"/>
      <path d="M24 15.5 L27 19 H21 Z" fill="#ffffff"/>
    </svg>`;
  }

  function setDronePositions(positions) {
    layers.drones.clearLayers();
    Object.keys(positions).forEach(function (id) {
      const p = positions[id];
      if (!p) return;
      L.marker(p.at, {
        icon: L.divIcon({
          className: 'drone-marker',
          html: droneSvg(colorFor(id), p.heading || 0, !!p.flying) +
                `<b style="border-color:${colorFor(id)}">${id}</b>`,
          iconSize: [42, 56], iconAnchor: [21, 21]
        }),
        interactive: false, zIndexOffset: 1000
      }).addTo(layers.drones);
    });
  }

  function clearDrones() { layers.drones.clearLayers(); }

  const REDUCED_MOTION =
    window.matchMedia && window.matchMedia('(prefers-reduced-motion: reduce)').matches;
  const DROP_MS = 1500;

  function dropPackage(nodeId, color) {
    // A parcel released over the destination, with a ring marking the delivery.
    // The marker removes itself once the animation is done, so nothing
    // accumulates over a long playback.
    const at = latlng(nodeId);
    if (!at || REDUCED_MOTION) return;
    const marker = L.marker(at, {
      interactive: false, zIndexOffset: 900,
      icon: L.divIcon({
        className: 'drop-marker',
        html: `<span class="ring" style="border-color:${color}"></span>
               <svg class="parcel" viewBox="0 0 20 20" width="20" height="20">
                 <rect x="2.5" y="5" width="15" height="12" rx="2"
                       fill="${color}" stroke="#ffffff" stroke-width="1.4"/>
                 <path d="M10 5 V17" stroke="#ffffff" stroke-width="1.4"/>
                 <path d="M2.5 9.5 H17.5" stroke="#ffffff" stroke-width="1.4"/>
               </svg>`,
        iconSize: [46, 46], iconAnchor: [23, 23]
      })
    }).addTo(layers.drops);
    setTimeout(() => layers.drops.removeLayer(marker), DROP_MS);
  }

  function clearDrops() { layers.drops.clearLayers(); }

  D.map = { init, drawScenario, drawBackdrop, drawZones, drawPlan, latlng,
            refit: () => refit(), dropPackage, clearDrops,
            colorFor, setDronePositions, clearDrones,
            nodes: () => nodesById };
})();
