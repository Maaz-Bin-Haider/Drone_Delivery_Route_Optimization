/* Plan presentation (FR-10). Every decision is shown with the reason behind
   it, so a first-time reader never meets an unexplained result (FR-10.7). */
(function () {
  const D = window.DDROS || (window.DDROS = {});
  const PRIORITY_RANK = { URGENT: 0, HIGH: 1, NORMAL: 2 };

  function esc(s) {
    return String(s === null || s === undefined ? '' : s)
      .replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
  }

  function tile(key, value, bad) {
    return `<div class="tile${bad ? ' bad' : ''}">
              <div class="k">${esc(key)}</div><div class="v">${esc(value)}</div>
            </div>`;
  }

  function renderTotals(plan) {
    const t = plan.totals;
    const el = document.getElementById('totals');
    el.innerHTML =
      tile('Makespan', plan.makespan_min.toFixed(1) + ' min') +
      tile('Delivered', t.delivered) +
      tile('Unserviceable', t.unserviceable, t.unserviceable > 0) +
      tile('Total distance', t.distance_km.toFixed(1) + ' km') +
      tile('Total energy', t.energy_pct.toFixed(0) + ' %') +
      tile('Searches', plan.search.all_pairs_searches) +
      tile('Nodes expanded', plan.search.all_pairs_nodes_expanded);
  }

  function droneTag(id, plan) {
    const i = plan.drones.findIndex(d => d.id === id);
    return `<span class="drone-tag"><span class="chip"
            style="background:${D.map.colorFor(id, i < 0 ? 0 : i)}"></span>${esc(id)}</span>`;
  }

  function renderTable(plan) {
    const body = document.querySelector('#plan-table tbody');
    if (!plan.assignments.length) {
      body.innerHTML = '<tr><td colspan="9" style="color:var(--muted)">' +
                       'No deliveries could be assigned.</td></tr>';
    } else {
      body.innerHTML = plan.assignments.map(a => {
        const stop = a.charging_stop
          ? ` <span class="pill" style="background:var(--high)">via ${esc(a.charging_stop.station_id)}</span>`
          : '';
        return `<tr>
          <td class="mono">${esc(a.delivery_id)}</td>
          <td><span class="pill ${esc(a.priority)}">${esc(a.priority)}</span></td>
          <td>${droneTag(a.drone_id, plan)}</td>
          <td class="route-cell">${a.route.path.map(esc).join(' &rsaquo; ')}${stop}</td>
          <td class="num">${a.route.distance_km.toFixed(1)}</td>
          <td class="num">${a.route.energy_pct.toFixed(1)}%</td>
          <td class="num">${a.arrive_min.toFixed(1)}</td>
          <td class="mono">${esc(a.route.algorithm)}</td>
          <td class="why">${esc(a.reason)}</td>
        </tr>`;
      }).join('');
    }

    const warn = document.getElementById('unserviceable');
    warn.innerHTML = plan.unserviceable.length
      ? plan.unserviceable.map(u =>
          `<div class="warn-card"><strong>${esc(u.delivery_id)}</strong>
           &rarr; ${esc(u.destination)} (${esc(u.priority)}): ${esc(u.reason)}</div>`
        ).join('')
      : '';
  }

  function renderFleetNote(plan) {
    const f = plan.fleet;
    const el = document.getElementById('fleet-note');
    if (!f || !f.options.length) { el.innerHTML = ''; return; }
    const sizes = f.options.map(o => {
      const cls = o.drones === f.chosen ? 'sz on' : (o.unserviceable ? 'sz bad' : 'sz');
      const mark = o.unserviceable ? '\u2717' : '';
      return `<span class="${cls}" title="${o.unserviceable} undelivered">` +
             `${o.drones}: ${o.makespan_min.toFixed(0)}m${mark}</span>`;
    }).join('');
    el.innerHTML =
      `<b>${f.chosen} of ${f.chosen + f.reserve.length} launched</b>` +
      (f.reserve.length ? ` &middot; reserve ${esc(f.reserve.join(' '))}` : '') +
      `<span class="why">${esc(f.reason)}</span>` +
      `<div class="sizes">${sizes}</div>`;
  }

  function renderFleet(plan) {
    document.getElementById('fleet').innerHTML = plan.drones.map((d, i) => {
      const low = d.battery_pct < 30;
      return `<div class="fleet-card">
        <div class="head">${droneTag(d.id, plan)}
          <span class="mono">${d.battery_pct.toFixed(0)}%</span></div>
        <div class="bar${low ? ' low' : ''}">
          <span style="width:${Math.max(0, Math.min(100, d.battery_pct))}%"></span></div>
        <div class="meta">${d.deliveries} pkg &middot; ${d.distance_km.toFixed(1)} km
          &middot; busy ${d.busy_min.toFixed(0)}m &middot; idle ${d.idle_min.toFixed(0)}m</div>
        <div class="meta">${d.packages.map(esc).join(' ') || '&mdash;'}</div>
      </div>`;
    }).join('');
  }

  function renderQueue(deliveries) {
    const sorted = deliveries.slice().sort((a, b) => {
      const pa = PRIORITY_RANK[a.priority], pb = PRIORITY_RANK[b.priority];
      return pa !== pb ? pa - pb : deliveries.indexOf(a) - deliveries.indexOf(b);
    });
    document.getElementById('queue').innerHTML = sorted.map(p =>
      `<li><span class="mono">${esc(p.id)}</span>
       <span class="pill ${esc(p.priority)}">${esc(p.priority)}</span>
       &rarr; ${esc(p.destination)}</li>`).join('');
  }

  function renderOrders(deliveries, onRemove) {
    const list = document.getElementById('order-list');
    const count = document.getElementById('order-count');
    count.textContent = deliveries.length ? `(${deliveries.length})` : '';
    if (!deliveries.length) {
      list.innerHTML = '<div class="order-empty">No orders. ' +
                       'Add one above or load a plan.</div>';
      return;
    }
    list.innerHTML = deliveries.map(d => `
      <div class="order-row">
        <span class="pill ${esc(d.priority)}">${esc(d.priority[0])}</span>
        <span>${esc(d.destination_name || d.destination)}</span>
        <span class="mono" style="color:var(--muted)">${esc(d.id)}</span>
        <button class="rm" data-id="${esc(d.id)}" title="Remove">&times;</button>
      </div>`).join('');
    list.querySelectorAll('.rm').forEach(b =>
      b.addEventListener('click', () => onRemove(b.dataset.id)));
  }

  function renderPresets(presets, onLoad) {
    document.getElementById('preset-list').innerHTML = presets.map(p => `
      <div class="preset-card" data-id="${esc(p.id)}">
        <div class="t">${esc(p.name)}<span>${p.count} orders</span></div>
        <div class="s">${esc(p.summary)}</div>
      </div>`).join('');
    document.querySelectorAll('.preset-card').forEach(card =>
      card.addEventListener('click', () => {
        document.querySelectorAll('.preset-card').forEach(c => c.classList.remove('on'));
        card.classList.add('on');
        onLoad(card.dataset.id);
      }));
  }

  function renderPair(html) {
    document.getElementById('pair-result').innerHTML = html;
  }

  D.results = { renderTotals, renderTable, renderFleet, renderFleetNote, renderQueue,
                renderOrders, renderPresets, renderPair, esc };
})();
