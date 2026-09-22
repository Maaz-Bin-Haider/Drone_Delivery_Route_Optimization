/* Bootstrap and orchestration. */
(function () {
  const D = window.DDROS || (window.DDROS = {});
  let scenario = null;
  let planning = false;

  function status(text, cls) {
    const el = document.getElementById('status');
    el.textContent = text;
    el.className = 'status' + (cls ? ' ' + cls : '');
  }

  function toast(message) {
    const el = document.getElementById('toast');
    el.textContent = message;
    el.hidden = false;
    clearTimeout(el._timer);
    el._timer = setTimeout(() => { el.hidden = true; }, 5000);
  }

  async function call(url, options) {
    const response = await fetch(url, options);
    const body = await response.json().catch(() => ({}));
    if (!response.ok) {
      // Errors name the offending field, so the message is actionable (UI-11).
      const where = body.field ? ` (${body.field})` : '';
      throw new Error((body.error || response.statusText) + where);
    }
    return body;
  }

  async function plan() {
    if (planning) return;
    planning = true;
    status('planning…', 'busy');
    const config = D.controls.config();
    try {
      const result = await call('/api/plan', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(config)
      });
      D.map.drawZones(scenario.zones, config.active_no_fly_zones);
      D.map.drawPlan(result);
      D.animation.build(result);
      D.results.renderTotals(result);
      D.results.renderTable(result);
      D.results.renderFleetNote(result);
      D.results.renderFleet(result);
      status(`${result.totals.delivered} delivered · ` +
             `${result.makespan_min.toFixed(1)} min`);
    } catch (err) {
      status('error', 'error');
      toast(err.message);
    } finally {
      planning = false;
    }
  }

  function syncOrders(deliveries) {
    scenario.deliveries = deliveries;
    D.results.renderOrders(deliveries, removeOrder);
    D.results.renderQueue(deliveries);
  }

  async function loadPreset(presetId) {
    status('loading plan\u2026', 'busy');
    try {
      const out = await call('/api/presets/' + encodeURIComponent(presetId),
                             { method: 'POST' });
      syncOrders(out.deliveries);
      await plan();
    } catch (err) { status('error', 'error'); toast(err.message); }
  }

  async function addOrder() {
    const destination = document.getElementById('order-dest').value;
    const priority = document.getElementById('order-priority').value;
    try {
      const out = await call('/api/deliveries', {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ destination, priority })
      });
      syncOrders(out.deliveries);
      clearPresetSelection();
      await plan();
    } catch (err) { toast(err.message); }
  }

  async function removeOrder(id) {
    try {
      const out = await call('/api/deliveries/' + encodeURIComponent(id),
                             { method: 'DELETE' });
      syncOrders(out.deliveries);
      clearPresetSelection();
      await plan();
    } catch (err) { toast(err.message); }
  }

  async function clearOrders() {
    try {
      const out = await call('/api/deliveries/clear', { method: 'POST' });
      syncOrders(out.deliveries);
      clearPresetSelection();
      await plan();
    } catch (err) { toast(err.message); }
  }

  function clearPresetSelection() {
    document.querySelectorAll('.preset-card').forEach(c => c.classList.remove('on'));
  }

  async function comparePair() {
    const payload = Object.assign(D.controls.config(), {
      source: document.getElementById('pair-from').value,
      target: document.getElementById('pair-to').value
    });
    try {
      const out = await call('/api/compare', {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload)
      });
      if (out.unreachable) {
        D.results.renderPair('No route exists between those nodes.');
        return;
      }
      const ratio = (out.expansion_ratio * 100).toFixed(0);
      D.results.renderPair(
        `<strong>Dijkstra</strong>  ${out.dijkstra.stats.nodes_expanded} expanded, ` +
        `${out.dijkstra.stats.runtime_ms.toFixed(3)} ms\n` +
        `<strong>A*</strong>        ${out.astar.stats.nodes_expanded} expanded, ` +
        `${out.astar.stats.runtime_ms.toFixed(3)} ms\n` +
        `costs agree: ${out.agree}\n` +
        `A* searched ${ratio}% of Dijkstra's nodes`);
    } catch (err) { toast(err.message); }
  }

  async function alternatives() {
    const payload = Object.assign(D.controls.config(), {
      source: document.getElementById('pair-from').value,
      target: document.getElementById('pair-to').value
    });
    try {
      const out = await call('/api/route/alternatives', {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload)
      });
      if (!out.min_distance || !out.min_energy) {
        D.results.renderPair('No route exists between those nodes.');
        return;
      }
      D.results.renderPair(
        `<strong>shortest</strong>  ${out.min_distance.distance_km.toFixed(2)} km, ` +
        `${out.min_distance.energy_pct.toFixed(1)}%\n` +
        `<strong>greenest</strong>  ${out.min_energy.distance_km.toFixed(2)} km, ` +
        `${out.min_energy.energy_pct.toFixed(1)}%\n` +
        (out.annotation || ''));
    } catch (err) { toast(err.message); }
  }

  async function benchmark(which) {
    status('benchmarking…', 'busy');
    try {
      // The full sweep runs the real size ladder (V up to 1000, ~3 s); the
      // single-experiment buttons stay quick so they feel instant.
      const payload = which === 'all' ? { quick: false }
                                      : { experiment: Number(which), quick: true };
      const results = await call('/api/benchmark', {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload)
      });
      D.charts.render(results);
      status('benchmark complete');
    } catch (err) {
      status('error', 'error');
      toast(err.message);
    }
  }

  async function start() {
    D.map.init();
    try {
      scenario = await call('/api/scenario');
    } catch (err) {
      toast('Could not load the scenario: ' + err.message);
      return;
    }
    D.map.drawBackdrop(scenario.backdrop);
    D.map.drawScenario(scenario);
    D.results.renderQueue(scenario.deliveries);
    D.results.renderOrders(scenario.deliveries, removeOrder);
    D.results.renderPresets(scenario.presets, loadPreset);
    D.controls.populateZones(scenario.zones, plan);
    D.controls.populatePairs(scenario.nodes);
    D.controls.populateDestinations(scenario.nodes);
    D.controls.setRosterSize(scenario.drones.length);
    D.controls.wire(plan);

    document.getElementById('btn-plan').addEventListener('click', plan);
    document.getElementById('btn-csv').addEventListener('click',
      () => { window.location = '/api/export?format=csv'; });
    document.getElementById('btn-json').addEventListener('click',
      () => { window.open('/api/export?format=json', '_blank'); });
    document.getElementById('btn-play').addEventListener('click', D.animation.play);
    document.getElementById('btn-pause').addEventListener('click', D.animation.pause);
    document.getElementById('btn-reset').addEventListener('click', D.animation.reset);
    document.getElementById('clock').addEventListener('input',
      e => D.animation.seek(e.target.value));
    const expand = document.getElementById('btn-expand');
    function setExpanded(on) {
      document.body.classList.toggle('map-expanded', on);
      expand.querySelector('.txt').textContent = on ? 'Collapse' : 'Expand';
      // Leaflet caches container dimensions, so the map must be told the panel
      // changed size before it can refit to the new one.
      setTimeout(() => D.map.refit(), 60);
    }
    expand.addEventListener('click',
      () => setExpanded(!document.body.classList.contains('map-expanded')));
    document.addEventListener('keydown', e => {
      if (e.key === 'Escape' && document.body.classList.contains('map-expanded')) {
        setExpanded(false);
      }
    });

    document.getElementById('btn-add-order').addEventListener('click', addOrder);
    document.getElementById('btn-clear-orders').addEventListener('click', clearOrders);
    document.getElementById('btn-compare').addEventListener('click', comparePair);
    document.getElementById('btn-alts').addEventListener('click', alternatives);
    document.querySelectorAll('.analysis button[data-exp]').forEach(btn =>
      btn.addEventListener('click', () => benchmark(btn.dataset.exp)));

    await plan();
  }

  window.addEventListener('DOMContentLoaded', start);
})();
