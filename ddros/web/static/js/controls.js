/* Control panel state (UI-6 to UI-9).

   Slider positions are treated as raw proportions and normalised on read, so
   the constraint alpha + beta + gamma = 1 (FR-4.3) cannot be violated from the
   interface however the sliders are dragged. */
(function () {
  const D = window.DDROS || (window.DDROS = {});
  const DEBOUNCE_MS = 150;

  function el(id) { return document.getElementById(id); }

  function weights() {
    let a = Number(el('w-alpha').value);
    let b = Number(el('w-beta').value);
    let g = Number(el('w-gamma').value);
    const total = a + b + g;
    if (total <= 0) { a = 100; b = 0; g = 0; }
    const sum = a + b + g;
    return { alpha: a / sum, beta: b / sum, gamma: g / sum };
  }

  function config() {
    return {
      weights: weights(),
      wind: {
        speed_ms: Number(el('wind-speed').value),
        bearing_deg: Number(el('wind-bearing').value)
      },
      active_no_fly_zones: Array.from(
        document.querySelectorAll('#zone-list input:checked')).map(i => i.value),
      algorithm: document.querySelector('input[name=algo]:checked').value,
      reserve_pct: Number(el('reserve').value)
    };
  }

  function syncReadouts() {
    const w = weights();
    el('out-alpha').textContent = w.alpha.toFixed(2);
    el('out-beta').textContent = w.beta.toFixed(2);
    el('out-gamma').textContent = w.gamma.toFixed(2);
    el('out-wind').textContent = Number(el('wind-speed').value).toFixed(1) + ' m/s';
    el('out-bearing').textContent = el('wind-bearing').value + '°';
    el('out-reserve').textContent = el('reserve').value + '%';
    const needle = document.querySelector('#compass .needle');
    if (needle) {
      needle.style.transform =
        `translate(-50%, -100%) rotate(${el('wind-bearing').value}deg)`;
    }
  }

  function populateZones(zones, onChange) {
    el('zone-list').innerHTML = zones.map(z =>
      `<label><input type="checkbox" value="${D.results.esc(z.id)}"
        ${z.active ? 'checked' : ''}> ${D.results.esc(z.name)}</label>`).join('')
      || '<p class="hint">No zones defined.</p>';
    document.querySelectorAll('#zone-list input')
      .forEach(i => i.addEventListener('change', onChange));
  }

  function populatePairs(nodes) {
    const options = nodes.map(n =>
      `<option value="${D.results.esc(n.id)}">${D.results.esc(n.id)} — ${D.results.esc(n.name)}</option>`
    ).join('');
    el('pair-from').innerHTML = options;
    el('pair-to').innerHTML = options;
    const warehouse = nodes.find(n => n.type === 'warehouse');
    const far = nodes.filter(n => n.type === 'customer').slice(-1)[0];
    if (warehouse) el('pair-from').value = warehouse.id;
    if (far) el('pair-to').value = far.id;
  }

  function debounce(fn, ms) {
    let timer = null;
    return function () {
      clearTimeout(timer);
      timer = setTimeout(fn, ms === undefined ? DEBOUNCE_MS : ms);
    };
  }

  function wire(onChange) {
    const deferred = debounce(onChange);
    ['w-alpha', 'w-beta', 'w-gamma', 'wind-speed', 'wind-bearing', 'reserve']
      .forEach(id => el(id).addEventListener('input', function () {
        syncReadouts();
        deferred();                    // one request per drag, not dozens
      }));
    document.querySelectorAll('input[name=algo]')
      .forEach(i => i.addEventListener('change', onChange));
    document.querySelectorAll('.presets button[data-preset]')
      .forEach(btn => btn.addEventListener('click', function () {
        const [a, b, g] = btn.dataset.preset.split(',').map(Number);
        el('w-alpha').value = a * 100;
        el('w-beta').value = b * 100;
        el('w-gamma').value = g * 100;
        syncReadouts();
        onChange();
      }));
    syncReadouts();
  }

  D.controls = { config, wire, populateZones, populatePairs, syncReadouts, weights };
})();
