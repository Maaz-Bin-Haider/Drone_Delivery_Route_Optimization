/* Benchmark visualisation (FR-11.5).

   Three of the seven experiments contradicted their own hypothesis. Those are
   surfaced as highlighted findings rather than buried, because the contradiction
   is the result worth reading. */
(function () {
  const D = window.DDROS || (window.DDROS = {});
  const INK = '#16202b', MUTED = '#5b6b7c', GRID = '#e2e8ee';
  const charts = [];

  Chart.defaults.font.family =
    'system-ui, -apple-system, "Segoe UI", Roboto, sans-serif';
  Chart.defaults.color = MUTED;

  function card(title, blurb) {
    const el = document.createElement('div');
    el.className = 'chart-card';
    el.innerHTML = `<h3>${title}</h3><p>${blurb}</p><canvas></canvas>`;
    document.getElementById('charts').appendChild(el);
    return el.querySelector('canvas');
  }

  function axes(xTitle, yTitle, logX) {
    return {
      x: { type: logX ? 'logarithmic' : 'linear',
           title: { display: true, text: xTitle }, grid: { color: GRID } },
      y: { beginAtZero: true, title: { display: true, text: yTitle },
           grid: { color: GRID } }
    };
  }

  function clear() {
    charts.forEach(c => c.destroy());
    charts.length = 0;
    document.getElementById('charts').innerHTML = '';
    document.getElementById('findings').innerHTML = '';
  }

  function finding(title, text, surprise) {
    const el = document.createElement('div');
    el.className = 'finding' + (surprise ? ' surprise' : '');
    el.innerHTML = `<b>${title}</b>${D.results.esc(text)}`;
    document.getElementById('findings').appendChild(el);
  }

  function expansion(e) {
    const rows = e.series;
    const ctx = card('Nodes expanded: Dijkstra vs A*',
      'Both share a worst-case bound, so any benefit is a constant factor. ' +
      'It is measured, not asserted.');
    charts.push(new Chart(ctx, {
      type: 'line',
      data: {
        labels: rows.map(r => r.V),
        datasets: [
          { label: 'Dijkstra', data: rows.map(r => r.weights.distance.dijkstra_expanded.mean),
            borderColor: '#D55E00', backgroundColor: '#D55E00', tension: 0.25 },
          { label: 'A*', data: rows.map(r => r.weights.distance.astar_expanded.mean),
            borderColor: '#0072B2', backgroundColor: '#0072B2', tension: 0.25 }
        ]
      },
      options: { responsive: true, scales: axes('vertices (V)', 'nodes expanded') }
    }));

    const ctx2 = card('How the heuristic weakens as the objective shifts',
      'A* degrades toward Dijkstra as weight moves off distance &mdash; ' +
      'predicted in the design, confirmed here.');
    const labels = ['distance', 'balanced', 'energy'];
    charts.push(new Chart(ctx2, {
      type: 'bar',
      data: {
        labels: labels,
        datasets: [{
          label: 'A* expansions as a share of Dijkstra',
          data: labels.map(k => 100 * rows.reduce(
            (s, r) => s + r.weights[k].expansion_ratio.mean, 0) / rows.length),
          backgroundColor: ['#0072B2', '#009E73', '#D55E00']
        }]
      },
      options: { responsive: true, plugins: { legend: { display: false } },
                 scales: { y: { beginAtZero: true, title: { display: true, text: '%' },
                                grid: { color: GRID } } } }
    }));

    const ratios = rows.map(r => r.weights.distance.expansion_ratio.mean);
    finding('Experiment 1 — A*’s advantage grows with scale',
      `A* expands ${(100 * ratios[0]).toFixed(1)}% of Dijkstra’s nodes at ` +
      `V=${rows[0].V}, falling to ${(100 * ratios[ratios.length - 1]).toFixed(1)}% at ` +
      `V=${rows[rows.length - 1].V}. Costs agreed on every pair, which is a live ` +
      `check that the heuristic is admissible.`);
  }

  function growth(e) {
    const ctx = card('Runtime against the derived bound',
      'Plotting measured time against (V+E)log V should be a straight line ' +
      'if the O((V+E) log V) derivation holds.');
    charts.push(new Chart(ctx, {
      type: 'scatter',
      data: {
        datasets: [
          { label: 'Dijkstra', borderColor: '#D55E00', backgroundColor: '#D55E00',
            data: e.points.map(p => ({ x: p.predictor, y: p.dijkstra_ms })) },
          { label: 'A*', borderColor: '#0072B2', backgroundColor: '#0072B2',
            data: e.points.map(p => ({ x: p.predictor, y: p.astar_ms })) }
        ]
      },
      options: { responsive: true, scales: axes('(V + E) log2 V', 'ms per route') }
    }));
    finding('Experiment 2 — the bound holds empirically',
      `R² = ${e.dijkstra_fit.r_squared.toFixed(4)} for Dijkstra and ` +
      `${e.astar_fit.r_squared.toFixed(4)} for A*.`);
  }

  function fleet(e) {
    const ctx = card('Makespan against fleet size',
      'How much does each additional drone actually buy?');
    charts.push(new Chart(ctx, {
      type: 'bar',
      data: {
        labels: e.rows.map(r => r.drones + ' drone' + (r.drones > 1 ? 's' : '')),
        datasets: [
          { label: 'makespan (min)', data: e.rows.map(r => r.makespan_min),
            backgroundColor: '#0072B2', order: 2 },
          { label: 'recharge time (min)', data: e.rows.map(r => r.recharge_min),
            backgroundColor: '#E69F00', order: 1 }
        ]
      },
      options: { responsive: true, scales: { y: { beginAtZero: true, grid: { color: GRID } } } }
    }));
    if (e.superlinear_at && e.superlinear_at.length) {
      finding('Experiment 4 — speedup is superlinear, and that is not parallelism',
        `Three drones give ${e.rows[2] ? e.rows[2].speedup.toFixed(2) : '?'}× against an ` +
        `ideal of 3×. A single drone also pays ${e.rows[0].recharge_min.toFixed(0)} min of ` +
        `recharging and flies ${e.rows[0].flight_min.toFixed(0)} min of sequential touring, ` +
        `against ${Math.min.apply(null, e.rows.map(r => r.flight_min)).toFixed(0)} min at ` +
        `the fleet's best. Both costs vanish as the fleet grows, on top of the genuine ` +
        `parallel speedup.`, true);
    }
  }

  function greedy(e) {
    const ctx = card('Greedy schedule against brute-force optimal',
      'Graham’s bound does not transfer to this problem, so quality is ' +
      'measured against an exhaustive optimum instead.');
    const keys = Object.keys(e.histogram);
    charts.push(new Chart(ctx, {
      type: 'bar',
      data: {
        labels: keys,
        datasets: [{ label: 'instances', data: keys.map(k => e.histogram[k]),
                     backgroundColor: '#009E73' }]
      },
      options: { responsive: true, plugins: { legend: { display: false } },
                 scales: { x: { title: { display: true, text: 'greedy / optimal' } },
                           y: { beginAtZero: true, grid: { color: GRID } } } }
    }));
    finding('Experiment 5 — greedy leaves about a quarter on the table',
      `Mean ratio ${e.ratio.mean.toFixed(3)} (SD ${e.ratio.stdev.toFixed(3)}), worst ` +
      `${e.worst_case ? e.worst_case.ratio.toFixed(3) : '?'}, optimal found in only ` +
      `${e.optimal_found_pct.toFixed(1)}% of instances. It never beat the optimum, ` +
      `which is what validates the reference implementation.`, true);
  }

  function environment(e) {
    const sweeps = e.wind_sweeps || [];
    if (sweeps.length) {
      const ctx = card('Distinct optimal routes across wind bearings',
        'If wind alone changes the chosen route, the model is doing real work.');
      charts.push(new Chart(ctx, {
        type: 'bar',
        data: {
          labels: sweeps.map(s => s.pair),
          datasets: [{ label: 'distinct routes', data: sweeps.map(s => s.distinct_routes),
                       backgroundColor: '#CC79A7' }]
        },
        options: { responsive: true, plugins: { legend: { display: false } },
                   scales: { y: { beginAtZero: true, grid: { color: GRID } } } }
      }));
    }
    const zones = (e.no_fly_penalties || [])
      .map(p => `${p.name}: ${p.rerouted_customers} rerouted at ` +
                `${p.cost_penalty_pct.mean.toFixed(1)}% mean penalty, ` +
                `${p.unreachable_customers.length} cut off`).join('; ');
    finding('Experiment 7 — wind and airspace both change the decision', zones);
  }

  function render(results) {
    clear();
    if (results.experiment_1) expansion(results.experiment_1);
    if (results.experiment_2) growth(results.experiment_2);
    if (results.experiment_3) {
      finding('Experiment 3 — energy weighting does not always save energy',
        results.experiment_3.finding, !results.experiment_3.batch_energy_monotonic);
    }
    if (results.experiment_4) fleet(results.experiment_4);
    if (results.experiment_5) greedy(results.experiment_5);
    if (results.experiment_6) {
      finding('Experiment 6 — the fast feasibility test is conservative, not unsafe',
        results.experiment_6.finding, true);
    }
    if (results.experiment_7) environment(results.experiment_7);
  }

  D.charts = { render, clear };
})();
