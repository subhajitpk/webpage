const canvas = document.getElementById('view');
    const ctx = canvas.getContext('2d');
    const stepBtn = document.getElementById('stepBtn');
    const runBtn = document.getElementById('runBtn');
    const resetBtn = document.getElementById('resetBtn');
    const randomBtn = document.getElementById('randomBtn');
    const cornerSelect = document.getElementById('cornerSelect');
    const statusEl = document.getElementById('status');

    const EPS = 1e-9;
    const W = canvas.width;
    const H = canvas.height;

    let robots = [];
    let hull = [];
    let selected = 0;
    let state = null;
    let running = false;
    let runTimer = null;

    const stages = [
      'Initial',
      'Hull + neighbors',
      'Reduced triangle',
      'Half-plane pruning',
      'Final visible region'
    ];

    function v(x, y) { return { x, y }; }
    function add(a, b) { return v(a.x + b.x, a.y + b.y); }
    function sub(a, b) { return v(a.x - b.x, a.y - b.y); }
    function mul(a, t) { return v(a.x * t, a.y * t); }
    function dot(a, b) { return a.x * b.x + a.y * b.y; }
    function cross2(a, b) { return a.x * b.y - a.y * b.x; }
    function cross(a, b, c) { return cross2(sub(b, a), sub(c, a)); }
    function dist(a, b) { return Math.hypot(a.x - b.x, a.y - b.y); }

    function lineIntersection(p1, p2, p3, p4) {
      const r = sub(p2, p1);
      const s = sub(p4, p3);
      const d = cross2(r, s);
      if (Math.abs(d) < EPS) return null;
      const t = cross2(sub(p3, p1), s) / d;
      return add(p1, mul(r, t));
    }

    function pointInTriangleStrict(p, a, b, c) {
      const c1 = cross(a, b, p);
      const c2 = cross(b, c, p);
      const c3 = cross(c, a, p);
      const hasNeg = (c1 < -EPS) || (c2 < -EPS) || (c3 < -EPS);
      const hasPos = (c1 > EPS) || (c2 > EPS) || (c3 > EPS);
      if (!(hasNeg && hasPos)) {
        return Math.abs(c1) > EPS && Math.abs(c2) > EPS && Math.abs(c3) > EPS;
      }
      return false;
    }

    function convexHull(points) {
      if (points.length <= 1) return points.slice();
      const pts = points.map((p, i) => ({ ...p, _i: i }))
        .sort((a, b) => a.x === b.x ? a.y - b.y : a.x - b.x);
      const lower = [];
      for (const p of pts) {
        while (lower.length >= 2 && cross(lower[lower.length - 2], lower[lower.length - 1], p) <= EPS) lower.pop();
        lower.push(p);
      }
      const upper = [];
      for (let i = pts.length - 1; i >= 0; i--) {
        const p = pts[i];
        while (upper.length >= 2 && cross(upper[upper.length - 2], upper[upper.length - 1], p) <= EPS) upper.pop();
        upper.push(p);
      }
      upper.pop();
      lower.pop();
      return lower.concat(upper).map(({ _i, ...rest }) => rest);
    }

    function polygonCentroid(poly) {
      let A = 0, Cx = 0, Cy = 0;
      for (let i = 0; i < poly.length; i++) {
        const p = poly[i], q = poly[(i + 1) % poly.length];
        const k = p.x * q.y - q.x * p.y;
        A += k;
        Cx += (p.x + q.x) * k;
        Cy += (p.y + q.y) * k;
      }
      A *= 0.5;
      if (Math.abs(A) < EPS) return v(0, 0);
      return v(Cx / (6 * A), Cy / (6 * A));
    }

    function clipByLineKeepR(poly, linePoint, lineDir, r) {
      if (poly.length < 3) return [];
      const n = v(-lineDir.y, lineDir.x);
      const sideR = dot(sub(r, linePoint), n);
      const out = [];

      function side(p) { return dot(sub(p, linePoint), n); }
      function inside(s) { return s * sideR >= -EPS; }

      for (let i = 0; i < poly.length; i++) {
        const A = poly[i], B = poly[(i + 1) % poly.length];
        const sA = side(A), sB = side(B);
        const inA = inside(sA), inB = inside(sB);

        if (inA) out.push(A);

        if (inA !== inB) {
          const I = lineIntersection(A, B, linePoint, add(linePoint, lineDir));
          if (I) out.push(I);
        }
      }

      if (out.length < 3) return [];
      const cleaned = [];
      for (let i = 0; i < out.length; i++) {
        const p = out[i], q = out[(i + 1) % out.length];
        if (dist(p, q) > 1e-6) cleaned.push(p);
      }
      return cleaned;
    }

    function initDefault() {
      robots = [
        v(210, 565), v(508, 100), v(850, 565),
        v(360, 360), v(430, 410), v(520, 440),
        v(580, 405), v(490, 312), v(630, 330),
        v(730, 420), v(305, 470)
      ];
      reset();
    }

    function initRandom() {
      const n = 11;
      robots = [];
      for (let i = 0; i < n; i++) {
        robots.push(v(
          120 + Math.random() * (W - 240),
          90 + Math.random() * (H - 160)
        ));
      }
      reset();
    }

    function rebuildHullAndSelector() {
      hull = convexHull(robots);
      cornerSelect.innerHTML = '';
      hull.forEach((p, idx) => {
        const opt = document.createElement('option');
        opt.value = String(idx);
        opt.textContent = `hull corner ${idx + 1} (${Math.round(p.x)}, ${Math.round(p.y)})`;
        cornerSelect.appendChild(opt);
      });
      selected = Math.min(selected, hull.length - 1);
      cornerSelect.value = String(selected);
    }

    function reset() {
      running = false;
      if (runTimer) clearInterval(runTimer);
      rebuildHullAndSelector();
      state = {
        stage: 0,
        r: hull[selected],
        a: null,
        b: null,
        u: null,
        w: null,
        c: null,
        y: null,
        z: null,
        visiblePoly: [],
        pairs: [],
        pairIndex: 0
      };
      render();
    }

    function computeStep() {
      if (!state) return;

      if (state.stage === 0) {
        const i = selected;
        const n = hull.length;
        state.r = hull[i];
        state.a = hull[(i + 1) % n];     // ccw neighbor
        state.b = hull[(i - 1 + n) % n]; // cw neighbor
        state.u = mul(add(state.r, state.a), 0.5);
        state.w = mul(add(state.r, state.b), 0.5);
        state.stage = 1;
      } else if (state.stage === 1) {
        const inside = robots.filter(p => p !== state.r && pointInTriangleStrict(p, state.r, state.u, state.w));
        if (inside.length === 0) {
          state.visiblePoly = [state.r, state.u, state.w];
        } else {
          const uw = sub(state.w, state.u);
          const nrm = v(-uw.y, uw.x);
          let best = inside[0], bestDist = Infinity;
          for (const p of inside) {
            const d = Math.abs(dot(sub(p, state.r), nrm)) / (Math.hypot(nrm.x, nrm.y) + EPS);
            if (d < bestDist) {
              bestDist = d;
              best = p;
            }
          }
          state.c = best;
          const L1 = add(best, uw);
          state.y = lineIntersection(state.r, state.a, best, L1);
          state.z = lineIntersection(state.r, state.b, best, L1);
          state.visiblePoly = (state.y && state.z) ? [state.r, state.y, state.z] : [state.r, state.u, state.w];
        }

        const C = robots.filter(p => p !== state.r);
        for (let i = 0; i < C.length; i++) {
          for (let j = 0; j < C.length; j++) {
            if (i === j) continue;
            state.pairs.push([C[i], C[j]]);
          }
        }
        state.stage = 2;
      } else if (state.stage === 2) {
        if (state.pairIndex < state.pairs.length) {
          const [cp, cq] = state.pairs[state.pairIndex];
          const d = sub(cq, state.r);
          if (Math.hypot(d.x, d.y) > EPS) {
            state.visiblePoly = clipByLineKeepR(state.visiblePoly, cp, d, state.r);
          }
          state.pairIndex++;
        } else {
          state.stage = 3;
        }
      } else if (state.stage === 3) {
        state.stage = 4;
      }

      render();
    }

    function drawPolygon(poly, fill, stroke, lw = 2) {
      if (!poly || poly.length < 3) return;
      ctx.beginPath();
      ctx.moveTo(poly[0].x, poly[0].y);
      for (let i = 1; i < poly.length; i++) ctx.lineTo(poly[i].x, poly[i].y);
      ctx.closePath();
      if (fill) { ctx.fillStyle = fill; ctx.fill(); }
      if (stroke) { ctx.strokeStyle = stroke; ctx.lineWidth = lw; ctx.stroke(); }
    }

    function drawPoint(p, color, radius = 5) {
      ctx.beginPath();
      ctx.arc(p.x, p.y, radius, 0, Math.PI * 2);
      ctx.fillStyle = color;
      ctx.fill();
    }

    function render() {
      ctx.clearRect(0, 0, W, H);

      drawPolygon(hull, 'rgba(0,0,0,0)', '#192230', 1.5);

      if (state && state.stage >= 1) {
        drawPolygon([state.r, state.u, state.w], 'rgba(37,130,255,0.14)', '#2582ff', 2);
      }

      if (state && state.stage >= 2) {
        drawPolygon(state.visiblePoly, 'rgba(12,193,95,0.22)', '#0dbb63', 2.5);
      }

      robots.forEach(p => drawPoint(p, '#2f68ff', 5));

      if (state && state.r) {
        drawPoint(state.r, '#ff3c3c', 7);
      }
      if (state && state.a) drawPoint(state.a, '#7d4dff', 6);
      if (state && state.b) drawPoint(state.b, '#7d4dff', 6);
      if (state && state.c) drawPoint(state.c, '#ff8a00', 6);

      if (state && state.visiblePoly.length >= 3) {
        const cc = polygonCentroid(state.visiblePoly);
        ctx.fillStyle = '#102a43';
        ctx.font = 'bold 14px Inter, Arial, sans-serif';
        ctx.fillText('A_visible', cc.x + 6, cc.y);
      }

      const st = state || { stage: 0 };
      if (st.stage <= 1) {
        statusEl.textContent = `${stages[st.stage]} (press Step)`;
      } else if (st.stage === 2) {
        statusEl.textContent = `Half-plane pruning ${st.pairIndex}/${st.pairs.length}`;
      } else {
        statusEl.textContent = stages[Math.min(st.stage, stages.length - 1)];
      }
    }

    stepBtn.addEventListener('click', computeStep);
    runBtn.addEventListener('click', () => {
      running = !running;
      runBtn.textContent = running ? 'Pause' : 'Run';
      if (runTimer) clearInterval(runTimer);
      if (running) {
        runTimer = setInterval(() => {
          computeStep();
          if (state.stage >= 4) {
            running = false;
            runBtn.textContent = 'Run';
            clearInterval(runTimer);
          }
        }, 80);
      }
    });

    resetBtn.addEventListener('click', reset);
    randomBtn.addEventListener('click', initRandom);
    cornerSelect.addEventListener('change', () => {
      selected = Number(cornerSelect.value);
      reset();
    });

    initDefault();
