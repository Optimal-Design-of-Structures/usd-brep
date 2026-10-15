// SPDX-License-Identifier: Apache-2.0
// Contract tests of the narrow API (implementation-independent: any kernel behind brep.mjs must pass them).
// usage: node wasm/test/test-api.mjs [distDir=wasm/dist]
import { readFileSync } from 'node:fs';
import { brotliCompressSync } from 'node:zlib';
import { pathToFileURL } from 'node:url';
import { resolve } from 'node:path';

const opt = 'dist';
const dir = process.argv[2] ? pathToFileURL(resolve(process.argv[2]) + '/') : new URL('../dist/', import.meta.url);
const t0 = performance.now();
const M = await (await import(new URL('brep.mjs', dir))).default();
const tInit = performance.now() - t0;

const fails = [];
const check = (name, cond, extra = '') => { if (!cond) fails.push(name); console.log(`${cond ? 'PASS' : 'FAIL'} ${name} ${extra}`); };
const rel = (a, b) => (a - b) / b;

// --- 1. parity: IPE300 6 m, 20 holes, 2 copes (same as spike.cpp)
const H = 300, B = 150, TW = 7.1, TF = 10.7, R = 15, L = 6000, D = 22, G = 45;
let t = performance.now();
const beam = M.extrudeI(H, B, TW, TF, R, L);
const tSweep = performance.now() - t;
const holes = [];
for (const z of [100, 170, L - 170, L - 100]) for (const x of [-G, G]) holes.push([x, H / 2 - TF - 5, z], [x, -H / 2 - 5, z]);
for (const z of [L / 2 - 40, L / 2 + 40]) for (const x of [-G, G]) holes.push([x, H / 2 - TF - 5, z]);
let holeMs = 0, guardOverheadMs = 0;
t = performance.now();
for (const [x, y, z] of holes) {
  const r = M.hole(beam, x, y, z, 0, 1, 0, D, TF + 10);
  if (!r.ok) fails.push('hole ' + [x, y, z]);
  holeMs += r.ms;
}
const tHoles = performance.now() - t;
const c1 = M.cutBox(beam, -B, H / 2 - 40, L - 50, 2 * B, 50, 60);
const c2 = M.cutBox(beam, -B, -H / 2 - 10, L - 50, 2 * B, 50, 60);
const vol = M.volume(beam, 1e-8);
const A = 2 * B * TF + (H - 2 * TF) * TW + (4 - Math.PI) * R * R;
const aCope = B * TF + (40 - TF) * TW + 2 * (1 - Math.PI / 4) * R * R;
const ref = A * L - holes.length * Math.PI * (D / 2) ** 2 * TF - 2 * aCope * 50;
check('parity: 20 holes + 2 copes ok', holes.length === 20 && c1.ok && c2.ok);
check('parity: volume vs analytic', Math.abs(rel(vol, ref)) < 1e-7, `rel=${rel(vol, ref).toExponential(2)}`);
check('parity: mass kg', true, `${(vol * 7850e-9).toFixed(4)} kg (ref ${(ref * 7850e-9).toFixed(4)})`);
const val = M.validate(beam);
check('validate: manifold', val.manifold, JSON.stringify(val));

// --- 2. rollback: hole tangent to the flange tip (kernel returns success + non-manifold in the spike)
const before = M.validate(beam), vBefore = M.volume(beam, 1e-8);
const tang = M.hole(beam, B / 2 - D / 2, H / 2 - TF - 5, 500, 0, 1, 0, D, TF + 10);
const after = M.validate(beam), vAfter = M.volume(beam, 1e-8);
check('rollback: tangent hole rejected', !tang.ok && tang.rolledBack, JSON.stringify(tang));
check('rollback: part unchanged', after.faces === before.faces && after.manifold && Math.abs(vAfter - vBefore) < 1e-6 * vBefore);

// --- 3. miter via cutPlane on a copy (45 deg about Y at the start of the beam)
const m = M.copy(beam);
const s2 = Math.SQRT1_2;
const mr = M.cutPlane(m, 0, 0, 150, s2, 0, s2); // keep material where x+z > 150 -> removes a wedge at z=0 end
const vm = M.volume(m, 1e-8);
check('cutPlane: miter ok', mr.ok && vm < vol, `${JSON.stringify(mr)} dV=${(vm - vol).toFixed(0)}`);

// --- 4. end plate from polygon, welded (union, face contact) at z = L
const plate = M.extrudePolygon([-100, -170, 100, -170, 100, 170, -100, 170], 20);
M.translate(plate, 0, 0, L);
const vPlate = M.volume(plate, 1e-8);
const u = M.unite(beam, plate);
const vU = M.volume(beam, 1e-8);
// 1e-5: SmApiBrepComputeVolume has body-dependent integration error up to ~3e-6 even at 1e-8 (geometry verified exact by fine mesh)
check('unite: end plate (face contact)', u.ok && Math.abs(vU - (vol + vPlate)) < 1e-5 * vU, `${JSON.stringify(u)} diff=${(vU - vol - vPlate).toFixed(2)} vAfterRollback-vol=${(vAfter - vol).toFixed(3)}`);
check('unite: tool id freed', M.validate(plate) === null);

// --- 5. tessellation + edges
t = performance.now();
const mesh = M.tessellate(beam, 0.1, 15);
const tTess = performance.now() - t;
const ntri = mesh.indices.length / 3;
const badFace = mesh.faceIds.some((f) => f === 0xffffffff);
check('tessellate: typed arrays', mesh.positions instanceof Float32Array && mesh.indices instanceof Uint32Array && mesh.normals.length === mesh.positions.length, `tris=${ntri} verts=${mesh.positions.length / 3}`);
check('tessellate: every triangle mapped to a B-rep face', !badFace && mesh.faceIds.length === ntri, `faces used=${new Set(mesh.faceIds).size}/${M.validate(beam).faces}`);
// mesh volume vs exact (closed mesh => divergence theorem)
let mv = 0; const p = mesh.positions, ix = mesh.indices;
for (let i = 0; i < ix.length; i += 3) {
  const a = ix[i] * 3, b = ix[i + 1] * 3, c = ix[i + 2] * 3;
  mv += (p[a] * (p[b + 1] * p[c + 2] - p[b + 2] * p[c + 1]) - p[a + 1] * (p[b] * p[c + 2] - p[b + 2] * p[c]) + p[a + 2] * (p[b] * p[c + 1] - p[b + 1] * p[c])) / 6;
}
check('tessellate: closed + outward (mesh volume ~ exact)', Math.abs(rel(mv, vU)) < 1e-3, `rel=${rel(mv, vU).toExponential(2)}`);
const ed = M.edges(beam, 10);
check('edges: polylines', ed.counts.length > 0 && ed.counts.reduce((s, c) => s + c, 0) * 3 === ed.positions.length, `polylines=${ed.counts.length} pts=${ed.positions.length / 3}`);

// --- 6. drawings (SmHCR semantics frozen: view vector towards the eye, smooth edges suppressed)
const lens = (g) => { const out = []; let o = 0; for (const n of g.counts) { let L = 0; for (let i = 0; i < n - 1; i++) { const a = (o + i) * 2, c = (o + i + 1) * 2; L += Math.hypot(g.positions[c] - g.positions[a], g.positions[c + 1] - g.positions[a + 1]); } out.push(Math.round(L)); o += n; } return out.sort((x, y) => x - y); };
{
  const pb = M.box(0, 0, 0, 100, 100, 100);
  M.cutBox(pb, 30, 30, 80, 40, 40, 30); // pocket open at the top
  const top = M.drawView(pb, 0, 0, -1, 0, 1, 0, 0.05), below = M.drawView(pb, 0, 0, 1, 0, 1, 0, 0.05);
  const vt = lens(top.visible), vb = lens(below.visible);
  check('drawView: pocket visible from the top', vt.filter((x) => x === 40).length >= 4 && vt.filter((x) => x === 100).length === 4, JSON.stringify(vt));
  check('drawView: pocket hidden from below', vb.every((x) => x === 100) && vb.length === 4, JSON.stringify(vb));
  check('drawView: edges parallel to the view dropped', ![...lens(top.visible), ...lens(top.hidden)].includes(0));
  M.free(pb);
}
{
  const ib = M.extrudeI(300, 150, 7.1, 10.7, 15, 1000);
  M.hole(ib, 45, 134.3, 500, 0, 1, 0, 22, 20.7);
  const front = M.drawView(ib, 1, 0, 0, 0, 1, 0, 0.05);
  // root-fillet tangent lines lie at v = +-124.3 along the whole beam: must not come out as hidden lines
  let fillet = 0, o = 0;
  for (const n of front.hidden.counts) { const v0 = front.hidden.positions[o * 2 + 1], v1 = front.hidden.positions[(o + n - 1) * 2 + 1]; if (Math.abs(Math.abs(v0) - 124.3) < 0.5 && Math.abs(v0 - v1) < 0.5) fillet++; o += n; }
  check('drawView: smooth (fillet tangent) edges not drawn as hidden', fillet === 0 && front.droppedSmooth > 0, `droppedSmooth=${front.droppedSmooth}`);
  const top = M.drawView(ib, 0, -1, 0, 1, 0, 0, 0.05);
  let circ = null; o = 0;
  for (const n of top.visible.counts) { if (n > 8) { const P = top.visible.positions; const cu = [...Array(n - 1).keys()].reduce((s, i) => s + P[(o + i) * 2], 0) / (n - 1), cv = [...Array(n - 1).keys()].reduce((s, i) => s + P[(o + i) * 2 + 1], 0) / (n - 1); circ = { u: cu, v: cv, r: Math.hypot(P[o * 2] - cu, P[o * 2 + 1] - cv) }; } o += n; }
  check('drawView: hole is an exact circle at its feature position', circ && Math.abs(circ.r - 11) < 1e-3 && Math.abs(circ.u - 500) < 1e-3 && Math.abs(circ.v - 45) < 1e-3, JSON.stringify(circ));
  M.free(ib);
}

// --- 7. lifecycle
[beam, m].forEach((id) => M.free(id));
check('free: registry empty', M.count() === 0, `count=${M.count()}`);

const wasm = readFileSync(new URL('brep.wasm', dir));
console.log(JSON.stringify({
  opt, init_ms: +tInit.toFixed(1), sweep_ms: +tSweep.toFixed(1),
  holes_wall_ms: +tHoles.toFixed(1), holes_guarded_sum_ms: +holeMs.toFixed(1),
  copes_ms: +(c1.ms + c2.ms).toFixed(1), tangent_rollback_ms: +tang.ms.toFixed(1),
  tess_ms: +tTess.toFixed(1), tris: ntri, wasm_memory_mb: M.wasmMemoryMB(),
  wasm_mb: +(wasm.length / 1e6).toFixed(2), wasm_br_mb: +(brotliCompressSync(wasm).length / 1e6).toFixed(2),
}, null, 1));
if (fails.length) { console.log('FAILED:', fails); process.exit(1); }
