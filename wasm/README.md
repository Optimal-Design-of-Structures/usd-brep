# USD BRep → WebAssembly (iQSteel)

This branch (`iqsteel-wasm`) of the fork builds the **unmodified** USD BRep (SMLib) kernel together with a narrow
JavaScript API into an ES module for browsers, Web Workers and Node. It is the geometry kernel of the iQSteel steel
detailing module (decision D193 in the ODS vault).

## Rules

- **Zero patches in the kernel.** Everything outside `wasm/` and `.github/` must be byte-identical to the upstream
  commit pinned in [`UPSTREAM`](UPSTREAM). CI enforces it. The first patch we would need is a signal to re-evaluate
  the decision, not an invitation to maintain a CAD kernel.
- **Kernel access only through the API** in [`brep_api.cpp`](brep_api.cpp). The contract tests in
  [`test/test-api.mjs`](test/test-api.mjs) are implementation-independent, so any replacement kernel behind the same
  API must pass them.
- **Every mutating operation is validated** (manifold check) and rolled back to a snapshot on failure, because the
  kernel can report success with a non-manifold result in tangent cases.

## API

| Function | Purpose |
|---|---|
| `extrudeI(h, b, tw, tf, r, L)`, `extrudePolygon([x,y,…], L)`, `box(x,y,z,dx,dy,dz)` | construction (I section with root radius, plate / any polygon) |
| `hole(id, px,py,pz, dx,dy,dz, Ø, depth)` | cylindrical hole in any direction |
| `cutBox(id, …)`, `cutPlane(id, p, n)` | box cut; plane cut keeping the side `n` points to (implemented as a half-space boolean: `SmApiCut` never yields a manifold solid in 1.0.0) |
| `unite(a, b)`, `subtract(a, b)` | booleans between parts (`b` is freed on success) |
| `copy`, `free`, `count`, `translate`, `rotate` | lifecycle and placement |
| `volume(id, relAcc)`, `validate(id)`, `bbox(id)` | queries (use `relAcc = 1e-8`; body-dependent integration error up to ~3e-6) |
| `tessellate(id, chord, angleDeg)` | `positions`, `normals` (sharp across B-rep edges), `indices`, `faceIds` (B-rep face per triangle) |
| `edges(id, angleDeg)` | B-rep edges as polylines |
| `drawView(id, dir, up, chord)` | exact hidden-line removal (kernel `SmHCR`): 2D polylines `visible` / `hidden`; seams and smooth (G1) edges suppressed |

Mutating ops return `{ status, ok, rolledBack, ms }`.

## Build

```bash
source /path/to/emsdk/emsdk_env.sh   # emsdk 6.0.12
python3 wasm/build_wasm.py --opt Oz  # -> wasm/dist/brep.mjs, brep.wasm, build-info.json, LICENSE, NOTICE
node wasm/test/test-api.mjs wasm/dist
```

On Windows, set `EMXX` to `…\emsdk\upstream\emscripten\em++.exe` instead of activating emsdk.

## Releases

Pushing a tag `brep-wasm-vMAJOR.MINOR.PATCH` publishes a GitHub release with `brep.mjs`, `brep.wasm`,
`build-info.json` and a zip including `LICENSE` and `NOTICE`. The version follows the **API** (semver), not the
kernel; `build-info.json` records the kernel commit, the fork commit and the toolchain.

## Updating the kernel

1. `git fetch upstream && git merge <upstream tag>` into `iqsteel-wasm`.
2. Update `UPSTREAM` with the new commit.
3. CI must pass: pristine check and contract tests. A failing contract test is a kernel behaviour change and blocks
   the update.
