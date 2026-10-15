// SPDX-License-Identifier: Apache-2.0
// iQSteel narrow JS API over the USD BRep (SMLib) kernel. Kernel sources in ../source are used unmodified (see UPSTREAM).
// Narrow JS API over the SMLib (USD BRep) kernel — embind.
// Breps live in a WASM-side registry; JS holds integer ids. Every mutating op is validated
// (manifold check) and rolled back to a snapshot on failure, because the kernel can report
// success with a non-manifold result (tangent cases — see spike README).
#include <SmApiBrep.h>
#include <SmApiCurves.h>
#include <SmApiGeneral.h>
#include <SmApiPrimitives.h>
#include <SmApiQueries.h>
#include <SmBrep.h>
#include <SmBSplineCurve.h>
#include <SmEdge.h>
#include <SmFace.h>
#include <SmHCR.h>
#include <SmLine.h>
#include <SmPoly.h>
#include <SmVector3d.h>

#include <emscripten/bind.h>
#include <emscripten/val.h>

#include <chrono>
#include <cmath>
#include <unordered_map>
#include <vector>

using namespace emscripten;

namespace
{
std::unordered_map<int, SmBrep*> g_breps;
int g_nextId = 1;
bool g_init = false;

void ensureContext()
{
    if (!g_init) { SmApiCreateContext(); g_init = true; }
}

int put(SmBrep* b)
{
    if (!b) return 0;
    g_breps[g_nextId] = b;
    return g_nextId++;
}

SmBrep* get(int id)
{
    auto it = g_breps.find(id);
    return it == g_breps.end() ? nullptr : it->second;
}

double nowMs() { return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now().time_since_epoch()).count(); }

bool isManifold(SmBrep* b)
{
    SmBoolean m = FALSE;
    return SmApiBrepIsManifoldSolid(b, m) == SM_SUCCESS && m == TRUE;
}

// Result of a mutating op: status = kernel status (1000 = SM_SUCCESS), ok = success AND manifold.
// rolledBack = the op produced an invalid solid and the part was restored from the snapshot.
struct OpResult
{
    int status = 0;
    bool ok = false;
    bool rolledBack = false;
    double ms = 0;
};

// Runs fn(brep) -> status with snapshot/rollback. fn may consume/alias the brep pointer via the out param.
template <class Fn> OpResult guarded(int id, Fn fn)
{
    OpResult r;
    SmBrep* b = get(id);
    if (!b) { r.status = -1; return r; }
    const double t0 = nowMs();
    SmBrep* snapshot = nullptr;
    SmApiBrepCopy(b, snapshot);
    SmApiStatus s;
    try { s = fn(b); }
    catch (...) { s = SM_ERR_INVALID_INPUT; }
    r.status = (int)s;
    r.ok = s == SM_SUCCESS && isManifold(b);
    if (!r.ok && snapshot)
    {
        delete b; // on failure the kernel leaves the operand allocated (possibly modified)
        g_breps[id] = snapshot;
        snapshot = nullptr;
        r.rolledBack = true;
    }
    delete snapshot;
    r.ms = nowMs() - t0;
    return r;
}

// Rotation taking +Z onto unit direction d (axis-angle), applied about the origin.
void orientZTo(SmBrep* b, double dx, double dy, double dz)
{
    const double len = std::sqrt(dx * dx + dy * dy + dz * dz);
    dx /= len; dy /= len; dz /= len;
    const double c = dz; // dot(Z, d)
    if (c > 1 - 1e-12) return;
    if (c < -1 + 1e-12) { SmApiRotate(b, SmVector3d(0, 0, 0), SmVector3d(1, 0, 0), 180.0); return; }
    SmVector3d axis(-dy, dx, 0); // Z x d
    SmApiRotate(b, SmVector3d(0, 0, 0), axis, std::acos(c) * 180.0 / M_PI);
}

SmBrep* sweepClosedCurves(SmTArray<SmCurve*>& c, double length)
{
    SmBrep* b = nullptr;
    SmApiStatus s = SmApiCreateLinearSweep(c, SmVector3d(0, 0, 1), length, TRUE, b);
    for (ULONG i = 0; i < c.GetSize(); ++i) delete c[i];
    return s == SM_SUCCESS ? b : nullptr;
}
} // namespace

// ---- construction --------------------------------------------------------------------------

// I/H section (EN 10365 style, root radius r) in XY, centred, extruded along +Z.
int extrudeI(double h, double b, double tw, double tf, double r, double length)
{
    ensureContext();
    const double hb = b / 2, hh = h / 2, hw = tw / 2, yf = hh - tf, xr = hw + r, yr = yf - r;
    SmTArray<SmCurve*> c;
    auto line = [&](double x1, double y1, double x2, double y2) {
        SmLine* l = nullptr; SmApiCreateLineSegment(SmPoint3d(x1, y1, 0), SmPoint3d(x2, y2, 0), l); c.Add(l);
    };
    auto arc = [&](double cx, double cy, double a0, double a1) {
        SmBSplineCurve* a = nullptr; SmApiCreateArc(SmPoint3d(cx, cy, 0), r, a0, a1, a); c.Add(a);
    };
    line(-hb, -hh, hb, -hh); line(hb, -hh, hb, -yf); line(hb, -yf, xr, -yf);
    if (r > 0) arc(xr, -yr, 180, 270);
    line(hw, -yr, hw, yr);
    if (r > 0) arc(xr, yr, 90, 180);
    line(xr, yf, hb, yf); line(hb, yf, hb, hh); line(hb, hh, -hb, hh); line(-hb, hh, -hb, yf); line(-hb, yf, -xr, yf);
    if (r > 0) arc(-xr, yr, 0, 90);
    line(-hw, yr, -hw, -yr);
    if (r > 0) arc(-xr, -yr, 270, 360);
    line(-xr, -yf, -hb, -yf); line(-hb, -yf, -hb, -hh);
    return put(sweepClosedCurves(c, length));
}

// Closed polygon [x0,y0,x1,y1,...] in XY extruded along +Z (plates, custom profiles).
int extrudePolygon(val xy, double length)
{
    ensureContext();
    std::vector<double> p = convertJSArrayToNumberVector<double>(xy);
    const size_t n = p.size() / 2;
    if (n < 3) return 0;
    SmTArray<SmCurve*> c;
    for (size_t i = 0; i < n; ++i)
    {
        const size_t j = (i + 1) % n;
        SmLine* l = nullptr;
        SmApiCreateLineSegment(SmPoint3d(p[2 * i], p[2 * i + 1], 0), SmPoint3d(p[2 * j], p[2 * j + 1], 0), l);
        c.Add(l);
    }
    return put(sweepClosedCurves(c, length));
}

// Closed outline in XY built from line and arc segments, extruded along +Z (exact arcs: tube corners,
// root and toe radii, circles). segs = [{t:'L', a:[x0,y0,x1,y1]}, {t:'A', a:[cx,cy,r,startDeg,endDeg]}, ...];
// consecutive segments must meet (within modelling tolerance). Hollow sections = outer minus inner (subtract).
int extrudeOutline(val segs, double length)
{
    ensureContext();
    const unsigned n = segs["length"].as<unsigned>();
    if (n < 2) return 0;
    SmTArray<SmCurve*> c;
    bool bad = false;
    for (unsigned i = 0; i < n && !bad; ++i)
    {
        val s = segs[i];
        const std::string t = s["t"].as<std::string>();
        std::vector<double> a = convertJSArrayToNumberVector<double>(s["a"]);
        if (t == "L" && a.size() == 4)
        {
            SmLine* l = nullptr;
            bad = SmApiCreateLineSegment(SmPoint3d(a[0], a[1], 0), SmPoint3d(a[2], a[3], 0), l) != SM_SUCCESS;
            if (l) c.Add(l);
        }
        else if (t == "A" && a.size() == 5)
        {
            SmBSplineCurve* arc = nullptr;
            bad = SmApiCreateArc(SmPoint3d(a[0], a[1], 0), a[2], a[3], a[4], arc) != SM_SUCCESS;
            if (arc) c.Add(arc);
        }
        else bad = true;
    }
    if (bad)
    {
        for (ULONG i = 0; i < c.GetSize(); ++i) delete c[i];
        return 0;
    }
    return put(sweepClosedCurves(c, length));
}

int box(double x, double y, double z, double dx, double dy, double dz)
{
    ensureContext();
    SmBrep* b = nullptr;
    return SmApiCreateBox(SmVector3d(x, y, z), dx, dy, dz, b) == SM_SUCCESS ? put(b) : 0;
}

int copy(int id)
{
    SmBrep* b = get(id);
    SmBrep* c = nullptr;
    return b && SmApiBrepCopy(b, c) == SM_SUCCESS ? put(c) : 0;
}

void freeBrep(int id)
{
    auto it = g_breps.find(id);
    if (it != g_breps.end()) { delete it->second; g_breps.erase(it); }
}

int count() { return (int)g_breps.size(); }

bool translate(int id, double x, double y, double z)
{
    SmBrep* b = get(id);
    return b && SmApiTranslate(b, SmVector3d(x, y, z)) == SM_SUCCESS;
}

bool rotate(int id, double px, double py, double pz, double ax, double ay, double az, double deg)
{
    SmBrep* b = get(id);
    return b && SmApiRotate(b, SmVector3d(px, py, pz), SmVector3d(ax, ay, az), deg) == SM_SUCCESS;
}

// ---- features (validated, rolled back on failure) ------------------------------------------

// Cylindrical hole: base centre p, axis direction d, length depth (make it longer than the material).
OpResult hole(int id, double px, double py, double pz, double dx, double dy, double dz, double diameter, double depth)
{
    return guarded(id, [&](SmBrep*& b) -> SmApiStatus {
        SmBrep* cyl = nullptr;
        SmApiStatus s = SmApiCreateCylinder(SmVector3d(0, 0, 0), diameter / 2, depth, cyl);
        if (s != SM_SUCCESS) return s;
        orientZTo(cyl, dx, dy, dz);
        SmApiTranslate(cyl, SmVector3d(px, py, pz));
        SmBrep* res = nullptr;
        s = SmApiBooleanDifference(b, cyl, res);
        if (s != SM_SUCCESS) delete cyl;
        return s;
    });
}

OpResult cutBox(int id, double x, double y, double z, double dx, double dy, double dz)
{
    return guarded(id, [&](SmBrep*& b) -> SmApiStatus {
        SmBrep* tool = nullptr;
        SmApiStatus s = SmApiCreateBox(SmVector3d(x, y, z), dx, dy, dz, tool);
        if (s != SM_SUCCESS) return s;
        SmBrep* res = nullptr;
        s = SmApiBooleanDifference(b, tool, res);
        if (s != SM_SUCCESS) delete tool;
        return s;
    });
}

// Plane cut (end cut, miter): keeps material on the side the normal points to.
// Not SmApiCut: in this build it never yields a manifold solid (not even on a box), so the cut is a boolean
// difference with a large box filling the negative half-space, centred on the part's projection onto the plane.
OpResult cutPlane(int id, double px, double py, double pz, double nx, double ny, double nz)
{
    return guarded(id, [&](SmBrep*& b) -> SmApiStatus {
        const double nl = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (!(nl > 0)) return SM_ERR_INVALID_INPUT;
        nx /= nl; ny /= nl; nz /= nl;
        SmPoint3d lo, hi;
        SmApiStatus s = SmApiBrepBoundingBox(b, FALSE, lo, hi);
        if (s != SM_SUCCESS) return s;
        const double cx = (lo.x + hi.x) / 2, cy = (lo.y + hi.y) / 2, cz = (lo.z + hi.z) / 2;
        const double diag = std::sqrt((hi.x - lo.x) * (hi.x - lo.x) + (hi.y - lo.y) * (hi.y - lo.y) + (hi.z - lo.z) * (hi.z - lo.z));
        const double S = 4 * diag + 1;
        const double d = (cx - px) * nx + (cy - py) * ny + (cz - pz) * nz; // signed distance of centre to plane
        SmBrep* tool = nullptr;
        s = SmApiCreateBox(SmVector3d(-S / 2, -S / 2, -S), S, S, S, tool); // local: z in [-S, 0] = negative side
        if (s != SM_SUCCESS) return s;
        orientZTo(tool, nx, ny, nz);
        SmApiTranslate(tool, SmVector3d(cx - d * nx, cy - d * ny, cz - d * nz));
        SmBrep* res = nullptr;
        s = SmApiBooleanDifference(b, tool, res);
        if (s != SM_SUCCESS) delete tool;
        return s;
    });
}

static OpResult booleanWith(int idA, int idB, SmBooleanOperationType op)
{
    SmBrep* tool = get(idB);
    if (!tool || idA == idB) { OpResult r; r.status = -1; return r; }
    SmBrep* toolCopy = nullptr; // keep B intact on rollback: operate on a copy, free B only on success
    SmApiBrepCopy(tool, toolCopy);
    OpResult r = guarded(idA, [&](SmBrep*& b) -> SmApiStatus {
        SmBrep* res = nullptr;
        SmApiStatus s = SmApiBoolean(b, toolCopy, op, res);
        if (s == SM_SUCCESS) toolCopy = nullptr; // consumed by the kernel
        return s;
    });
    delete toolCopy;
    if (r.ok) freeBrep(idB);
    return r;
}

OpResult unite(int idA, int idB) { return booleanWith(idA, idB, SM_BO_UNION); }
OpResult subtract(int idA, int idB) { return booleanWith(idA, idB, SM_BO_DIFFERENCE); }

// ---- queries --------------------------------------------------------------------------------

double volume(int id, double relAccuracy)
{
    SmBrep* b = get(id);
    double v = NAN;
    if (!b || SmApiBrepComputeVolume(b, relAccuracy, v) != SM_SUCCESS) return NAN;
    return v;
}

val validate(int id)
{
    val o = val::object();
    SmBrep* b = get(id);
    if (!b) return val::null();
    SmTArray<SmFace*> f; SmTArray<SmEdge*> e;
    SmApiGetFaces(b, f); SmApiGetEdges(b, e);
    o.set("manifold", isManifold(b));
    o.set("faces", (int)f.GetSize());
    o.set("edges", (int)e.GetSize());
    return o;
}

val bbox(int id)
{
    SmBrep* b = get(id);
    if (!b) return val::null();
    SmPoint3d lo, hi;
    if (SmApiBrepBoundingBox(b, FALSE, lo, hi) != SM_SUCCESS) return val::null();
    val a = val::array();
    for (double d : { lo.x, lo.y, lo.z, hi.x, hi.y, hi.z }) a.call<void>("push", d);
    return a;
}

template <class T> val toTyped(const char* ctor, const std::vector<T>& v)
{
    // new Float32Array(view) copies out of the WASM heap, so the JS object owns its data
    return val::global(ctor).new_(typed_memory_view(v.size(), v.data()));
}

// Triangle mesh: vertices split per (mesh vertex, B-rep face) so normals are smooth inside a face and sharp
// across edges. faceIds[t] = index of the B-rep face that triangle t came from (picking / face selection).
val tessellate(int id, double chordTol, double angleTolDeg)
{
    SmBrep* b = get(id);
    if (!b) return val::null();
    SmTessellationParams tp;
    tp.dChordHeightTol = chordTol;
    tp.dCurveAngleTolDeg = tp.dSurfaceAngleTolDeg = angleTolDeg;
    SmPolyBrep* mesh = nullptr;
    const double t0 = nowMs();
    SmApiStatus s = SmApiTessellate(b, mesh, tp);
    if (s != SM_SUCCESS || !mesh) { delete mesh; return val::null(); }

    SmTArray<SmFace*> faces;
    SmApiGetFaces(b, faces);
    std::unordered_map<const void*, uint32_t> faceIndex;
    for (ULONG i = 0; i < faces.GetSize(); ++i) faceIndex[faces[i]] = i;

    struct Key { const void* v; const void* f; bool operator==(const Key& o) const { return v == o.v && f == o.f; } };
    struct KeyHash { size_t operator()(const Key& k) const { return std::hash<const void*>()(k.v) * 31 ^ std::hash<const void*>()(k.f); } };
    std::unordered_map<Key, uint32_t, KeyHash> vid;
    std::vector<float> pos, nrm;
    std::vector<double> nacc;
    std::vector<uint32_t> idx, fid;

    SmTArray<SmPolyFace*> pf;
    mesh->GetPolyFaces(pf);
    for (ULONG i = 0; i < pf.GetSize(); ++i)
    {
        SmPolyLoop* lp = pf[i]->GetOuterPolyLoop();
        if (!lp) continue;
        SmTArray<SmPolyEdge*> pe;
        lp->GetPolyEdges(pe);
        const ULONG n = pe.GetSize();
        if (n < 3) continue;
        const void* orig = pf[i]->GetOriginalFace();
        std::vector<uint32_t> loop(n);
        std::vector<SmPoint3d> pts(n);
        for (ULONG j = 0; j < n; ++j)
        {
            SmPolyVertex* v = pe[j]->GetStartPolyVertex();
            pts[j] = v->GetPoint();
            Key k{ v, orig };
            auto it = vid.find(k);
            if (it == vid.end())
            {
                const uint32_t ni = (uint32_t)(pos.size() / 3);
                pos.insert(pos.end(), { (float)pts[j].x, (float)pts[j].y, (float)pts[j].z });
                nacc.insert(nacc.end(), { 0.0, 0.0, 0.0 });
                it = vid.emplace(k, ni).first;
            }
            loop[j] = it->second;
        }
        const uint32_t faceId = faceIndex.count(orig) ? faceIndex[orig] : 0xffffffffu;
        for (ULONG j = 1; j + 1 < n; ++j) // fan (tessellator emits triangles; fan covers convex quads too)
        {
            const SmPoint3d &a = pts[0], &bb = pts[j], &cc = pts[j + 1];
            const double ux = bb.x - a.x, uy = bb.y - a.y, uz = bb.z - a.z, vx = cc.x - a.x, vy = cc.y - a.y, vz = cc.z - a.z;
            const double nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx; // area-weighted
            for (uint32_t k : { loop[0], loop[j], loop[j + 1] })
            {
                idx.push_back(k);
                nacc[3 * k] += nx; nacc[3 * k + 1] += ny; nacc[3 * k + 2] += nz;
            }
            fid.push_back(faceId);
        }
    }
    nrm.resize(pos.size());
    for (size_t k = 0; k < nacc.size(); k += 3)
    {
        const double l = std::sqrt(nacc[k] * nacc[k] + nacc[k + 1] * nacc[k + 1] + nacc[k + 2] * nacc[k + 2]);
        const double inv = l > 0 ? 1.0 / l : 0.0;
        nrm[k] = (float)(nacc[k] * inv); nrm[k + 1] = (float)(nacc[k + 1] * inv); nrm[k + 2] = (float)(nacc[k + 2] * inv);
    }
    delete mesh;

    val o = val::object();
    o.set("positions", toTyped("Float32Array", pos));
    o.set("normals", toTyped("Float32Array", nrm));
    o.set("indices", toTyped("Uint32Array", idx));
    o.set("faceIds", toTyped("Uint32Array", fid));
    o.set("ms", nowMs() - t0);
    return o;
}

// B-rep edges sampled as polylines: positions = flat xyz, counts[i] = points in polyline i.
val edges(int id, double angleTolDeg)
{
    SmBrep* b = get(id);
    if (!b) return val::null();
    SmTArray<SmPoint3d> pts;
    SmTArray<ULONG> counts;
    SmTArray<SmEdge*> es;
    SmApiTessellateBoundaries(b, angleTolDeg, pts, counts, es);
    std::vector<float> pos;
    pos.reserve(pts.GetSize() * 3);
    for (ULONG i = 0; i < pts.GetSize(); ++i) pos.insert(pos.end(), { (float)pts[i].x, (float)pts[i].y, (float)pts[i].z });
    std::vector<uint32_t> cnt(counts.GetSize());
    for (ULONG i = 0; i < counts.GetSize(); ++i) cnt[i] = (uint32_t)counts[i];
    val o = val::object();
    o.set("positions", toTyped("Float32Array", pos));
    o.set("counts", toTyped("Uint32Array", cnt));
    return o;
}

// ---- drawings: exact hidden-line removal (kernel SmHCR) -------------------------------------

namespace
{
// Samples a curve over its natural interval: lines -> 2 points, others -> chord-limited polyline.
void sampleCurve(const SmCurve* c, double chordTol, std::vector<SmPoint3d>& out)
{
    out.clear();
    const SmExtent1d iv = c->GetNaturalInterval();
    const double t0 = iv.GetMin(), t1 = iv.GetMax();
    SmVector3d pv[1];
    auto at = [&](double t) { c->Evaluate(t, 0, FALSE, pv); return SmPoint3d(pv[0].x, pv[0].y, pv[0].z); };
    if (dynamic_cast<const SmLine*>(c)) { out.push_back(at(t0)); out.push_back(at(t1)); return; }
    // adaptive bisection on chord deviation, capped depth
    std::vector<double> ts{ t0, t1 };
    for (int depth = 0; depth < 8; ++depth)
    {
        std::vector<double> next{ ts[0] };
        bool split = false;
        for (size_t i = 0; i + 1 < ts.size(); ++i)
        {
            const double tm = (ts[i] + ts[i + 1]) / 2;
            const SmPoint3d a = at(ts[i]), b = at(ts[i + 1]), m = at(tm);
            const double dx = (a.x + b.x) / 2 - m.x, dy = (a.y + b.y) / 2 - m.y, dz = (a.z + b.z) / 2 - m.z;
            if (depth < 2 || dx * dx + dy * dy + dz * dz > chordTol * chordTol) { next.push_back(tm); split = true; }
            next.push_back(ts[i + 1]);
        }
        ts.swap(next);
        if (!split) break;
    }
    for (double t : ts) out.push_back(at(t));
}
} // namespace

// Orthographic view: dir = viewing direction (from the eye into the scene), up = screen up.
// Returns 2D polylines in view coordinates (u = right, v = up), split into visible and hidden curves.
// Smooth (G1) edges such as root-fillet tangent lines and surface seams are suppressed, as on steel drawings.
val drawView(int id, double dx, double dy, double dz, double ux, double uy, double uz, double chordTol)
{
    SmBrep* b = get(id);
    if (!b) return val::null();
    const double t0 = nowMs();
    SmVector3d d(dx, dy, dz);
    d.Unitize();
    SmVector3d w(-d.x, -d.y, -d.z); // towards the eye
    SmVector3d up(ux, uy, uz);
    SmVector3d u(up.y * w.z - up.z * w.y, up.z * w.x - up.x * w.z, up.x * w.y - up.y * w.x); // up x w = right
    u.Unitize();
    SmVector3d v(w.y * u.z - w.z * u.y, w.z * u.x - w.x * u.z, w.x * u.y - w.y * u.x);

    SmPoint3d lo, hi;
    SmApiBrepBoundingBox(b, FALSE, lo, hi);
    const double diag = std::sqrt((hi.x - lo.x) * (hi.x - lo.x) + (hi.y - lo.y) * (hi.y - lo.y) + (hi.z - lo.z) * (hi.z - lo.z));
    // SmHCR's view vector points TOWARDS the eye (verified on a pocketed box: passing d shows the bottom view)
    SmHCR hcr(*SmApiGetOrCreateContext(), b, w, diag * 1e-4, 10.0 * M_PI / 180.0, FALSE /*seams*/, FALSE /*smooth edges*/);
    SmStatus s = hcr.ComputeGlobalVisibility();
    if (s != SM_SUCCESS) return val::null();
    SmTArray<SmCurve*> vis, hid0, hid;
    hcr.GetVisibleCurves(vis);
    hcr.GetInvisibleCurves(hid0);

    // With bDisplaySmoothEdges=FALSE, SmHCR moves G1 (non-silhouette) edges and seams into the INVISIBLE list,
    // which would draw e.g. root-fillet tangent lines as dashed hidden lines. Steel drawings omit them, so drop
    // invisible curves lying on edges that pass the same test SmHCR uses (IsSeam || IsTangentEdge(2.0)).
    std::vector<std::pair<SmPoint3d, SmPoint3d>> smoothSegs;
    {
        SmTArray<SmPoint3d> pts; SmTArray<ULONG> counts; SmTArray<SmEdge*> es;
        SmApiTessellateBoundaries(b, 5.0, pts, counts, es);
        ULONG o = 0;
        for (ULONG i = 0; i < counts.GetSize(); ++i)
        {
            SmEdge* e = i < es.GetSize() ? es[i] : nullptr;
            if (e && (e->IsSeam() || e->IsTangentEdge(2.0)))
                for (ULONG k = 0; k + 1 < counts[i]; ++k) smoothSegs.emplace_back(pts[o + k], pts[o + k + 1]);
            o += counts[i];
        }
    }
    const double onTol = std::max(chordTol * 4, diag * 1e-5);
    auto distToSeg = [](const SmPoint3d& p, const SmPoint3d& a, const SmPoint3d& c) {
        const double ex = c.x - a.x, ey = c.y - a.y, ez = c.z - a.z, L = ex * ex + ey * ey + ez * ez;
        double t = L > 0 ? ((p.x - a.x) * ex + (p.y - a.y) * ey + (p.z - a.z) * ez) / L : 0;
        t = std::max(0.0, std::min(1.0, t));
        const double qx = a.x + t * ex - p.x, qy = a.y + t * ey - p.y, qz = a.z + t * ez - p.z;
        return std::sqrt(qx * qx + qy * qy + qz * qz);
    };
    int droppedSmooth = 0;
    for (ULONG i = 0; i < hid0.GetSize(); ++i)
    {
        const SmExtent1d iv = hid0[i]->GetNaturalInterval();
        bool onSmooth = !smoothSegs.empty();
        for (double f : { 0.2, 0.5, 0.8 })
        {
            if (!onSmooth) break;
            SmVector3d pv[1];
            hid0[i]->Evaluate(iv.GetMin() + f * (iv.GetMax() - iv.GetMin()), 0, FALSE, pv);
            const SmPoint3d p(pv[0].x, pv[0].y, pv[0].z);
            bool near = false;
            for (const auto& sg : smoothSegs) if (distToSeg(p, sg.first, sg.second) < onTol) { near = true; break; }
            onSmooth = near;
        }
        if (onSmooth) ++droppedSmooth; else hid.Add(hid0[i]);
    }

    auto pack = [&](SmTArray<SmCurve*>& curves) {
        std::vector<float> pos;
        std::vector<uint32_t> cnt;
        std::vector<SmPoint3d> pts;
        for (ULONG i = 0; i < curves.GetSize(); ++i)
        {
            sampleCurve(curves[i], chordTol, pts);
            if (pts.size() < 2) continue;
            const size_t start = pos.size();
            double len = 0;
            for (size_t k = 0; k < pts.size(); ++k)
            {
                const SmPoint3d& p = pts[k];
                const float pu = (float)(p.x * u.x + p.y * u.y + p.z * u.z), pv = (float)(p.x * v.x + p.y * v.y + p.z * v.z);
                if (k) len += std::hypot(pu - pos[pos.size() - 2], pv - pos[pos.size() - 1]);
                pos.push_back(pu);
                pos.push_back(pv);
            }
            if (len < chordTol) { pos.resize(start); continue; } // edges parallel to the view collapse to a point
            cnt.push_back((uint32_t)pts.size());
        }
        val o = val::object();
        o.set("positions", toTyped("Float32Array", pos));
        o.set("counts", toTyped("Uint32Array", cnt));
        return o;
    };
    val o = val::object();
    o.set("visible", pack(vis));
    o.set("hidden", pack(hid));
    o.set("droppedSmooth", droppedSmooth);
    o.set("ms", nowMs() - t0);
    return o;
}

double wasmMemoryMB() { return __builtin_wasm_memory_size(0) * 65536.0 / 1e6; }

EMSCRIPTEN_BINDINGS(brep)
{
    value_object<OpResult>("OpResult")
        .field("status", &OpResult::status)
        .field("ok", &OpResult::ok)
        .field("rolledBack", &OpResult::rolledBack)
        .field("ms", &OpResult::ms);
    function("extrudeI", &extrudeI);
    function("extrudePolygon", &extrudePolygon);
    function("extrudeOutline", &extrudeOutline);
    function("box", &box);
    function("copy", &copy);
    function("free", &freeBrep);
    function("count", &count);
    function("translate", &translate);
    function("rotate", &rotate);
    function("hole", &hole);
    function("cutBox", &cutBox);
    function("cutPlane", &cutPlane);
    function("unite", &unite);
    function("subtract", &subtract);
    function("volume", &volume);
    function("validate", &validate);
    function("bbox", &bbox);
    function("tessellate", &tessellate);
    function("edges", &edges);
    function("drawView", &drawView);
    function("wasmMemoryMB", &wasmMemoryMB);
}
