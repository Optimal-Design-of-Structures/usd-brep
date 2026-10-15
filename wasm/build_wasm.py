# SPDX-License-Identifier: Apache-2.0
"""Build the SMLib kernel (unmodified ../source) + the iQSteel narrow API to a WASM ES module.

usage: python wasm/build_wasm.py [--opt Oz] [--jobs N] [--out wasm/dist]
needs: em++ on PATH (emsdk activated) or EMXX=<path to em++>; Python 3.9+

output (--out): brep.mjs, brep.wasm, build-info.json, LICENSE, NOTICE
"""
import argparse
import concurrent.futures as cf
import hashlib
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

WASM = Path(__file__).resolve().parent
ROOT = WASM.parent
SRC = ROOT / "source"

ap = argparse.ArgumentParser()
ap.add_argument("--opt", default="Oz", choices=["O2", "O3", "Os", "Oz"])
ap.add_argument("--jobs", type=int, default=os.cpu_count() or 4)
ap.add_argument("--out", default=str(WASM / "dist"))
args = ap.parse_args()

emxx = os.environ.get("EMXX") or shutil.which("em++")
if not emxx:
    sys.exit("em++ not found: activate emsdk (source emsdk_env.sh) or set EMXX")

CFLAGS = [
    f"-{args.opt}", "-std=c++17", "-fwasm-exceptions", "-w",
    "-DNDEBUG", "-DNOMINMAX", "-DSM_NO_DEBUG_CODE", "-DSM_NO_GFX_OUTPUT_CODE",
    f"-I{SRC / 'SMLib' / 'inc'}", f"-I{SRC / 'SM_API' / 'inc'}",
]
LINK = [
    "-lembind", "-sMODULARIZE=1", "-sEXPORT_ES6=1", "-sENVIRONMENT=web,worker,node",
    "-sALLOW_MEMORY_GROWTH=1", "-sMAXIMUM_MEMORY=4GB", "-sSTACK_SIZE=8MB",
]
# SM_API sources rely on the Windows PCH (StdAfx.h) for some declarations; reproduce it explicitly.
SM_API_EXTRA = ["-DSM_API_EXPORTS", "-include", str(SRC / "SM_API" / "inc" / "StdAfx.h")]


def run(cmd):
    return subprocess.run(cmd, capture_output=True, text=True)


em_version = run([emxx, "--version"]).stdout.splitlines()[0].strip()
flags_hash = hashlib.sha1(json.dumps([CFLAGS, SM_API_EXTRA, em_version]).encode()).hexdigest()[:10]
obj_dir = WASM / "build" / f"obj-{args.opt}-{flags_hash}"  # new flags/toolchain -> fresh objects
obj_dir.mkdir(parents=True, exist_ok=True)
out = Path(args.out)
out.mkdir(parents=True, exist_ok=True)

sources = sorted((SRC / "SMLib" / "src").glob("*.cpp")) + sorted((SRC / "SM_API" / "src").glob("*.cpp"))


def compile_one(src: Path):
    lib = src.parent.parent.name
    obj = obj_dir / f"{lib}__{src.stem}.o"
    if obj.exists() and obj.stat().st_mtime > src.stat().st_mtime:
        return src.name, 0.0, None
    t = time.time()
    extra = SM_API_EXTRA if lib == "SM_API" else []
    r = run([emxx, *CFLAGS, *extra, "-c", str(src), "-o", str(obj)])
    return src.name, time.time() - t, None if r.returncode == 0 else (r.stderr or r.stdout)[-4000:]


t0 = time.time()
failures = []
with cf.ThreadPoolExecutor(max_workers=args.jobs) as ex:
    for name, _, err in ex.map(compile_one, sources):
        if err:
            failures.append((name, err))
print(f"compile: {len(sources)} kernel files, {time.time() - t0:.0f}s, {len(failures)} failures ({em_version})")
for name, err in failures[:10]:
    print(f"===== {name}\n{err}")
if failures:
    sys.exit(1)

t1 = time.time()
r = run([emxx, *CFLAGS, str(WASM / "brep_api.cpp"), *sorted(str(p) for p in obj_dir.glob("*.o")), *LINK, "-o", str(out / "brep.mjs")])
if r.returncode:
    print(r.stderr[-6000:])
    sys.exit(1)
print(f"link: {time.time() - t1:.0f}s")

upstream = (WASM / "UPSTREAM").read_text().split()
git = lambda *a: run(["git", "-C", str(ROOT), *a]).stdout.strip()
info = {
    "upstream": {"commit": upstream[0], "repo": upstream[1], "version": upstream[2] if len(upstream) > 2 else None},
    "fork_commit": git("rev-parse", "HEAD"),
    "fork_describe": git("describe", "--tags", "--always", "--dirty"),
    "emscripten": em_version,
    "opt": args.opt,
    "cflags": CFLAGS,
    "link": LINK,
    "bytes": {f.name: f.stat().st_size for f in (out / "brep.wasm", out / "brep.mjs")},
}
(out / "build-info.json").write_text(json.dumps(info, indent=1))
shutil.copy(ROOT / "LICENSE", out / "LICENSE")
shutil.copy(WASM / "NOTICE", out / "NOTICE")
for f in ("brep.mjs", "brep.wasm"):
    print(f"  {f:10s} {(out / f).stat().st_size / 1e6:6.2f} MB")
