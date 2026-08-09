"""Verify mjcf/rebot_devarm is in sync with its upstream MuJoCo Menagerie source.

The model here is no longer generated from the repo URDF: it is vendored from
the Menagerie `seeed_rebot_devarm` package, which is itself derived from this
repo's URDF and carries the reviewed collision geometry and self-collision
policy. Only the `<mujoco model=...>` name differs, so the package can be
diffed against upstream mechanically.

Usage:
    python check_menagerie_sync.py            # verify against the pinned SHA
    python check_menagerie_sync.py --update   # re-vendor from the pinned SHA

Exits non-zero if the vendored files differ from upstream.
"""

from __future__ import annotations

import argparse
import hashlib
import io
import shutil
import sys
import tarfile
import urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent
MODEL_XML = HERE / "rebot_devarm.xml"
ASSETS = HERE / "assets"

# Pinned upstream source: the merge commit of mujoco_menagerie#300.
MENAGERIE_SHA = "da76818e269b82289eba39808e2fb91d679d6994"
UPSTREAM_PKG = "seeed_rebot_devarm"
TARBALL = (f"https://github.com/google-deepmind/mujoco_menagerie/archive/"
           f"{MENAGERIE_SHA}.tar.gz")

# our only intentional divergence from upstream
LOCAL_MODEL_NAME = "rebot_devarm"
UPSTREAM_MODEL_NAME = "seeed_rebot_devarm"


def fetch_upstream() -> dict[str, bytes]:
    """Return {relative path: bytes} for the upstream package."""
    with urllib.request.urlopen(TARBALL, timeout=120) as r:
        blob = r.read()
    out: dict[str, bytes] = {}
    with tarfile.open(fileobj=io.BytesIO(blob), mode="r:gz") as tf:
        for member in tf.getmembers():
            if not member.isfile():
                continue
            parts = Path(member.name).parts
            if len(parts) < 2 or parts[1] != UPSTREAM_PKG:
                continue
            rel = Path(*parts[2:])
            if rel.parts and rel.parts[0] in ("assets",) or rel.name.endswith(
                    ".xml"):
                f = tf.extractfile(member)
                if f:
                    out[str(rel)] = f.read()
    return out


def localize(xml: bytes) -> bytes:
    return xml.replace(
        f'<mujoco model="{UPSTREAM_MODEL_NAME}">'.encode(),
        f'<mujoco model="{LOCAL_MODEL_NAME}">'.encode())


def digest(b: bytes) -> str:
    return hashlib.sha256(b).hexdigest()[:12]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--update", action="store_true",
                    help="rewrite local files from the pinned upstream SHA")
    args = ap.parse_args()

    print(f"upstream: mujoco_menagerie@{MENAGERIE_SHA[:12]} / {UPSTREAM_PKG}")
    upstream = fetch_upstream()
    if not upstream:
        print("ERROR: could not read the upstream package", file=sys.stderr)
        return 2

    problems = 0

    # the model file, with the name localized
    want = localize(upstream[f"{UPSTREAM_PKG}.xml"])
    if args.update:
        MODEL_XML.write_bytes(want)
        print(f"updated {MODEL_XML.name}")
    else:
        have = MODEL_XML.read_bytes() if MODEL_XML.exists() else b""
        if have != want:
            problems += 1
            print(f"DIFFERS  {MODEL_XML.name}  "
                  f"local={digest(have)} upstream={digest(want)}")
        else:
            print(f"ok       {MODEL_XML.name}")

    # assets
    up_assets = {k: v for k, v in upstream.items() if k.startswith("assets/")}
    if args.update:
        shutil.rmtree(ASSETS, ignore_errors=True)
        ASSETS.mkdir(parents=True)
        for rel, data in up_assets.items():
            (HERE / rel).write_bytes(data)
        print(f"updated {len(up_assets)} asset files")
    else:
        local = {f"assets/{p.name}": p.read_bytes()
                 for p in sorted(ASSETS.glob("*")) if p.is_file()}
        missing = sorted(set(up_assets) - set(local))
        extra = sorted(set(local) - set(up_assets))
        changed = sorted(k for k in set(up_assets) & set(local)
                         if up_assets[k] != local[k])
        for k in missing:
            print(f"MISSING  {k}")
        for k in extra:
            print(f"EXTRA    {k}")
        for k in changed:
            print(f"DIFFERS  {k}")
        problems += len(missing) + len(extra) + len(changed)
        if not (missing or extra or changed):
            print(f"ok       {len(local)} asset files")

    if args.update:
        print("\nre-vendored; review the diff and commit")
        return 0
    if problems:
        print(f"\n{problems} file(s) out of sync with "
              f"mujoco_menagerie@{MENAGERIE_SHA[:12]}")
        print("run with --update to re-vendor")
        return 1
    print("\nin sync with upstream")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
