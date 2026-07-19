"""Rescale URDF inertia tensors to match the PR#3 mass update.

PR#3 (eb10e2d) updated <mass> for link2..link6 but left the <inertia>
tensors at their pre-update (CAD, old-mass) values. For unchanged geometry,
inertia scales linearly with mass, so the consistent correction is
I_new = I_old * (m_new / m_old) per link. CoM is left untouched (no
information about distribution changes). Gravity compensation g(q) is
unaffected (depends on m and CoM only); this fixes M(q)/dynamics and the
Gain Tuner's accumulated-inertia computation.

Run: python3 fix_inertia.py   (rewrites urdf in place, prints the scaling)
"""

import re
from pathlib import Path

URDF = Path("/home/spark/Projects/demo/reBot-Isaacsim/urdf/00-arm-rs_asm-v3/urdf/00-arm-rs_asm-v3.urdf")

# m_old (c2eba19) -> m_new (b094da6)
SCALE = {
    "link2": 1.552 / 1.972,
    "link3": 1.252 / 1.062,
    "link4": 0.46 / 0.66,
    "link5": 0.20120457182895 / 0.150120457182895,
    "link6": 0.1 / 0.03,
}

text = URDF.read_text()

for link, s in SCALE.items():
    lm = re.search(rf'(<link\s+name="{link}">.*?</link>)', text, re.S)
    block = lm.group(1)
    im = re.search(r"<inertia\b[^/]*?/>", block, re.S)
    tag = im.group(0)

    def scaled(m):
        return f'{m.group(1)}="{float(m.group(2)) * s:.9g}"'

    new_tag = re.sub(r'(i[xyz][xyz])="([^"]+)"', scaled, tag)
    text = text.replace(tag, new_tag, 1)
    print(f"{link}: x{s:.4f}")
    print(f"  old: {' '.join(tag.split())}")
    print(f"  new: {' '.join(new_tag.split())}")

URDF.write_text(text)
print("\nWROTE", URDF)
