"""Draw src/planesim/gfx/icon.png: the plane seen from above as an Ethos mask
(greyscale: black is drawn, white is clear, as lcd.loadMask expects).

    python tools/make_icon.py      (needs lupa and pillow)
"""
import os

from lupa import lua54
from PIL import Image, ImageDraw

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SIZE, SS = 75, 4   # final size, supersampling

os.chdir(os.path.join(ROOT, "src", "planesim"))
L = lua54.LuaRuntime()
p = L.eval('assert(loadfile("lib/plane.lua"))()')
p.build()
while not p.buildStep():
    pass

n = p.vertexCount
xs = [p.MX[i] for i in range(1, n + 1)]   # forward
ys = [p.MY[i] for i in range(1, n + 1)]   # left
x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
scale = (SIZE * SS * 0.92) / max(x1 - x0, y1 - y0)
cx, cy = (x0 + x1) / 2, (y0 + y1) / 2

img = Image.new("L", (SIZE * SS, SIZE * SS), 0)
d = ImageDraw.Draw(img)


def pt(i):
    # Nose up, left wing on the left.
    return (SIZE * SS / 2 - (p.MY[i] - cy) * scale, SIZE * SS / 2 - (p.MX[i] - cx) * scale)


for f in range(1, p.faceCount + 1):
    if p.FMAT[f] in (p.MAT_DISC, p.MAT_PROP):
        continue
    ids = [p.FA[f], p.FB[f], p.FC[f]] + ([p.FD[f]] if p.FD[f] else [])
    d.polygon([pt(i) for i in ids], fill=255)

img = img.resize((SIZE, SIZE), Image.LANCZOS)
img = Image.eval(img, lambda a: 255 - a)
out = os.path.join(ROOT, "src", "planesim", "gfx", "icon.png")
img.save(out)
print("wrote", out)
