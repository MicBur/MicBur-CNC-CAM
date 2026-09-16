"""Erzeugt das Programmsymbol (app_icon.ico / app_icon.png) für MicBur-CNC-CAM.

Motiv: Schaftfräser mit Spiralnuten fräst eine leuchtende Bahn in einen Stahlblock, Funkenflug.
Aufruf: python make_icon.py   (benötigt Pillow)
"""
import math
import os
import random

from PIL import Image, ImageDraw, ImageFilter

S = 1024          # Endgröße der Vorlage
SS = 4            # Supersampling für glatte Kanten
W = S * SS
HERE = os.path.dirname(os.path.abspath(__file__))


def lerp(a, b, t):
    return tuple(int(a[i] + (b[i] - a[i]) * t) for i in range(len(a)))


def vertical_gradient(size, top, bottom):
    img = Image.new('RGBA', size)
    d = ImageDraw.Draw(img)
    for y in range(size[1]):
        d.line([(0, y), (size[0], y)], fill=lerp(top, bottom, y / max(1, size[1] - 1)))
    return img


def horizontal_gradient(size, stops):
    img = Image.new('RGBA', size)
    d = ImageDraw.Draw(img)
    for x in range(size[0]):
        t = x / max(1, size[0] - 1)
        for i in range(len(stops) - 1):
            t0, c0 = stops[i]
            t1, c1 = stops[i + 1]
            if t0 <= t <= t1:
                d.line([(x, 0), (x, size[1])], fill=lerp(c0, c1, (t - t0) / max(1e-6, t1 - t0)))
                break
    return img


def masked(img, mask):
    out = Image.new('RGBA', img.size, (0, 0, 0, 0))
    out.paste(img, (0, 0), mask)
    return out


def s(v):
    return int(v * SS)


canvas = Image.new('RGBA', (W, W), (0, 0, 0, 0))

# ── Hintergrund: abgerundetes Quadrat, tiefes Blau mit Schein ──
bg_mask = Image.new('L', (W, W), 0)
ImageDraw.Draw(bg_mask).rounded_rectangle([s(40), s(40), s(984), s(984)], radius=s(200), fill=255)
bg = vertical_gradient((W, W), (18, 38, 66, 255), (5, 9, 18, 255))
glow = Image.new('RGBA', (W, W), (0, 0, 0, 0))
ImageDraw.Draw(glow).ellipse([s(160), s(420), s(900), s(1000)], fill=(0, 180, 255, 70))
glow = glow.filter(ImageFilter.GaussianBlur(s(90)))
bg = Image.alpha_composite(bg, glow)
canvas = Image.alpha_composite(canvas, masked(bg, bg_mask))

# Rahmenlinie
frame = Image.new('RGBA', (W, W), (0, 0, 0, 0))
ImageDraw.Draw(frame).rounded_rectangle([s(46), s(46), s(978), s(978)], radius=s(195), outline=(0, 210, 255, 150), width=s(10))
canvas = Image.alpha_composite(canvas, frame)

# ── Stahlblock (schräg von oben gesehen) ──
top_face = [(s(150), s(700)), (s(560), s(560)), (s(900), s(680)), (s(490), s(830))]
front_face = [(s(150), s(700)), (s(490), s(830)), (s(490), s(930)), (s(150), s(800))]
side_face = [(s(490), s(830)), (s(900), s(680)), (s(900), s(780)), (s(490), s(930))]

block = Image.new('RGBA', (W, W), (0, 0, 0, 0))
for poly, grad in ((top_face, horizontal_gradient((W, W), [(0.0, (120, 132, 148, 255)), (0.55, (215, 225, 236, 255)), (1.0, (140, 152, 168, 255))])),
                   (front_face, vertical_gradient((W, W), (70, 78, 92, 255), (40, 45, 55, 255))),
                   (side_face, vertical_gradient((W, W), (95, 104, 120, 255), (55, 62, 74, 255)))):
    m = Image.new('L', (W, W), 0)
    ImageDraw.Draw(m).polygon(poly, fill=255)
    block = Image.alpha_composite(block, masked(grad, m))
canvas = Image.alpha_composite(canvas, block)

# Fräserspuren auf der Oberseite (Bögen)
marks = Image.new('RGBA', (W, W), (0, 0, 0, 0))
md = ImageDraw.Draw(marks)
top_mask = Image.new('L', (W, W), 0)
ImageDraw.Draw(top_mask).polygon(top_face, fill=255)
for k in range(-6, 14):
    cx = s(300 + k * 45)
    cy = s(650 + k * 16)
    md.arc([cx - s(120), cy - s(60), cx + s(120), cy + s(60)], start=200, end=340, fill=(255, 255, 255, 55), width=s(4))
canvas = Image.alpha_composite(canvas, masked(marks, top_mask))

# Leuchtende Fräsbahn (Tasche) auf der Oberseite
path = Image.new('RGBA', (W, W), (0, 0, 0, 0))
pd = ImageDraw.Draw(path)
pts = [(s(300), s(700)), (s(560), s(610)), (s(760), s(680)), (s(505), s(770)), (s(360), s(720))]
pd.line(pts, fill=(0, 220, 255, 255), width=s(16), joint='curve')
path_glow = path.filter(ImageFilter.GaussianBlur(s(18)))
canvas = Image.alpha_composite(canvas, path_glow)
canvas = Image.alpha_composite(canvas, path_glow)
canvas = Image.alpha_composite(canvas, masked(path, top_mask))
core = Image.new('RGBA', (W, W), (0, 0, 0, 0))
ImageDraw.Draw(core).line(pts, fill=(210, 250, 255, 255), width=s(5), joint='curve')
canvas = Image.alpha_composite(canvas, masked(core, top_mask))

# ── Schaftfräser ──
tip_x, tip_y = s(560), s(612)
shank_w = s(150)
cutter_w = s(118)
cutter_top = s(360)
shank_top = s(70)

mill = Image.new('RGBA', (W, W), (0, 0, 0, 0))
steel = horizontal_gradient((W, W), [(0.0, (60, 66, 78, 255)), (0.35, (235, 240, 246, 255)), (0.55, (170, 178, 190, 255)), (1.0, (50, 56, 66, 255))])

# Schaft (breiter)
shank_mask = Image.new('L', (W, W), 0)
ImageDraw.Draw(shank_mask).rounded_rectangle([tip_x - shank_w // 2, shank_top, tip_x + shank_w // 2, cutter_top + s(20)], radius=s(18), fill=255)
shank_grad = steel.transform((W, W), Image.AFFINE, (W / shank_w, 0, -(tip_x - shank_w // 2) * W / shank_w, 0, 1, 0))
mill = Image.alpha_composite(mill, masked(shank_grad, shank_mask))

# Schneidteil mit Spiralnuten
cut_mask = Image.new('L', (W, W), 0)
ImageDraw.Draw(cut_mask).polygon([(tip_x - cutter_w // 2, cutter_top), (tip_x + cutter_w // 2, cutter_top),
                                  (tip_x + cutter_w // 2, tip_y - s(14)), (tip_x - cutter_w // 2, tip_y - s(14))], fill=255)
ImageDraw.Draw(cut_mask).ellipse([tip_x - cutter_w // 2, tip_y - s(34), tip_x + cutter_w // 2, tip_y + s(6)], fill=255)
cut_grad = steel.transform((W, W), Image.AFFINE, (W / cutter_w, 0, -(tip_x - cutter_w // 2) * W / cutter_w, 0, 1, 0))
cutter = masked(cut_grad, cut_mask)
flutes = Image.new('RGBA', (W, W), (0, 0, 0, 0))
fd = ImageDraw.Draw(flutes)
for i in range(-2, 9):
    y0 = cutter_top + i * s(52)
    fd.polygon([(tip_x - cutter_w // 2, y0 + s(38)), (tip_x + cutter_w // 2, y0 - s(20)),
                (tip_x + cutter_w // 2, y0 + s(4)), (tip_x - cutter_w // 2, y0 + s(62))], fill=(25, 30, 40, 200))
    fd.line([(tip_x - cutter_w // 2, y0 + s(66)), (tip_x + cutter_w // 2, y0 + s(8))], fill=(255, 255, 255, 120), width=s(4))
cutter = Image.alpha_composite(cutter, masked(flutes, cut_mask))
mill = Image.alpha_composite(mill, cutter)

# Spannzangen-Ring
ring = Image.new('RGBA', (W, W), (0, 0, 0, 0))
ImageDraw.Draw(ring).rounded_rectangle([tip_x - shank_w // 2 - s(12), shank_top + s(120), tip_x + shank_w // 2 + s(12), shank_top + s(150)],
                                       radius=s(8), fill=(0, 200, 255, 230))
mill = Image.alpha_composite(mill, ring)

shadow = mill.split()[3].point(lambda a: int(a * 0.6))
shadow_img = Image.new('RGBA', (W, W), (0, 0, 0, 255))
shadow_img.putalpha(shadow)
shadow_img = shadow_img.transform((W, W), Image.AFFINE, (1, 0, -s(26), 0, 1, -s(18))).filter(ImageFilter.GaussianBlur(s(20)))
canvas = Image.alpha_composite(canvas, shadow_img)
canvas = Image.alpha_composite(canvas, mill)

# ── Funken an der Schneide ──
random.seed(7)
sparks = Image.new('RGBA', (W, W), (0, 0, 0, 0))
sd = ImageDraw.Draw(sparks)
for _ in range(26):
    ang = random.uniform(math.radians(195), math.radians(345))
    length = random.uniform(60, 230)
    x0 = tip_x + s(random.uniform(-40, 40))
    y0 = tip_y - s(6)
    x1 = x0 + s(math.cos(ang) * length)
    y1 = y0 + s(math.sin(ang) * length * 0.75)
    col = random.choice([(255, 196, 60, 255), (255, 140, 30, 255), (255, 235, 150, 255)])
    sd.line([(x0, y0), (x1, y1)], fill=col, width=s(random.uniform(4, 8)))
spark_glow = sparks.filter(ImageFilter.GaussianBlur(s(10)))
canvas = Image.alpha_composite(canvas, spark_glow)
canvas = Image.alpha_composite(canvas, sparks)
hot = Image.new('RGBA', (W, W), (0, 0, 0, 0))
ImageDraw.Draw(hot).ellipse([tip_x - s(70), tip_y - s(40), tip_x + s(70), tip_y + s(30)], fill=(255, 220, 120, 200))
canvas = Image.alpha_composite(canvas, hot.filter(ImageFilter.GaussianBlur(s(22))))

# Alles außerhalb des abgerundeten Quadrats abschneiden
final = Image.new('RGBA', (W, W), (0, 0, 0, 0))
final.paste(canvas, (0, 0), bg_mask)
final = final.resize((S, S), Image.LANCZOS)

final.save(os.path.join(HERE, 'app_icon.png'))
sizes = [(16, 16), (20, 20), (24, 24), (32, 32), (40, 40), (48, 48), (64, 64), (128, 128), (256, 256)]
final.save(os.path.join(HERE, 'app_icon.ico'), sizes=sizes)
print('Symbol erzeugt:', os.path.join(HERE, 'app_icon.ico'))
