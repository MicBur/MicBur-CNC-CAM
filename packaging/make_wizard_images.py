"""Erzeugt die Bilder für den Installations-Assistenten (Inno Setup) aus dem Programmsymbol.

Aufruf: python make_wizard_images.py <Ausgabeordner>
"""
import os
import sys

from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
ICON = os.path.join(HERE, '..', 'resources', 'icons', 'app_icon.png')
OUT = sys.argv[1] if len(sys.argv) > 1 else HERE


def gradient(w, h, top, bottom):
    img = Image.new('RGB', (w, h))
    d = ImageDraw.Draw(img)
    for y in range(h):
        t = y / max(1, h - 1)
        d.line([(0, y), (w, y)], fill=tuple(int(top[i] + (bottom[i] - top[i]) * t) for i in range(3)))
    return img


def font(size, bold=True):
    for name in (('segoeuib.ttf' if bold else 'segoeui.ttf'), 'arialbd.ttf', 'arial.ttf'):
        try:
            return ImageFont.truetype(os.path.join(os.environ.get('WINDIR', 'C:/Windows'), 'Fonts', name), size)
        except OSError:
            continue
    return ImageFont.load_default()


icon = Image.open(ICON).convert('RGBA')

# Großes Bild links im Assistenten (Willkommen / Fertig), 2× für hohe DPI
W, H = 328, 628
large = gradient(W, H, (14, 30, 54), (4, 8, 16))
ic = icon.resize((240, 240), Image.LANCZOS)
large.paste(ic, ((W - 240) // 2, 150), ic)
d = ImageDraw.Draw(large)
title = font(40)
sub = font(22, bold=False)
for text, f, y, color in (('MicBur', title, 420, (240, 246, 252)),
                          ('CNC-CAM', title, 468, (0, 210, 255)),
                          ('Conversational CAM', sub, 540, (148, 163, 184)),
                          ('3D-Simulation', sub, 570, (148, 163, 184))):
    w = d.textlength(text, font=f)
    d.text(((W - w) / 2, y), text, font=f, fill=color)
large.save(os.path.join(OUT, 'wizard_large.bmp'))

# Kleines Bild oben rechts auf den Folgeseiten
small = Image.new('RGB', (110, 110), (255, 255, 255))
ic = icon.resize((104, 104), Image.LANCZOS)
small.paste(ic, (3, 3), ic)
small.save(os.path.join(OUT, 'wizard_small.bmp'))
print('Assistentenbilder erzeugt in', OUT)
