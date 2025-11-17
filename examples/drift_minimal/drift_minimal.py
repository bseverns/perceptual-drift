#!/usr/bin/env python3
"""Minimal perceptual drift sketch.

This script is the "hello world" loop promised in the repo README.
It watches a single image, keeps layering it with tiny offsets, and
pushes the stack into a Tkinter window so you can watch drift happen
without touching drones, Betaflight, or even OSC.

Dependencies: Python 3.9+ and Pillow (``pip install pillow``).
"""
from __future__ import annotations

import argparse
import math
import pathlib
import time
import tkinter as tk
from typing import Iterable, Tuple

from PIL import Image, ImageChops, ImageTk
FRAME_MS = 60  # ~16 FPS keeps CPU low but motion legible.


class DriftLoop:
    """Encapsulates the micro state machine for the visual drift."""

    def __init__(self, source: Image.Image, shifts: Iterable[Tuple[int, str]]):
        # Store a copy so we don't mutate the caller's Image object.
        self.base = source.convert("RGBA")
        self.layers = [self._tint_layer(self.base, tint) for _, tint in shifts]
        self.shift_specs = list(shifts)
        self.frame_index = 0

    @staticmethod
    def _tint_layer(layer: Image.Image, tint: str) -> Image.Image:
        """Blend the layer with a solid color so each pass feels unique."""
        tint_image = Image.new("RGBA", layer.size, tint)
        # Blend keeps most of the underlying image intact but pushes the tone.
        return Image.blend(layer, tint_image, alpha=0.2)

    def next_frame(self) -> Image.Image:
        """Return the next layered frame."""
        frame = Image.new("RGBA", self.base.size, (0, 0, 0, 255))
        for index, (shift_range, tint) in enumerate(self.shift_specs):
            layer = self.layers[index]
            # Use frame index to drive slow sin/cos offsets.
            offset_x = math.sin((self.frame_index + index * 7) / 35.0) * shift_range
            offset_y = math.cos((self.frame_index + index * 5) / 45.0) * shift_range
            shifted = ImageChops.offset(layer, int(offset_x), int(offset_y))
            frame = Image.alpha_composite(frame, shifted)
        self.frame_index += 1
        return frame


def generate_seed_image(size: Tuple[int, int] = (480, 480)) -> Image.Image:
    """Procedurally build a gradient seed so we don't need binary assets."""

    width, height = size
    gradient = Image.new("RGBA", size)
    pixels = []
    width_norm = max(1, width - 1)
    height_norm = max(1, height - 1)
    for y in range(height):
        y_ratio = y / height_norm
        for x in range(width):
            x_ratio = x / width_norm
            # Sin/cos recipes chosen so the palette slowly morphs across axes.
            r = int(40 + 180 * abs(math.sin((x_ratio * 3.2) + (y_ratio * 0.8))))
            g = int(60 + 150 * abs(math.cos((y_ratio * 2.5) - (x_ratio * 0.9))))
            b = int(90 + 140 * abs(math.sin((x_ratio + y_ratio) * 2.1)))
            pixels.append((r, g, b, 255))
    gradient.putdata(pixels)

    # Blend a shifted copy back in so the seed already feels layered.
    shifted = ImageChops.offset(gradient, 35, -25)
    return Image.blend(gradient, shifted, alpha=0.35)


class DriftApp:
    """Tkinter wrapper that paints frames and overlays helper HUD text."""

    def __init__(self, source_path):
        if source_path:
            path = pathlib.Path(source_path)
            if not path.exists():
                raise SystemExit(f"Input image not found: {path}")
            image = Image.open(path)
        else:
            image = generate_seed_image()
        # Each tuple = (max pixel offset, tint color). Smaller offsets stay centered
        # so the image feels anchored while the outer rings slowly wander.
        self.loop = DriftLoop(
            image,
            shifts=[
                (4, "#66c2ff"),
                (9, "#ff7eb9"),
                (15, "#fff178"),
            ],
        )

        self.root = tk.Tk()
        self.root.title("Drift Minimal — layered hello world")
        self.canvas = tk.Canvas(self.root, width=image.width, height=image.height)
        self.canvas.pack()

        self.overlay_text = self.canvas.create_text(
            10,
            10,
            anchor="nw",
            fill="#ffffff",
            font=("Helvetica", 12, "bold"),
            text="",
        )

        self.photo = None  # Reference kept alive to avoid garbage collection.
        self.is_running = True
        self.root.bind("<space>", self.toggle_pause)
        self.root.bind("q", self.quit)

    def toggle_pause(self, _event=None):
        self.is_running = not self.is_running

    def quit(self, _event=None):
        self.root.destroy()

    def start(self):
        self._tick()
        self.root.mainloop()

    def _tick(self):
        # Tkinter calls this method every FRAME_MS. We do three things:
        # 1. Ask DriftLoop for the next composite image.
        # 2. Convert it into a PhotoImage Tkinter understands.
        # 3. Re-schedule ourselves so the loop keeps breathing.
        start_time = time.time()
        if self.is_running:
            frame = self.loop.next_frame()
            hud = self._hud_text()
            self.photo = ImageTk.PhotoImage(frame)
            self.canvas.create_image(0, 0, anchor="nw", image=self.photo)
            self.canvas.itemconfig(self.overlay_text, text=hud)
        elapsed_ms = (time.time() - start_time) * 1000
        delay = max(5, FRAME_MS - int(elapsed_ms))
        self.root.after(delay, self._tick)

    def _hud_text(self) -> str:
        hints = [
            "space = pause",
            "q = quit",
            "Layer offsets modulated by sin/cos pairs",
        ]
        return "\n".join(hints)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Loop a single image through layered offsets so students can watch "
            "perceptual drift without booting the full stack."
        )
    )
    parser.add_argument(
        "--input",
        type=pathlib.Path,
        default=None,
        help="Path to an image. Defaults to a procedurally generated gradient.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    app = DriftApp(args.input)
    app.start()


if __name__ == "__main__":
    main()
