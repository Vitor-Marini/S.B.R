#!/usr/bin/env python3
"""Printable fiducial tags for the robot and the floor (A4 PDFs in hardware/printables/).

    python3 tools/make_tags.py           → writes the PDFs and checks every tag is detected with the right ID

- apriltag-36h11.pdf : robot-top tags (70 mm, IDs 0–3) + floor tags (120 mm, IDs 10–15)   ← the robust family
- aruco-4x4.pdf      : the same layout in ArUco 4x4_50                                     ← simpler / faster
- charuco-a4.pdf     : camera calibration board (ChArUco, 5x5_100 so its IDs never clash with the tags)
Tag size = outer edge of the black border. Every page has a 100 mm bar to check the printer scale.
"""
import os

import cv2
import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.backends.backend_pdf import PdfPages  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "hardware", "printables")
A4 = (210, 297)  # mm
ROBOT_IDS, ROBOT_MM = [0, 1, 2, 3], 70
FLOOR_IDS, FLOOR_MM = [10, 11, 12, 13, 14, 15], 120
FAMILIES = {"apriltag-36h11": ("AprilTag 36h11", cv2.aruco.DICT_APRILTAG_36h11),
            "aruco-4x4": ("ArUco 4x4_50", cv2.aruco.DICT_4X4_50)}


def page():
    fig = plt.figure(figsize=(A4[0] / 25.4, A4[1] / 25.4))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, A4[0])
    ax.set_ylim(A4[1], 0)        # mm, origin top-left like a sheet of paper
    ax.axis("off")
    ax.plot([20, 120], [289, 289], "k", lw=1.5)
    for x in (20, 120):
        ax.plot([x, x], [286, 292], "k", lw=1.5)
    ax.text(125, 290.5, "esta barra deve medir 100 mm — imprima em 100% (tamanho real), sem 'ajustar à página'",
            fontsize=6.5, va="center")
    return fig, ax


def draw_tag(ax, dictionary, tag_id, x, y, size_mm, label):
    d = cv2.aruco.getPredefinedDictionary(dictionary)
    cells = d.markerSize + 2                                     # bits + 1-cell black border each side
    bits = cv2.aruco.drawMarker(d, tag_id, cells, borderBits=1)  # one pixel per cell
    c = size_mm / cells
    for r in range(cells):                                       # vector squares: sharp at any printer DPI
        for k in range(cells):
            if bits[r, k] < 128:
                ax.add_patch(Rectangle((x + k * c, y + r * c), c, c, color="black", lw=0))
    ax.text(x + size_mm / 2, y + size_mm + 5, label, ha="center", fontsize=8)
    ax.plot([x - 3, x - 3], [y, y + 4], "k", lw=0.5)            # small corner ticks: where to cut (white margin)
    ax.plot([x - 3, x + 1], [y - 3, y - 3], "k", lw=0.5)


def tags_pdf(name):
    title, dictionary = FAMILIES[name]
    path = os.path.join(OUT, name + ".pdf")
    with PdfPages(path) as pdf:
        fig, ax = page()
        ax.text(105, 18, f"{title} — tags do robô ({ROBOT_MM} mm)", ha="center", fontsize=12, weight="bold")
        for i, tid in enumerate(ROBOT_IDS):
            x, y = 25 + (i % 2) * 90, 40 + (i // 2) * 100
            draw_tag(ax, dictionary, tid, x, y, ROBOT_MM, f"{title} · ID {tid} · {ROBOT_MM} mm")
        pdf.savefig(fig)
        plt.close(fig)
        for p in range(0, len(FLOOR_IDS), 2):
            fig, ax = page()
            ax.text(105, 14, f"{title} — tags do chão ({FLOOR_MM} mm)", ha="center", fontsize=12, weight="bold")
            for j, tid in enumerate(FLOOR_IDS[p:p + 2]):
                draw_tag(ax, dictionary, tid, 45, 22 + j * 130, FLOOR_MM, f"{title} · ID {tid} · {FLOOR_MM} mm")
            pdf.savefig(fig)
            plt.close(fig)
    return path


def charuco_pdf():
    sq, mk, nx, ny = 30, 22, 6, 8                                 # mm, squares; 180 x 240 mm board
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
    board = cv2.aruco.CharucoBoard_create(nx, ny, sq / 1000, mk / 1000, d)
    img = board.draw((nx * sq * 10, ny * sq * 10))                # 10 px/mm
    path = os.path.join(OUT, "charuco-a4.pdf")
    with PdfPages(path) as pdf:
        fig, ax = page()
        ax.text(105, 12, f"ChArUco {nx}x{ny} · quadrado {sq} mm · marcador {mk} mm · DICT_5X5_100", ha="center",
                fontsize=10, weight="bold")
        ax.imshow(img, cmap="gray", extent=(15, 15 + nx * sq, 20 + ny * sq, 20), interpolation="none")
        ax.text(105, 270, "calibração de câmera: cole numa superfície plana e rígida (papelão, MDF)", ha="center",
                fontsize=8)
        pdf.savefig(fig)
        plt.close(fig)
    return path


def check(name):
    """Render the tag pages again as images and detect them: every ID must come back."""
    title, dictionary = FAMILIES[name]
    d = cv2.aruco.getPredefinedDictionary(dictionary)
    found = set()
    pages = [ROBOT_IDS] + [FLOOR_IDS[p:p + 2] for p in range(0, len(FLOOR_IDS), 2)]
    for ids in pages:
        fig, ax = page()
        size = ROBOT_MM if ids is ROBOT_IDS else FLOOR_MM
        for i, tid in enumerate(ids):
            x, y = (25 + (i % 2) * 90, 40 + (i // 2) * 100) if ids is ROBOT_IDS else (45, 22 + i * 130)
            draw_tag(ax, dictionary, tid, x, y, size, "")
        fig.canvas.draw()
        img = cv2.cvtColor(np.asarray(fig.canvas.buffer_rgba()), cv2.COLOR_RGBA2GRAY)
        plt.close(fig)
        _, got, _ = cv2.aruco.detectMarkers(img, d)
        found |= set(got.flatten().tolist()) if got is not None else set()
    want = set(ROBOT_IDS + FLOOR_IDS)
    assert found == want, f"{title}: detected {sorted(found)}, expected {sorted(want)}"
    return title


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    for name in FAMILIES:
        print("wrote", tags_pdf(name), "| detection check ok:", check(name))
    print("wrote", charuco_pdf())
