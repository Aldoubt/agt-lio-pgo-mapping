#!/usr/bin/env python3
"""Exercise MapStudio annotation controls through X11/XTest using a temp asset copy.

Requires a running X11 display, libX11, libXtst and a built map_viewer binary.
The test copies only the small research bundle into a temporary directory; source
PCDs, rosbag metadata, run008 and the persistent annotation project are read-only.
"""

from __future__ import annotations

import argparse
import ctypes
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

from contract import validate_bundle


class X11:
    def __init__(self) -> None:
        self.x11 = ctypes.CDLL("libX11.so.6")
        self.xtst = ctypes.CDLL("libXtst.so.6")
        self.x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
        self.x11.XOpenDisplay.restype = ctypes.c_void_p
        self.display = self.x11.XOpenDisplay(None)
        if not self.display:
            raise RuntimeError("Cannot connect to DISPLAY; run from an X11 desktop session")
        self.x11.XDefaultRootWindow.argtypes = [ctypes.c_void_p]
        self.x11.XDefaultRootWindow.restype = ctypes.c_ulong
        self.x11.XDefaultScreen.argtypes = [ctypes.c_void_p]
        self.x11.XDefaultScreen.restype = ctypes.c_int
        self.root = self.x11.XDefaultRootWindow(self.display)
        self.screen = self.x11.XDefaultScreen(self.display)
        self.x11.XFetchName.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.POINTER(ctypes.c_void_p)]
        self.x11.XFetchName.restype = ctypes.c_int
        self.x11.XQueryTree.argtypes = [
            ctypes.c_void_p, ctypes.c_ulong, ctypes.POINTER(ctypes.c_ulong),
            ctypes.POINTER(ctypes.c_ulong), ctypes.POINTER(ctypes.POINTER(ctypes.c_ulong)),
            ctypes.POINTER(ctypes.c_uint),
        ]
        self.x11.XGetGeometry.argtypes = [
            ctypes.c_void_p, ctypes.c_ulong, ctypes.POINTER(ctypes.c_ulong),
            ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int),
            ctypes.POINTER(ctypes.c_uint), ctypes.POINTER(ctypes.c_uint),
            ctypes.POINTER(ctypes.c_uint), ctypes.POINTER(ctypes.c_uint),
        ]
        self.x11.XTranslateCoordinates.argtypes = [
            ctypes.c_void_p, ctypes.c_ulong, ctypes.c_ulong, ctypes.c_int, ctypes.c_int,
            ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_ulong),
        ]
        self.x11.XFree.argtypes = [ctypes.c_void_p]
        self.x11.XFlush.argtypes = [ctypes.c_void_p]
        self.x11.XSetInputFocus.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int, ctypes.c_ulong]
        self.xtst.XTestFakeMotionEvent.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_ulong]
        self.xtst.XTestFakeButtonEvent.argtypes = [ctypes.c_void_p, ctypes.c_uint, ctypes.c_int, ctypes.c_ulong]
        self.xtst.XTestFakeKeyEvent.argtypes = [ctypes.c_void_p, ctypes.c_uint, ctypes.c_int, ctypes.c_ulong]
        self.x11.XKeysymToKeycode.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
        self.x11.XKeysymToKeycode.restype = ctypes.c_uint
        self.x11.XStringToKeysym.argtypes = [ctypes.c_char_p]
        self.x11.XStringToKeysym.restype = ctypes.c_ulong

    def _name(self, window: int) -> str:
        value = ctypes.c_void_p()
        if not self.x11.XFetchName(self.display, window, ctypes.byref(value)) or not value:
            return ""
        try:
            return ctypes.string_at(value).decode(errors="replace")
        finally:
            self.x11.XFree(value)

    def _children(self, parent: int) -> list[int]:
        root = ctypes.c_ulong()
        parent_return = ctypes.c_ulong()
        children = ctypes.POINTER(ctypes.c_ulong)()
        count = ctypes.c_uint()
        if not self.x11.XQueryTree(self.display, parent, ctypes.byref(root), ctypes.byref(parent_return),
                                   ctypes.byref(children), ctypes.byref(count)):
            return []
        result = [int(children[i]) for i in range(count.value)]
        if children:
            self.x11.XFree(children)
        return result

    def find_window(self, title: str, timeout_s: float = 90.0) -> int:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            tree = subprocess.run(["xwininfo", "-root", "-tree"], check=False, text=True,
                                  stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
            for match in re.finditer(r"(0x[0-9a-fA-F]+)\s+\"([^\"]*)\"", tree.stdout):
                if title not in match.group(2):
                    continue
                window = int(match.group(1), 16)
                info = subprocess.run(["xwininfo", "-id", hex(window)], check=False, text=True,
                                      stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
                if info.returncode == 0 and "Map State: IsViewable" in info.stdout:
                    return window
            time.sleep(0.2)
        raise TimeoutError(f"MapStudio window '{title}' did not appear")

    def geometry(self, window: int) -> tuple[int, int, int, int]:
        # XGetGeometry reports coordinates relative to the window manager's
        # frame. xwininfo also reports the absolute client origin, which is the
        # coordinate system XTestFakeMotionEvent expects.
        result = subprocess.run(["xwininfo", "-id", hex(window)], check=True, text=True,
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        absolute = re.search(r"Absolute upper-left X:\s*(-?\d+).*?Absolute upper-left Y:\s*(-?\d+)",
                             result.stdout, re.DOTALL)
        width = re.search(r"^\s*Width:\s*(\d+)\s*$", result.stdout, re.MULTILINE)
        height = re.search(r"^\s*Height:\s*(\d+)\s*$", result.stdout, re.MULTILINE)
        if not absolute or not width or not height:
            raise RuntimeError(f"Cannot parse MapStudio window geometry:\n{result.stdout}")
        return int(absolute.group(1)), int(absolute.group(2)), int(width.group(1)), int(height.group(1))

    def move(self, x: int, y: int) -> None:
        self.xtst.XTestFakeMotionEvent(self.display, self.screen, x, y, 0)
        self.x11.XFlush(self.display)
        time.sleep(0.12)

    def click(self, origin: tuple[int, int], point: tuple[float, float]) -> None:
        # xwininfo's absolute origin is the client area; screenshots include
        # the window-manager title bar above it.
        client_y = int(point[1]) - 56
        self.move(origin[0] + int(point[0]), origin[1] + client_y)
        self.xtst.XTestFakeButtonEvent(self.display, 1, 1, 0)
        self.x11.XFlush(self.display)
        time.sleep(0.08)
        self.xtst.XTestFakeButtonEvent(self.display, 1, 0, 0)
        self.x11.XFlush(self.display)
        time.sleep(0.18)

    def drag(self, origin: tuple[int, int], start: tuple[float, float], end: tuple[float, float]) -> None:
        self.move(origin[0] + int(start[0]), origin[1] + int(start[1]) - 56)
        self.xtst.XTestFakeButtonEvent(self.display, 1, 1, 0)
        self.x11.XFlush(self.display)
        time.sleep(0.12)
        self.move(origin[0] + int(end[0]), origin[1] + int(end[1]) - 56)
        self.xtst.XTestFakeButtonEvent(self.display, 1, 0, 0)
        self.x11.XFlush(self.display)
        time.sleep(0.4)

    def key(self, name: str) -> None:
        keysym = self.x11.XStringToKeysym(name.encode())
        keycode = self.x11.XKeysymToKeycode(self.display, keysym)
        if not keycode:
            raise RuntimeError(f"No X11 keycode for {name}")
        self.xtst.XTestFakeKeyEvent(self.display, keycode, 1, 0)
        self.x11.XFlush(self.display)
        time.sleep(0.05)
        self.xtst.XTestFakeKeyEvent(self.display, keycode, 0, 0)
        self.x11.XFlush(self.display)
        time.sleep(0.3)

    def key_combo(self, modifier: str, name: str) -> None:
        modifier_code = self.x11.XKeysymToKeycode(self.display, self.x11.XStringToKeysym(modifier.encode()))
        keycode = self.x11.XKeysymToKeycode(self.display, self.x11.XStringToKeysym(name.encode()))
        if not modifier_code or not keycode:
            raise RuntimeError(f"No X11 keycode for {modifier}+{name}")
        self.xtst.XTestFakeKeyEvent(self.display, modifier_code, 1, 0)
        self.xtst.XTestFakeKeyEvent(self.display, keycode, 1, 0)
        self.x11.XFlush(self.display)
        time.sleep(0.05)
        self.xtst.XTestFakeKeyEvent(self.display, keycode, 0, 0)
        self.xtst.XTestFakeKeyEvent(self.display, modifier_code, 0, 0)
        self.x11.XFlush(self.display)
        time.sleep(0.35)

    def close_window(self, origin: tuple[int, int], width: int, height: int) -> None:
        alt = self.x11.XKeysymToKeycode(self.display, self.x11.XStringToKeysym(b"Alt_L"))
        f4 = self.x11.XKeysymToKeycode(self.display, self.x11.XStringToKeysym(b"F4"))
        self.xtst.XTestFakeKeyEvent(self.display, alt, 1, 0)
        self.xtst.XTestFakeKeyEvent(self.display, f4, 1, 0)
        self.x11.XFlush(self.display)
        time.sleep(0.08)
        self.xtst.XTestFakeKeyEvent(self.display, f4, 0, 0)
        self.xtst.XTestFakeKeyEvent(self.display, alt, 0, 0)
        self.x11.XFlush(self.display)
        time.sleep(0.5)


def load_annotations(path: Path) -> dict:
    return json.loads(path.read_text())


def launch(app: Path, project: Path, env: dict[str, str], log_path: Path) -> subprocess.Popen:
    log = log_path.open("w")
    process = subprocess.Popen([str(app), "--research-project", str(project)], env=env,
                               stdout=log, stderr=subprocess.STDOUT)
    process._smoke_log = log  # type: ignore[attr-defined]
    return process


def stop(process: subprocess.Popen, x11: X11, window: int) -> None:
    x, y, width, height = x11.geometry(window)
    x11.close_window((x, y), width, height)
    try:
        process.wait(timeout=8)
    except subprocess.TimeoutExpired:
        process.terminate()
        process.wait(timeout=5)
    process._smoke_log.close()  # type: ignore[attr-defined]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mapstudio", type=Path, required=True)
    parser.add_argument("--project", type=Path, required=True)
    args = parser.parse_args()
    app = args.mapstudio.expanduser().resolve()
    project_path = args.project.expanduser().resolve()
    if not app.is_file() or not project_path.is_file():
        parser.error("--mapstudio and --project must name existing files")

    with tempfile.TemporaryDirectory(prefix="agt_mapstudio_gui_smoke_") as temp_name:
        temporary = Path(temp_name) / "research_assets"
        shutil.copytree(project_path.parent, temporary, ignore=shutil.ignore_patterns("evidence"))
        project = temporary / project_path.name
        annotation_path = temporary / "annotations/annotation_v1.json"
        env = os.environ.copy()
        env.setdefault("XAUTHORITY", str(Path.home() / ".Xauthority"))
        x11 = X11()

        process = launch(app, project, env, Path(temp_name) / "mapstudio-1.log")
        try:
            window = x11.find_window("AGT Map Studio")
            origin_x, origin_y, width, height = x11.geometry(window)
            # Select Draw in the Annotation workspace, then create a four-corner polygon.
            x11.click((origin_x, origin_y), (width * 0.305, height * 0.092))
            vertices = [
                (width * 0.36, height * 0.40), (width * 0.47, height * 0.40),
                (width * 0.47, height * 0.56), (width * 0.36, height * 0.56),
            ]
            for vertex in vertices:
                x11.click((origin_x, origin_y), vertex)
            x11.key("Return")
            time.sleep(0.5)
            x11.key_combo("Control_L", "s")
            time.sleep(0.3)
            first = load_annotations(annotation_path)
            if len(first["annotations"]) != 3:
                failure_log = Path(temp_name) / "mapstudio-1.log"
                debug_shot = Path("/tmp/agt_mapstudio_gui_failure.png")
                subprocess.run(["gnome-screenshot", "--file", str(debug_shot)], check=False,
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                print(f"GUI debug: window=({origin_x},{origin_y},{width},{height}); "
                      f"annotation_count={len(first['annotations'])}; log={failure_log}", file=sys.stderr)
                if failure_log.exists(): print(failure_log.read_text(errors="replace"), file=sys.stderr)
                print(f"GUI screenshot: {debug_shot}", file=sys.stderr)
                raise AssertionError("GUI did not add one polygon to the two imported candidates")
            created = first["annotations"][-1]
            if created["review_status"] != "DRAFT" or created["source_session_ids"] != [
                "session_green_house", "session_white_tomato_collect_20261006_080252"
            ]:
                raise AssertionError("GUI polygon lost its DRAFT or exact two-session binding")
            original = created["geometry"]["coordinates_xy_m"]
            stop(process, x11, window)

            # Reopen the saved revision, drag one vertex, save, and verify map-coordinate change.
            process = launch(app, project, env, Path(temp_name) / "mapstudio-2.log")
            window = x11.find_window("AGT Map Studio")
            origin_x, origin_y, width, height = x11.geometry(window)
            x11.click((origin_x, origin_y), (width * 0.86, height * 0.50))  # select the new feature row
            x11.click((origin_x, origin_y), (width * 0.82, height * 0.932))  # selected-object Edit
            x11.drag((origin_x, origin_y), vertices[0],
                     (vertices[0][0] + width * 0.018, vertices[0][1] + height * 0.022))
            x11.key_combo("Control_L", "s")
            time.sleep(0.3)
            edited_doc = load_annotations(annotation_path)
            edited = edited_doc["annotations"][-1]["geometry"]["coordinates_xy_m"]
            if len(edited) != 4 or edited == original:
                debug_shot = Path("/tmp/agt_mapstudio_gui_edit_failure.png")
                subprocess.run(["gnome-screenshot", "--window", "--file", str(debug_shot)], check=False,
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                print(f"GUI edit debug: window=({origin_x},{origin_y},{width},{height}); "
                      f"original={original}; edited={edited}; screenshot={debug_shot}", file=sys.stderr)
                raise AssertionError("GUI vertex drag did not change the persisted polygon coordinates")
            moved_vertex = (vertices[0][0] + width * 0.018, vertices[0][1] + height * 0.022)
            x11.click((origin_x, origin_y), moved_vertex)
            x11.key("Delete")  # selected vertex is the delete context
            x11.key_combo("Control_L", "s")
            after_delete = load_annotations(annotation_path)["annotations"][-1]["geometry"]["coordinates_xy_m"]
            if len(after_delete) != 3:
                raise AssertionError("GUI Delete vertex did not remove exactly one polygon vertex")
            x11.key_combo("Control_L", "z")
            x11.key_combo("Control_L", "s")
            after_undo = load_annotations(annotation_path)["annotations"][-1]["geometry"]["coordinates_xy_m"]
            if after_undo != edited:
                raise AssertionError("GUI Undo did not restore the edited polygon coordinates")
            stop(process, x11, window)

            # A third open proves those exact edited map coordinates reload from disk.
            process = launch(app, project, env, Path(temp_name) / "mapstudio-3.log")
            window = x11.find_window("AGT Map Studio")
            reloaded = load_annotations(annotation_path)["annotations"][-1]["geometry"]["coordinates_xy_m"]
            if reloaded != edited:
                raise AssertionError("polygon coordinates changed after GUI exit/reload")
            stop(process, x11, window)

            summary = validate_bundle(project)
            if summary["paper_statistics_eligible"]:
                raise AssertionError("DRAFT GUI annotations must not enter formal paper statistics")
            print(json.dumps({
                "status": "PASS", "created_feature": created["annotation_id"],
                "created_vertices": len(original), "vertex_edit_persisted": True,
                "session_ids": created["source_session_ids"],
                "paper_statistics_eligible": summary["paper_statistics_eligible"],
                "project_copy": str(temporary),
            }, indent=2))
            return 0
        finally:
            if process.poll() is None:
                try:
                    window = x11.find_window("AGT Map Studio", timeout_s=2)
                    stop(process, x11, window)
                except Exception:
                    process.terminate()
                    process.wait(timeout=5)


if __name__ == "__main__":
    raise SystemExit(main())
