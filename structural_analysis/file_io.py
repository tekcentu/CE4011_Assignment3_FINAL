"""
File I/O: text-based input parser (compatible with Assignment 2 format).

Extended with optional fields for element type, releases, and member loads.
Creates Element2D subclass instances (FrameElement2D / TrussElement2D).
"""

from __future__ import annotations

from .model import (
    StructuralModel, Node, Material, Support, NodalLoad,
    UniformDistributedLoad, PointLoad,
)
from .element import FrameElement2D, TrussElement2D


def read_input_file(filepath: str) -> StructuralModel:
    """Parse a structural model from a text input file.

    Supports sections: TITLE, NODES, MATERIALS, ELEMENTS, SUPPORTS,
    LOADS, MEMBER_POINT_LOADS, MEMBER_UDL. Lines starting with # are
    comments. Element lines accept optional type (FRAME/TRUSS) and
    release (START/END/BOTH) fields.

    Args:
        filepath: Path to the input file.

    Returns:
        A populated StructuralModel with Element2D subclass instances.
    """
    model = StructuralModel()

    with open(filepath, "r") as f:
        lines = [ln.strip() for ln in f.readlines()]

    # Temporary storage for member loads (we need elements to exist first)
    pending_point_loads: list[tuple[int, float, float, float]] = []
    pending_udls: list[tuple[int, float, float]] = []

    i = 0
    while i < len(lines):
        line = lines[i]
        if not line or line.startswith("#"):
            i += 1
            continue

        tokens = line.split()
        keyword = tokens[0].upper()

        if keyword == "TITLE":
            i += 1
            model.title = lines[i] if i < len(lines) else "Untitled"

        elif keyword == "NODES":
            count = int(tokens[1])
            for _ in range(count):
                i += 1
                while i < len(lines) and (not lines[i] or lines[i].startswith("#")):
                    i += 1
                parts = lines[i].split("#")[0].split()
                nid = int(parts[0])
                model.nodes[nid] = Node(nid, float(parts[1]), float(parts[2]))

        elif keyword == "MATERIALS":
            count = int(tokens[1])
            for _ in range(count):
                i += 1
                while i < len(lines) and (not lines[i] or lines[i].startswith("#")):
                    i += 1
                parts = lines[i].split("#")[0].split()
                mid = int(parts[0])
                A_val, I_val, E_val = float(parts[1]), float(parts[2]), float(parts[3])
                model.materials[mid] = Material(mid, E_val, A_val, I_val)

        elif keyword == "ELEMENTS":
            count = int(tokens[1])
            for _ in range(count):
                i += 1
                while i < len(lines) and (not lines[i] or lines[i].startswith("#")):
                    i += 1
                parts = lines[i].split("#")[0].split()
                eid = int(parts[0])
                sn, en, mat_id = int(parts[1]), int(parts[2]), int(parts[3])
                mat = model.materials[mat_id]

                # Optional element type
                etype = "FRAME"
                if len(parts) >= 5:
                    etype = parts[4].upper()

                # Optional release
                release_i = False
                release_j = False
                if len(parts) >= 6:
                    r = parts[5].upper()
                    if r == "START":
                        release_i = True
                    elif r == "END":
                        release_j = True
                    elif r == "BOTH":
                        release_i = True
                        release_j = True

                if etype == "TRUSS":
                    elem = TrussElement2D(
                        id=eid, node_i=sn, node_j=en,
                        E=mat.E, A=mat.A,
                    )
                else:
                    elem = FrameElement2D(
                        id=eid, node_i=sn, node_j=en,
                        E=mat.E, A=mat.A, I=mat.I,
                        release_i=release_i, release_j=release_j,
                    )
                model.elements.append(elem)

        elif keyword == "SUPPORTS":
            count = int(tokens[1])
            for _ in range(count):
                i += 1
                while i < len(lines) and (not lines[i] or lines[i].startswith("#")):
                    i += 1
                parts = lines[i].split("#")[0].split()
                nid = int(parts[0])
                model.supports[nid] = Support(
                    nid, bool(int(parts[1])), bool(int(parts[2])), bool(int(parts[3]))
                )

        elif keyword == "LOADS":
            count = int(tokens[1])
            for _ in range(count):
                i += 1
                while i < len(lines) and (not lines[i] or lines[i].startswith("#")):
                    i += 1
                parts = lines[i].split("#")[0].split()
                model.nodal_loads.append(NodalLoad(
                    int(parts[0]), float(parts[1]), float(parts[2]), float(parts[3])
                ))

        elif keyword == "MEMBER_POINT_LOADS":
            count = int(tokens[1])
            for _ in range(count):
                i += 1
                while i < len(lines) and (not lines[i] or lines[i].startswith("#")):
                    i += 1
                parts = lines[i].split("#")[0].split()
                eid = int(parts[0])
                a = float(parts[1])
                px = float(parts[2]) if len(parts) > 2 else 0.0
                py = float(parts[3]) if len(parts) > 3 else 0.0
                # Find element and add load
                for elem in model.elements:
                    if elem.id == eid:
                        elem.member_loads.append(PointLoad(py=py, a=a))
                        break

        elif keyword == "MEMBER_UDL":
            count = int(tokens[1])
            for _ in range(count):
                i += 1
                while i < len(lines) and (not lines[i] or lines[i].startswith("#")):
                    i += 1
                parts = lines[i].split("#")[0].split()
                eid = int(parts[0])
                wx = float(parts[1]) if len(parts) > 1 else 0.0
                wy = float(parts[2]) if len(parts) > 2 else 0.0
                for elem in model.elements:
                    if elem.id == eid:
                        elem.member_loads.append(UniformDistributedLoad(wy=wy))
                        break

        i += 1

    return model
