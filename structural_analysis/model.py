"""
Data model classes for 2D structural analysis.

Supports frame elements, truss elements, moment releases,
and member loads (point loads and UDL).

Design decisions
----------------
- Node, Support, NodalLoad are frozen (immutable) dataclasses — once created
  they should not be mutated.
- Element classes live in element.py and use inheritance (Element2D base).
- MemberLoad is a union type: UniformDistributedLoad | PointLoad.
- AnalysisResult is a structured container for all outputs.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable


# ── Nodes ──────────────────────────────────────────────────────


@dataclass(frozen=True)
class Node:
    """A node in the structural model."""

    id: int
    x: float
    y: float


# ── Material / Section ─────────────────────────────────────────


@dataclass(frozen=True)
class Material:
    """Material and cross-section properties.

    Field order follows the conventional E-A-I ordering used in
    most structural engineering textbooks.
    """

    id: int
    E: float   # modulus of elasticity (kN/m²)
    A: float   # cross-sectional area (m²)
    I: float   # moment of inertia (m⁴)


# ── Supports ───────────────────────────────────────────────────


@dataclass(frozen=True)
class Support:
    """Boundary condition at a node.

    Booleans indicate whether a DOF is restrained (True) or free (False).
    """

    node_id: int
    ux: bool = False
    uy: bool = False
    rz: bool = False


# ── Loads ──────────────────────────────────────────────────────


@dataclass(frozen=True)
class NodalLoad:
    """Applied load at a node (global coordinates)."""

    node_id: int
    fx: float = 0.0   # force in x (kN)
    fy: float = 0.0   # force in y (kN)
    mz: float = 0.0   # moment about z (kN·m)


@dataclass(frozen=True)
class UniformDistributedLoad:
    """Full-length UDL on a member (local transverse direction).

    wy > 0 acts in the positive local-y direction of the element.
    """

    wy: float


@dataclass(frozen=True)
class PointLoad:
    """Point load on a member at distance *a* from start node.

    py > 0 acts in the positive local-y direction of the element.
    """

    py: float
    a: float


MemberLoad = UniformDistributedLoad | PointLoad


# ── Structural Model ──────────────────────────────────────────


@dataclass
class StructuralModel:
    """Complete structural model container.

    Elements are stored as a list of Element2D subclass instances
    (FrameElement2D or TrussElement2D) — see element.py.
    """

    title: str = "Untitled"
    nodes: dict[int, Node] = field(default_factory=dict)
    materials: dict[int, Material] = field(default_factory=dict)
    elements: list = field(default_factory=list)        # list[Element2D]
    supports: dict[int, Support] = field(default_factory=dict)
    nodal_loads: list[NodalLoad] = field(default_factory=list)

    # ── convenience helpers ──

    def node(self, node_id: int) -> Node:
        """Return the Node with the given id.

        Args:
            node_id: The node identifier to look up.

        Returns:
            The Node object.
        """
        return self.nodes[node_id]

    def support_for(self, node_id: int) -> Support:
        """Return the Support for a node, or an all-free default.

        Args:
            node_id: The node identifier to look up.

        Returns:
            The Support object, or Support(node_id) with all DOFs free.
        """
        return self.supports.get(node_id, Support(node_id=node_id))

    @property
    def node_ids(self) -> list[int]:
        """Sorted list of node IDs in the model.

        Returns:
            List of integer node IDs in ascending order.
        """
        return sorted(self.nodes)


# ── Analysis Result ───────────────────────────────────────────


@dataclass
class AnalysisResult:
    """Structured container for all analysis outputs."""

    status: str                                          # "ok" or "error"
    title: str = ""
    warnings: list[str] = field(default_factory=list)

    # Step B
    E_map: dict[int, dict[str, int | None]] = field(default_factory=dict)
    num_eq: int = 0
    G_vectors: dict[int, list[int | None]] = field(default_factory=dict)

    # Step C
    K: object = None   # np.ndarray — kept as object to avoid import
    F: object = None

    # Step D
    D: object = None
    residual: float = 0.0

    # Step E
    member_results: dict[int, dict] = field(default_factory=dict)

    # Step F
    reactions: dict[int, dict[str, float]] = field(default_factory=dict)
    eq_residual: float = 0.0

    # Storage
    elem_data: dict[int, dict] = field(default_factory=dict)

    # Diagnostics
    diagnostics: dict[str, object] = field(default_factory=dict)
