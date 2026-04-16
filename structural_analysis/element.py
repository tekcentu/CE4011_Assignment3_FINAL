"""
Element-level computations using OOP inheritance.

Class hierarchy
---------------
    Element2D  (abstract base)
    ├── FrameElement2D   — 6-DOF beam-column with axial + flexural stiffness
    └── TrussElement2D   — axial-only (zero flexural stiffness)

Sign convention for consistent load vector *p*
-----------------------------------------------
``p`` represents the **equivalent nodal forces that do the same work as
the distributed/point member load** (energy-consistent convention).
During assembly: ``F[dof] += p[dof]``.
During force recovery: ``q = k · d − p``.
"""

from __future__ import annotations
from dataclasses import dataclass, field
import numpy as np
from .model import (
    FrameTemperatureLoad,
    MemberLoad,
    PointLoad,
    TrussTemperatureLoad,
    UniformDistributedLoad,
)


def _length_cos_sin(ni, nj) -> tuple[float, float, float]:
    """Compute element length and direction cosines.

    Args:
        ni: Start node (must have .x, .y attributes).
        nj: End node (must have .x, .y attributes).

    Returns:
        Tuple (L, c, s) where L is length, c = cos θ, s = sin θ.

    Raises:
        ValueError: If the element has zero length.
    """
    dx = nj.x - ni.x
    dy = nj.y - ni.y
    L = float(np.hypot(dx, dy))
    if L < 1e-12:
        raise ValueError(f"Zero-length element between ({ni.x},{ni.y}) and ({nj.x},{nj.y}).")
    return L, dx / L, dy / L


def _rotation_matrix_6x6(c: float, s: float) -> np.ndarray:
    """Build the 6×6 block-diagonal rotation matrix R = diag(T, T).

    Args:
        c: Cosine of the element inclination angle (dx/L).
        s: Sine of the element inclination angle (dy/L).

    Returns:
        6×6 numpy array — the rotation matrix R.
    """
    R = np.zeros((6, 6))
    T = np.array([[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]])
    R[0:3, 0:3] = T
    R[3:6, 3:6] = T
    return R


@dataclass
class Element2D:
    """Abstract base class for 2-D structural elements.

    All element types share the same 6-DOF local convention:
    [u_i, v_i, θ_i, u_j, v_j, θ_j].

    Attributes:
        id: Unique element identifier.
        node_i: Start node ID.
        node_j: End node ID.
        E: Modulus of elasticity (kN/m²).
        A: Cross-sectional area (m²).
        member_loads: List of MemberLoad objects (UDL or PointLoad).
    """

    id: int
    node_i: int
    node_j: int
    E: float
    A: float
    alpha: float = 0.0
    depth: float | None = None
    member_loads: list[MemberLoad] = field(default_factory=list)

    @property
    def kind(self) -> str:
        """Return the element type as a string.

        Returns:
            Element type identifier ("frame" or "truss").
        """
        raise NotImplementedError

    def raw_local_stiffness(self, nodes: dict) -> np.ndarray:
        """Compute the 6×6 local stiffness matrix (no releases).

        Args:
            nodes: Dict mapping node IDs to Node objects.

        Returns:
            6×6 numpy array — unreleased local stiffness k'.
        """
        raise NotImplementedError

    def length_cos_sin(self, nodes: dict) -> tuple[float, float, float]:
        """Compute element length and direction cosines.

        Args:
            nodes: Dict mapping node IDs to Node objects.

        Returns:
            Tuple (L, c, s) — length, cos θ, sin θ.
        """
        return _length_cos_sin(nodes[self.node_i], nodes[self.node_j])

    def transformation_matrix(self, nodes: dict) -> np.ndarray:
        """Build the 6×6 rotation matrix R (local → global).

        Args:
            nodes: Dict mapping node IDs to Node objects.

        Returns:
            6×6 numpy array — the rotation matrix R.
        """
        _, c, s = self.length_cos_sin(nodes)
        return _rotation_matrix_6x6(c, s)

    def local_consistent_load(self, nodes: dict) -> np.ndarray:
        """Compute the consistent load vector p in local coordinates.

        Default: zero vector. Overridden by FrameElement2D.

        Args:
            nodes: Dict mapping node IDs to Node objects.

        Returns:
            6-element numpy array — consistent load vector p.
        """
        return np.zeros(6)

    def assembly_local_indices(self) -> list[int | None]:
        """Map from element local DOFs to active DOF slots.

        None means the DOF is not assembled (e.g. truss θ).

        Returns:
            List of 6 entries: int index or None per local DOF.
        """
        return [0, 1, 2, 3, 4, 5]

    def assembled_local_stiffness_and_load(self, nodes: dict) -> tuple[np.ndarray, np.ndarray]:
        """Return stiffness and load after any release condensation.

        Args:
            nodes: Dict mapping node IDs to Node objects.

        Returns:
            Tuple (k_condensed, p_condensed) — 6×6 and 6×1 arrays.
        """
        return self.raw_local_stiffness(nodes), self.local_consistent_load(nodes)

    def global_stiffness_and_load(self, nodes: dict) -> tuple[np.ndarray, np.ndarray]:
        """Transform condensed k and p to global coordinates.

        Args:
            nodes: Dict mapping node IDs to Node objects.

        Returns:
            Tuple (k_global, p_global) — 6×6 and 6×1 arrays in global coords.
        """
        k_local, p_local = self.assembled_local_stiffness_and_load(nodes)
        R = self.transformation_matrix(nodes)
        return R.T @ k_local @ R, R.T @ p_local

    def local_displacement_and_end_forces(self, nodes: dict, u_global_elem: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Compute local displacements and member end forces.

        Args:
            nodes: Dict mapping node IDs to Node objects.
            u_global_elem: 6-element array of element global displacements.

        Returns:
            Tuple (d_local, q_local) where d_local is the 6-element local
            displacement vector and q_local is [N_i, V_i, M_i, N_j, V_j, M_j].
        """
        R = self.transformation_matrix(nodes)
        d_local = R @ u_global_elem
        q_local = self.raw_local_stiffness(nodes) @ d_local - self.local_consistent_load(nodes)
        return d_local, q_local


@dataclass
class FrameElement2D(Element2D):
    """2-D frame (beam-column) element with optional moment releases.

    Uses Schur-complement static condensation for releases, applied
    to both k and p simultaneously (exact for member loads + releases).

    Attributes:
        I: Moment of inertia (m⁴).
        release_i: If True, hinge at node i (local DOF 2).
        release_j: If True, hinge at node j (local DOF 5).
    """

    I: float = 0.0
    release_i: bool = False
    release_j: bool = False
    ex_i: float = 0.0
    ey_i: float = 0.0
    ex_j: float = 0.0
    ey_j: float = 0.0

    @property
    def kind(self) -> str:
        """Return the element type identifier.

        Returns:
            The string "frame".
        """
        return "frame"

    def raw_local_stiffness(self, nodes: dict) -> np.ndarray:
        """Compute the 6×6 local stiffness for an unreleased frame element.

        Args:
            nodes: Dict mapping node IDs to Node objects.

        Returns:
            6×6 numpy array — unreleased local stiffness using EA/L
            and 12EI/L³, 6EI/L², 4EI/L, 2EI/L terms.
        """
        L, _, _ = self.length_cos_sin(nodes)
        EA_L = self.E * self.A / L
        EI = self.E * self.I
        L2, L3 = L * L, L * L * L
        return np.array([
            [ EA_L,       0,         0,    -EA_L,        0,         0],
            [    0, 12*EI/L3,  6*EI/L2,        0, -12*EI/L3,  6*EI/L2],
            [    0,  6*EI/L2,  4*EI/L,         0,  -6*EI/L2,  2*EI/L ],
            [-EA_L,       0,         0,     EA_L,        0,         0],
            [    0,-12*EI/L3, -6*EI/L2,        0,  12*EI/L3, -6*EI/L2],
            [    0,  6*EI/L2,  2*EI/L,         0,  -6*EI/L2,  4*EI/L ],
        ])

    def local_consistent_load(self, nodes: dict) -> np.ndarray:
        """Compute equivalent nodal loads from member loads (fixed-fixed).

        Uses Hermitian shape functions for point loads and wL/2, wL²/12
        for UDL. Release condensation is applied separately in
        assembled_local_stiffness_and_load().

        Args:
            nodes: Dict mapping node IDs to Node objects.

        Returns:
            6-element numpy array — consistent load vector p.

        Raises:
            ValueError: If point load position is outside member length.
            TypeError: If unsupported member load type is encountered.
        """
        L, _, _ = self.length_cos_sin(nodes)
        p = np.zeros(6)
        for load in self.member_loads:
            if isinstance(load, UniformDistributedLoad):
                w = load.wy
                p += np.array([0, w*L/2, w*L**2/12, 0, w*L/2, -w*L**2/12])
            elif isinstance(load, PointLoad):
                a = float(load.a)
                if not (0 <= a <= L + 1e-10):
                    raise ValueError(f"Element {self.id}: point load a={a:.3f} outside L={L:.3f}.")
                xi = a / L if L > 0 else 0
                n1 = 1 - 3*xi**2 + 2*xi**3
                n2 = L*(xi - 2*xi**2 + xi**3)
                n3 = 3*xi**2 - 2*xi**3
                n4 = L*(-xi**2 + xi**3)
                p += np.array([0, load.py*n1, load.py*n2, 0, load.py*n3, load.py*n4])
            else:
                raise TypeError(f"Unsupported load on element {self.id}: {type(load)}")
        return p

    def _released_dofs(self) -> list[int]:
        """Return indices of released (condensed-out) local DOFs.

        Returns:
            List of int — indices 2 and/or 5, or empty if no releases.
        """
        r: list[int] = []
        if self.release_i: r.append(2)
        if self.release_j: r.append(5)
        return r

    def rigid_offset_matrix(self) -> np.ndarray:
        """Return 6x6 rigid-end-offset transformation matrix T."""
        T = np.eye(6)
        T[2, 0] = self.ey_i
        T[2, 1] = -self.ex_i
        T[5, 3] = self.ey_j
        T[5, 4] = -self.ex_j
        return T

    def local_thermal_load(self, load: FrameTemperatureLoad, depth: float, alpha: float) -> np.ndarray:
        """Equivalent local nodal thermal load from through-depth gradient."""
        if depth <= 0:
            raise ValueError(f"Frame element {self.id}: section depth must be positive for thermal gradient.")
        kappa = alpha * (float(load.t_top) - float(load.t_bottom)) / depth
        m = self.E * self.I * kappa
        return np.array([0.0, 0.0, m, 0.0, 0.0, -m])

    def assembly_local_indices(self) -> list[int | None]:
        """Mark released rotational DOFs as None (not assembled).

        Returns:
            List of 6 entries where released DOFs are None.
        """
        m: list[int | None] = [0, 1, 2, 3, 4, 5]
        if self.release_i: m[2] = None
        if self.release_j: m[5] = None
        return m

    def assembled_local_stiffness_and_load(self, nodes: dict) -> tuple[np.ndarray, np.ndarray]:
        """Apply Schur-complement static condensation for moment releases.

        Condenses both stiffness and load simultaneously:
            k_c = k_aa − k_ab · k_bb⁻¹ · k_ba
            p_c = p_a  − k_ab · k_bb⁻¹ · p_b

        Args:
            nodes: Dict mapping node IDs to Node objects.

        Returns:
            Tuple (k_condensed, p_condensed) — 6×6 and 6-element arrays
            with zeros at released DOF positions.
        """
        k_raw = self.raw_local_stiffness(nodes)
        p_raw = self.local_consistent_load(nodes)
        T = self.rigid_offset_matrix()
        k = T @ k_raw @ T.T
        p = T @ p_raw
        released = self._released_dofs()
        if not released:
            return k, p
        retained = [i for i in range(6) if i not in released]
        kaa = k[np.ix_(retained, retained)]
        kab = k[np.ix_(retained, released)]
        kba = k[np.ix_(released, retained)]
        kbb = k[np.ix_(released, released)]
        kbb_inv = np.linalg.inv(kbb)
        k_out = np.zeros_like(k)
        p_out = np.zeros_like(p)
        k_out[np.ix_(retained, retained)] = kaa - kab @ kbb_inv @ kba
        p_out[retained] = p[retained] - kab @ kbb_inv @ p[released]
        return k_out, p_out

    def local_displacement_and_end_forces(self, nodes: dict, u_global_elem: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Recover local displacements and end forces (with back-substitution).

        For released elements, condensed-out rotational DOFs are recovered:
            d_b = k_bb⁻¹ · (p_b − k_ba · d_a)
        Then: q = k_full · d_full − p_full.

        Args:
            nodes: Dict mapping node IDs to Node objects.
            u_global_elem: 6-element array of element global displacements.

        Returns:
            Tuple (d_local, q_local) where d_local includes recovered
            released DOFs and q_local = [N_i, V_i, M_i, N_j, V_j, M_j].
        """
        R = self.transformation_matrix(nodes)
        d_master_from_global = R @ u_global_elem
        k_full_raw = self.raw_local_stiffness(nodes)
        p_full_raw = self.local_consistent_load(nodes)
        T = self.rigid_offset_matrix()
        k_full = T @ k_full_raw @ T.T
        p_full = T @ p_full_raw
        released = self._released_dofs()
        if not released:
            d_physical = T.T @ d_master_from_global
            q_physical = k_full_raw @ d_physical - p_full_raw
            return d_physical, q_physical
        retained = [i for i in range(6) if i not in released]
        kba = k_full[np.ix_(released, retained)]
        kbb = k_full[np.ix_(released, released)]
        db = np.linalg.solve(kbb, p_full[released] - kba @ d_master_from_global[retained])
        d_master = np.array(d_master_from_global, copy=True)
        d_master[released] = db
        d_physical = T.T @ d_master
        q_physical = k_full_raw @ d_physical - p_full_raw
        return d_physical, q_physical


@dataclass
class TrussElement2D(Element2D):
    """2-D truss element — axial force only.

    Rotational DOFs are marked None in assembly_local_indices()
    so the DofManager auto-suppresses Rz at pure-truss nodes.
    """

    @property
    def kind(self) -> str:
        """Return the element type identifier.

        Returns:
            The string "truss".
        """
        return "truss"

    def raw_local_stiffness(self, nodes: dict) -> np.ndarray:
        """Compute 6×6 local stiffness with only axial terms.

        Args:
            nodes: Dict mapping node IDs to Node objects.

        Returns:
            6×6 numpy array — axial-only stiffness (DOFs 0, 3 non-zero).
        """
        L, _, _ = self.length_cos_sin(nodes)
        EA_L = self.E * self.A / L
        k = np.zeros((6, 6))
        k[0,0] = EA_L; k[0,3] = -EA_L
        k[3,0] = -EA_L; k[3,3] = EA_L
        return k

    def assembly_local_indices(self) -> list[int | None]:
        """Mark rotational DOFs as inactive for truss elements.

        Returns:
            [0, 1, None, 3, 4, None] — DOFs 2 and 5 suppressed.
        """
        return [0, 1, None, 3, 4, None]

    def local_thermal_load(self, load: TrussTemperatureLoad, alpha: float) -> np.ndarray:
        """Equivalent local nodal thermal load for uniform ΔT in truss."""
        n = self.E * self.A * alpha * float(load.delta_t)
        return np.array([-n, 0.0, 0.0, n, 0.0, 0.0])
