"""
Solver: partitioned K·D = F with SVD-based singularity detection.

The system is solved using the partition approach:
- Extract K_ff (free-free block) and F_f (free load block)
- Check rank via SVD — if rank-deficient, identify mechanism DOFs
  from the null-space vector
- Check condition number — warn if ill-conditioned
- Solve K_ff · D_f = F_f
- Expand D_f back to full DOF vector (restrained DOFs = 0)
"""

from __future__ import annotations

import numpy as np

from .assembler import DofManager


def solve_system(
    K: np.ndarray,
    F: np.ndarray,
    dofs: DofManager,
    D_prescribed: np.ndarray | None = None,
) -> tuple[np.ndarray, float, list[str]]:
    """Solve the partitioned system K_ff · D_f = F_f − K_fr · D_r.

    Extracts the free-free block of K, applies prescribed restrained
    displacements (support settlements) to modify the RHS, checks rank
    via SVD to detect mechanisms, checks condition number, then solves
    and expands the solution back to the full DOF vector.

    Args:
        K: Full global stiffness matrix, ndarray (n × n).
        F: Full global load vector, ndarray (n,).
        dofs: DofManager with free/restrained index lists.
        D_prescribed: Optional ndarray (n,) with prescribed displacements
            at restrained DOFs (support settlements). None means all zeros.

    Returns:
        Tuple (D, residual, warnings) where:
            D: Full displacement vector, ndarray (n,).
            residual: float — ||K_ff · D_f − F_eff||.
            warnings: list[str] — any warnings or errors encountered.
    """
    warnings: list[str] = []
    n = K.shape[0]

    if not dofs.free_indices:
        warnings.append("No free DOFs — structure is fully restrained.")
        return np.zeros(n), 0.0, warnings

    free = dofs.free_indices
    restrained = dofs.restrained_indices
    Kff = K[np.ix_(free, free)]
    Ff = F[free]

    # Build D_r from prescribed settlements (or zeros)
    Dr = np.zeros(len(restrained))
    if D_prescribed is not None and len(restrained) > 0:
        Dr = D_prescribed[restrained]

    # Modify RHS for non-zero restrained DOFs
    if len(restrained) > 0 and np.any(Dr != 0.0):
        Kfr = K[np.ix_(free, restrained)]
        Ff = Ff - Kfr @ Dr

    # ── SVD rank check ──
    try:
        u, s, vh = np.linalg.svd(Kff, full_matrices=False)
    except np.linalg.LinAlgError:
        warnings.append("ERROR: SVD failed on K_ff.")
        return np.full(n, np.nan), float("inf"), warnings

    tol = max(Kff.shape) * np.max(s) * 1e-12
    rank = int(np.sum(s > tol))

    if rank < Kff.shape[0]:
        # Identify mechanism DOFs from null-space
        null_vec = vh[-1]
        dominant = np.argsort(np.abs(null_vec))[::-1][:3]
        dominant_labels = [
            dofs.labels[free[i]]
            for i in dominant
            if abs(null_vec[i]) > 1e-6
        ]
        warnings.append(
            f"ERROR: Singular stiffness matrix (rank {rank}/{Kff.shape[0]}). "
            f"Structure is unstable. "
            f"Mechanism DOFs: {', '.join(dominant_labels) if dominant_labels else 'undetermined'}."
        )
        return np.full(n, np.nan), float("inf"), warnings

    # ── Condition number check ──
    cond = float(s[0] / s[-1]) if s[-1] > 0 else float("inf")
    if cond > 1e12:
        warnings.append(
            f"WARNING: Ill-conditioned K (cond ≈ {cond:.3e}). "
            f"Results may be unreliable."
        )

    # ── Solve ──
    try:
        Df = np.linalg.solve(Kff, Ff)
    except np.linalg.LinAlgError:
        warnings.append("ERROR: Solve failed — singular matrix.")
        return np.full(n, np.nan), float("inf"), warnings

    # Expand to full vector
    D = np.zeros(n)
    D[free] = Df
    if len(restrained) > 0:
        D[restrained] = Dr

    residual = float(np.linalg.norm(Kff @ Df - Ff))
    if residual > 1e-3:
        warnings.append(
            f"WARNING: Large residual ||K·D − F|| = {residual:.4e}."
        )

    return D, residual, warnings
