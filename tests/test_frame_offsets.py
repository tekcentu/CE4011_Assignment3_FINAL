import numpy as np
from numpy.testing import assert_allclose

from structural_analysis.model import Node
from structural_analysis.element import FrameElement2D


NODES = {1: Node(1, 0.0, 0.0), 2: Node(2, 6.0, 0.0)}


def test_offset_matrix_is_identity_when_offsets_zero():
    e = FrameElement2D(1, 1, 2, E=200000.0, A=0.02, I=0.08)
    assert_allclose(e.rigid_offset_matrix(), np.eye(6), atol=1e-12)


def test_offset_modified_stiffness_remains_symmetric():
    e = FrameElement2D(1, 1, 2, E=200000.0, A=0.02, I=0.08, ex_i=0.1, ey_i=0.2, ex_j=-0.05, ey_j=0.15)
    k, _ = e.assembled_local_stiffness_and_load(NODES)
    assert_allclose(k, k.T, atol=1e-10)


def test_zero_offsets_matches_original_stiffness():
    base = FrameElement2D(1, 1, 2, E=200000.0, A=0.02, I=0.08)
    k_raw = base.raw_local_stiffness(NODES)
    k_asm, _ = base.assembled_local_stiffness_and_load(NODES)
    assert_allclose(k_asm, k_raw, atol=1e-12)


def test_nonzero_offsets_change_stiffness():
    base = FrameElement2D(1, 1, 2, E=200000.0, A=0.02, I=0.08)
    off = FrameElement2D(1, 1, 2, E=200000.0, A=0.02, I=0.08, ey_i=0.2)
    k0, _ = base.assembled_local_stiffness_and_load(NODES)
    k1, _ = off.assembled_local_stiffness_and_load(NODES)
    assert np.linalg.norm(k1 - k0) > 0.0
