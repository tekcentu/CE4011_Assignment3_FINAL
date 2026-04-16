import numpy as np
from numpy.testing import assert_allclose

from structural_analysis.model import StructuralModel, Node, Material, Support, TrussTemperatureLoad
from structural_analysis.element import TrussElement2D
from structural_analysis.main import run_analysis


def test_truss_uniform_temperature_generates_expected_reactions_fixed_bar():
    m = StructuralModel(title="Truss Thermal")
    m.nodes = {1: Node(1, 0.0, 0.0), 2: Node(2, 2.0, 0.0)}
    mat = Material(1, E=200000.0, A=0.01, I=0.0, alpha=1.2e-5)
    m.materials = {1: mat}
    m.elements = [
        TrussElement2D(1, 1, 2, E=mat.E, A=mat.A, alpha=mat.alpha),
    ]
    m.supports = {
        1: Support(1, ux=True, uy=True),
        2: Support(2, ux=True, uy=True),
    }
    m.truss_temperature_loads = [TrussTemperatureLoad(element_id=1, delta_t=30.0)]

    r = run_analysis(m, verbose=False)
    assert r.status == "ok"

    n = mat.E * mat.A * mat.alpha * 30.0
    assert_allclose(r.reactions[1]["ux"], n, atol=1e-9)
    assert_allclose(r.reactions[2]["ux"], -n, atol=1e-9)
