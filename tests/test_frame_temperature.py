from numpy.testing import assert_allclose

from structural_analysis.model import StructuralModel, Node, Material, Support, FrameTemperatureLoad
from structural_analysis.element import FrameElement2D
from structural_analysis.main import run_analysis


def test_frame_thermal_gradient_generates_equal_opposite_end_moments_when_fixed_fixed():
    m = StructuralModel(title="Frame Thermal Gradient")
    m.nodes = {1: Node(1, 0.0, 0.0), 2: Node(2, 4.0, 0.0)}
    mat = Material(1, E=200000.0, A=0.02, I=0.08, alpha=1.0e-5, depth=0.5)
    m.materials = {1: mat}
    m.elements = [
        FrameElement2D(1, 1, 2, E=mat.E, A=mat.A, I=mat.I, alpha=mat.alpha, depth=mat.depth),
    ]
    m.supports = {
        1: Support(1, ux=True, uy=True, rz=True),
        2: Support(2, ux=True, uy=True, rz=True),
    }
    m.frame_temperature_loads = [FrameTemperatureLoad(element_id=1, t_top=40.0, t_bottom=10.0)]

    r = run_analysis(m, verbose=False)
    assert r.status == "ok"

    m_th = mat.E * mat.I * mat.alpha * ((40.0 - 10.0) / mat.depth)
    assert_allclose(r.reactions[1]["rz"], -m_th, atol=1e-9)
    assert_allclose(r.reactions[2]["rz"], m_th, atol=1e-9)
