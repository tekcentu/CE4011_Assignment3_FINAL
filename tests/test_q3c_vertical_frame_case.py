from pathlib import Path
import sys

PROJECT_ROOT = Path('/mnt/data/claude_proj')
sys.path.insert(0, str(PROJECT_ROOT))

from structural_analysis.file_io import read_input_file
from structural_analysis.main import run_analysis


def test_q3c_vertical_frame_supported_fixed():
    model = read_input_file(str(PROJECT_ROOT / 'inputs' / 'q3c_vertical_frame_supported_fixed.txt'))
    result = run_analysis(model, verbose=False)

    assert result.status == 'ok'
    assert any('disconnected but supported components' in w.lower() for w in result.warnings)

    # Reaction balance: support vertical reactions must balance the 15 kN load
    total_ry = sum(r.get('uy', 0.0) for r in result.reactions.values())
    assert abs(total_ry - 15.0) < 1e-6

    # The loaded left substructure should move; the unloaded right cantilever should remain at zero.
    n2 = result.E_map[2]
    n3 = result.E_map[3]
    n5 = result.E_map[5]

    ux2 = result.D[n2['ux']]
    uy2 = result.D[n2['uy']]
    rz2 = result.D[n2['rz']]
    ux3 = result.D[n3['ux']]
    rz3 = result.D[n3['rz']]
    ux5 = result.D[n5['ux']]
    uy5 = result.D[n5['uy']]
    rz5 = result.D[n5['rz']]

    assert uy2 < 0.0
    assert abs(ux2) > 0.0
    assert abs(rz2) > 0.0
    assert abs(ux3) > 0.0
    assert abs(rz3) > 0.0

    assert abs(ux5) < 1e-12
    assert abs(uy5) < 1e-12
    assert abs(rz5) < 1e-12
