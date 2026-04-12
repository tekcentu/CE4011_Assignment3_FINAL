from pathlib import Path
import subprocess
import sys

project_root = Path('/mnt/data/claude_proj')
input_file = project_root / 'inputs' / 'q3c_vertical_frame_supported_fixed.txt'
output_file = project_root / 'outputs' / 'q3c_vertical_frame_supported_fixed_output.txt'

cmd = [sys.executable, '-m', 'structural_analysis.main', str(input_file)]
res = subprocess.run(cmd, cwd=project_root, capture_output=True, text=True)
output_file.write_text(res.stdout + ('\n' + res.stderr if res.stderr else ''))
print(f'Wrote {output_file}')
print(res.stdout)
if res.stderr:
    print('STDERR:')
    print(res.stderr)
