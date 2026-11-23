"""Replay the same saved trajectory at 10 fps with lighter rendering settings."""
import argparse
import re
import sys
from pathlib import Path

parser=argparse.ArgumentParser()
parser.add_argument('--workdir',type=Path,required=True)
args=parser.parse_args()
work=args.workdir.expanduser().resolve()
original=work/'microduck-playground/hardware/microdock/sim/render_preview.py'
source=original.read_text(encoding='utf-8')
replacements={
 'model.vis.quality.shadowsize = 8192':'model.vis.quality.shadowsize = 1024',
 'else list(range(n))':'else list(range(0, n, 5))',
 'frames, fps=50,':'frames, fps=10,',
 'duration=125':'duration=100',
}
for old,new in replacements.items():
    if source.count(old)!=1:
        raise RuntimeError('Upstream rendering code differs: '+old)
    source=source.replace(old,new)
source,n=re.subn(r'    g = \[Image\.fromarray\(f\).*\n',
 '    g = [Image.fromarray(f).resize((320, 320), Image.LANCZOS) for f in frames]\n',source)
if n!=1:
    raise RuntimeError('Upstream GIF rendering code differs')
sys.path.insert(0,str(original.parent))
sys.argv=[str(original),str(work/'results/full.pkl'),str(work/'results/preview_class.mp4'),
          '--size','480','--end','0.06,0.0,0.085,0.62','--gif_to','end']
namespace={'__file__':str(original),'__name__':'__main__'}
try:
    exec(compile(source,str(original)+' [classroom-render]','exec'),namespace)
finally:
    if 'r' in namespace:
        namespace['r'].close()
