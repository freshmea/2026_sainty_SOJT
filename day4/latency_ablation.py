"""Run upstream undocking with only its latency wrapper disabled."""
import argparse
import runpy
import sys
from pathlib import Path

parser=argparse.ArgumentParser()
parser.add_argument('--workdir',type=Path,required=True)
parser.add_argument('--seed',default='41')
parser.add_argument('--n',default='10')
parser.add_argument('--out',required=True)
args=parser.parse_args()
script=args.workdir/'microduck-playground/hardware/microdock-v2/sim/undocking_seated.py'
sys.path.insert(0,str(script.parent))
import latency
latency.enable=lambda *a,**kw:None
sys.argv=[str(script),'--seeds',args.seed,'--n',args.n,'--mag','4','--out',args.out]
runpy.run_path(str(script),run_name='__main__')
