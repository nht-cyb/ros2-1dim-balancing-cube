#!/usr/bin/python3
"""Re-run the best tunings from tune_controller.py with many seeds (mean +- 95% CI).

    python3 scripts/confirm_tuning.py sim      # or: hw
"""
import os, sys
from multiprocessing import Pool

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
import tune_controller as tc  # noqa: E402

prof = sys.argv[1] if len(sys.argv) > 1 else 'sim'
mt = tc.PROFILES[prof]
cands = {
 'baseline Q=(100,1,1e-4) tau=0.3':      ((100.,1.,1e-4),100.,0.3,0),
 'A Q=(300,0.3,1e-4) tau=0.5':           ((300.,0.3,1e-4),100.,0.5,0),
 'B Q=(100,0.3,1e-4) tau=0.5':           ((100.,0.3,1e-4),100.,0.5,0),
 'C Q=(100,1,1e-4) tau=0.5':             ((100.,1.,1e-4),100.,0.5,0),
 'D Q=(100,0.3,1e-4) tau=0.5 delay1':    ((100.,0.3,1e-4),100.,0.5,1),
}
def job(a):
    name,t,seed,d = a
    return name, d, tc.simulate(t, seed, delay=d, jitter=2, max_torque=mt)
jobs=[(n,t,s,d) for n,t in cands.items() for s in range(20) for d in (1,2,3)]
if __name__ == '__main__':
    with Pool() as pool: res=pool.map(job, jobs, chunksize=4)
    print(f"profile {prof}, 20 seeds x delays 1-3 (+0-2 steps jitter), mean ± 95% CI")
    for n in cands:
        rs=[r for nn,d,r in res if nn==n]; ok=[r for r in rs if not r['fell']]
        def m(k):
            v=np.array([r[k] for r in ok]); return f"{v.mean():.3f}±{1.96*v.std()/np.sqrt(len(v)):.3f}"
        print(f"{n:36s} falls {len(rs)-len(ok)}/{len(rs)}  tilt_rms {m('tilt_rms')}°  du_rms {m('du_rms')} mNm  peak {m('peak')}°  settle {m('settle')} s")
