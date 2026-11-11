#!/usr/bin/env python3
"""Plot measured 1D microphone pressure amplitude profiles from a user-supplied CSV.

Required: scan_position_mm, robot1_pa, robot2_pa, robot3_pa, robot4_pa, four_robots_pa.
Missing values may be blank. No measured samples or inferred 2D maps are generated.
Optional 'sample_origin' column ('measured'/'extrapolated') marks end points separately.
"""
from __future__ import annotations
import argparse,csv
from pathlib import Path

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('csv_path',type=Path)
    p.add_argument('--output',type=Path,default=Path('pressure_profiles.png'))
    a=p.parse_args()
    import matplotlib.pyplot as plt
    with a.csv_path.open(newline='',encoding='utf-8-sig') as f:rows=list(csv.DictReader(f))
    if not rows:raise ValueError('No rows in input CSV')
    columns=[('robot1_pa','Robot 1'),('robot2_pa','Robot 2'),('robot3_pa','Robot 3'),
             ('robot4_pa','Robot 4'),('four_robots_pa','Four robots')]
    fig,ax=plt.subplots(figsize=(9,5))
    for col,label in columns:
        xs=[];ys=[];extra_x=[];extra_y=[]
        for r in rows:
            if not r.get(col): continue
            x=float(r['scan_position_mm']);y=float(r[col])
            if (r.get('sample_origin') or 'measured').strip().lower()=='extrapolated':
                extra_x.append(x);extra_y.append(y)
            else:xs.append(x);ys.append(y)
        ax.plot(xs,ys,marker='o',markersize=2.5,label=label)
        if extra_x:ax.scatter(extra_x,extra_y,marker='x',s=28)
    ax.set(xlabel='Scan position (mm)',ylabel='Acoustic pressure amplitude (Pa)')
    ax.legend(ncol=2);ax.grid(alpha=.25);fig.tight_layout()
    fig.savefig(a.output,dpi=220)
    print(a.output)

if __name__=='__main__':main()
