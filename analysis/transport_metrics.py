#!/usr/bin/env python3
"""Compute 1D lateral tracking metrics from a CSV of measured PAT-board poses.

Required CSV columns: x_cm,y_cm; the y reference is 40 cm in the submitted paper.
If all 15 trial logs are supplied, calculate per-trial metrics separately before
computing across-trial estimates. The manuscript's Fig. 7b uses 11 trajectory samples.
"""
from __future__ import annotations
import argparse, csv, math
from pathlib import Path


def metrics(ys:list[float], reference_y:float=40.0):
    if not ys: raise ValueError('At least one measured point is required')
    deviations=[abs(v-reference_y) for v in ys]
    return {
        'n':len(ys),
        'mean_absolute_lateral_deviation_cm':sum(deviations)/len(deviations),
        'maximum_absolute_lateral_deviation_cm':max(deviations),
        'lateral_rmse_cm':math.sqrt(sum(x*x for x in deviations)/len(deviations)),
    }

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('csv_path',type=Path)
    p.add_argument('--reference-y-cm',type=float,default=40.0)
    a=p.parse_args()
    with a.csv_path.open(newline='',encoding='utf-8-sig') as f:
        rows=list(csv.DictReader(f))
    ys=[float(row['y_cm']) for row in rows]
    result=metrics(ys,a.reference_y_cm)
    for key,value in result.items():print(f'{key}: {value:.6f}' if isinstance(value,float) else f'{key}: {value}')

if __name__=='__main__':main()
