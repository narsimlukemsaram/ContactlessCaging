import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'analysis'))
sys.path.insert(0,str(ROOT/'ros2_ws'/'src'/'contactlesscaging_control'))
from contactlesscaging_control.control_math import planar_to_cmd,PID2D
from transport_metrics import metrics

def test_metrics():
    result=metrics([39.0,40.0,42.0],40.0)
    assert result['n']==3
    assert abs(result['mean_absolute_lateral_deviation_cm']-1.0)<1e-9
    assert abs(result['maximum_absolute_lateral_deviation_cm']-2.0)<1e-9
    assert abs(result['lateral_rmse_cm']-(5/3)**.5)<1e-9

def test_leader_and_follower():
    import math
    leader=planar_to_cmd(.05,0,math.pi,backward=True,vmax=.05,omega_max=.8,heading_gain=1.0)
    follower=planar_to_cmd(.05,0,0,backward=False,vmax=.05,omega_max=.8,heading_gain=1.0)
    assert abs(leader[0]+.05)<1e-9 and abs(follower[0]-.05)<1e-9
    assert PID2D(1,0,0).update(.05,0,.05)[0]==.05
