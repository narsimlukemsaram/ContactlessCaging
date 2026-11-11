import math
from contactlesscaging_control.control_math import PID2D, planar_to_cmd, wrap_pi

def test_signed_differential_drive():
    v,w=planar_to_cmd(0.05,0,math.pi,backward=True,vmax=.05,omega_max=1.0,heading_gain=1.0)
    assert math.isclose(v,-.05,rel_tol=1e-8)
    assert math.isclose(w,0.0,abs_tol=1e-9)
    v,w=planar_to_cmd(0.05,0,0,backward=False,vmax=.05,omega_max=1.0,heading_gain=1.0)
    assert math.isclose(v,.05,rel_tol=1e-8)

def test_heading_guard_stops_forward_thrust():
    v,w=planar_to_cmd(0.05,0,math.pi,backward=False,vmax=.05,omega_max=1,heading_gain=1)
    assert v == 0.0
    assert abs(w) <= 1.0

def test_pid_reset_and_bounds():
    pid=PID2D(kp=1,ki=1,kd=0,integral_limit=.2)
    for _ in range(100): pid.update(1.0,0.0,.05)
    assert pid.ix <= .2
    pid.reset();assert pid.ix==0
    assert abs(wrap_pi(math.pi)-(-math.pi))<1e-9
