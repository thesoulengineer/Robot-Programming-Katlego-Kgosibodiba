"""Unit tests for PID controller."""

from core.controller import PIDController


def approx(value, expected, abs_tol=1e-6):
    """Helper for approximate equality."""
    return abs(value - expected) < abs_tol


def test_proportional_only():
    controller = PIDController(kp=1.0, ki=0.0, kd=0.0)
    output = controller.compute(error=1.0, dt=0.1)
    assert output == 1.0


def test_integral_accumulation():
    controller = PIDController(kp=0.0, ki=1.0, kd=0.0)
    controller.compute(error=1.0, dt=0.1)
    output = controller.compute(error=1.0, dt=0.1)
    assert approx(output, 0.2)


def test_derivative_response():
    controller = PIDController(kp=0.0, ki=0.0, kd=1.0)
    controller.compute(error=0.0, dt=0.1)
    output = controller.compute(error=1.0, dt=0.1)
    assert approx(output, 10.0)


def test_combined_pid():
    controller = PIDController(kp=1.0, ki=0.5, kd=0.1)
    output1 = controller.compute(error=1.0, dt=0.1)
    output2 = controller.compute(error=0.5, dt=0.1)
    
    assert approx(output1, 2.05)
    assert output2 < output1


def test_reset():
    controller = PIDController(kp=1.0, ki=1.0, kd=1.0)
    controller.compute(error=1.0, dt=0.1)
    controller.reset()
    
    assert controller.integral == 0.0
    assert controller.previous_error == 0.0


def test_zero_dt():
    controller = PIDController(kp=1.0, ki=1.0, kd=1.0)
    output = controller.compute(error=1.0, dt=0.0)
    assert approx(output, 1.0)
