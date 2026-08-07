from time import sleep_ms

from machine import PWM, Pin


class Servo:
    """Driver for a servo motor using PWM."""

    def __init__(self, pwm: PWM) -> None:
        """Init."""
        self.pwm = pwm

    def set_angle(self, angle: int) -> None:
        """Move the servo motor using PWM.

        Map angle (0-180) to duty cycle
        Typical range: 0.5ms (0°) to 2.5ms (180°) pulse width
        """
        min_duty = 500_000  # 0.5 ms
        max_duty = 2_500_000  # 2.5 ms
        duty = int(min_duty + (angle / 180) * (max_duty - min_duty))
        self.pwm.duty_ns(duty)


servo_pwm = PWM(Pin(22), freq=50)
servo = Servo(servo_pwm)

while True:
    servo.set_angle(0)
    sleep_ms(2000)
    servo.set_angle(90)
    sleep_ms(2000)
    servo.set_angle(180)
    sleep_ms(2000)
