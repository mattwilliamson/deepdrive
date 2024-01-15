#!/usr/bin/env python

import RPi.GPIO as GPIO

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)


class MotorDiffDrive:
    def __init__(
        self,
        left_enable=32,
        left_forward=38,
        left_backward=37,
        right_enable=33,
        right_forward=35,
        right_backward=36,
        hz=50,
    ):
        self.left_enable = left_enable
        self.left_forward = left_forward
        self.left_backward = left_backward
        self.right_enable = right_enable
        self.right_forward = right_forward
        self.right_backward = right_backward
        self.hz = hz
        self.init_pins()

    def init_pins(self):
        GPIO.setup(self.left_enable, GPIO.OUT)
        GPIO.setup(self.left_forward, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.left_backward, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.right_enable, GPIO.OUT)
        GPIO.setup(self.right_forward, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.right_backward, GPIO.OUT, initial=GPIO.LOW)

        # Turn on PWM for pin control
        self.left_pwm = GPIO.PWM(self.left_enable, self.hz)
        self.right_pwm = GPIO.PWM(self.right_enable, self.hz)
        self.left_pwm.start(0)
        self.right_pwm.start(0)

    def set_motors(self, left_speed=0.0, right_speed=0.0):
        """Set the motor speeds. Negative values for reverse."""
        if left_speed > 0.0:
            GPIO.output(self.left_forward, GPIO.HIGH)
            GPIO.output(self.left_backward, GPIO.LOW)
        elif left_speed < 0.0:
            GPIO.output(self.left_forward, GPIO.LOW)
            GPIO.output(self.left_backward, GPIO.HIGH)
        else:
            GPIO.output(self.left_forward, GPIO.LOW)
            GPIO.output(self.left_backward, GPIO.LOW)

        if right_speed > 0.0:
            GPIO.output(self.right_forward, GPIO.HIGH)
            GPIO.output(self.right_backward, GPIO.LOW)
        elif right_speed < 0.0:
            GPIO.output(self.right_forward, GPIO.LOW)
            GPIO.output(self.right_backward, GPIO.HIGH)
        else:
            GPIO.output(self.right_forward, GPIO.LOW)
            GPIO.output(self.right_backward, GPIO.LOW)

        self.left_pwm.ChangeDutyCycle(abs(left_speed))
        self.right_pwm.ChangeDutyCycle(abs(right_speed))


    def cleanup(self):
        GPIO.cleanup()
