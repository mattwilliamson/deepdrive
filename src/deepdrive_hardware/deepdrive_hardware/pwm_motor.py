#!/usr/bin/env python

import Jetson.GPIO as GPIO

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

# TODO: Jetson Orin Nano is having issues with PWM. Need to debug.
# Use full speed for now.

class MotorDriverL293:
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

    def _set_pin(self, pin, value):
        print(f"Setting pin {pin} to {value}")
        GPIO.output(pin, value)

    def set_motors(self, left_speed=0.0, right_speed=0.0):
        """Set the motor speeds. Negative values for reverse. 0.0-1.0"""
        if left_speed > 0.0:
            self._set_pin(self.left_forward, GPIO.HIGH)
            self._set_pin(self.left_backward, GPIO.LOW)
        elif left_speed < 0.0:
            self._set_pin(self.left_forward, GPIO.LOW)
            self._set_pin(self.left_backward, GPIO.HIGH)
        else:
            self._set_pin(self.left_forward, GPIO.LOW)
            self._set_pin(self.left_backward, GPIO.LOW)

        if right_speed > 0.0:
            self._set_pin(self.right_forward, GPIO.HIGH)
            self._set_pin(self.right_backward, GPIO.LOW)
        elif right_speed < 0.0:
            self._set_pin(self.right_forward, GPIO.LOW)
            self._set_pin(self.right_backward, GPIO.HIGH)
        else:
            self._set_pin(self.right_forward, GPIO.LOW)
            self._set_pin(self.right_backward, GPIO.LOW)

        print("Setting duty cycle to: ", abs(left_speed), abs(right_speed))
        left_pwm_speed = min(abs(left_speed) * 100.0, 100.0)
        right_pwm_speed = min(abs(right_speed) * 100.0, 100.0)
        self.left_pwm.ChangeDutyCycle(left_pwm_speed)
        self.right_pwm.ChangeDutyCycle(right_pwm_speed)


    def cleanup(self):
        GPIO.cleanup()
