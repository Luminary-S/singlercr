#!./venv/python
import sympy

def servo_control(line, q1, q2):
    pt1 = [line[0], line[1]]
    pt2 = [line[2], line[3]]
    