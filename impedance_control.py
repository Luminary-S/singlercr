#!./venv/python
import sympy as sy
from ur_move import *
from sensor_frame import *
# import copy
import rospy, time

def apply_ics(sol, ics, x, known_params):
	"""
	refer: https://vlight.me/2018/05/01/Numerical-Python-Ordinary-Differential-Equations/
	Apply the initial conditions (ics), given as a dictionary on
	the form ics = {y(0): y0, y(x).diff(x).subs(x, 0): yp0, ...},
	to the solution of the ODE with independent variable x.
	The undetermined integration constants C1, C2, ... are extracted
	from the free symbols of the ODE solution, excluding symbols in
	the known_params list.
	"""
	free_params = sol.free_symbols - set(known_params)
	eqs = [(sol.lhs.diff(x, n) - sol.rhs.diff(x, n)).subs(x, 0).subs(ics)
			for n in range(len(ics))]
	sol_params = sy.solve(eqs, free_params)

	return sol.subs(sol_params)

class ImpedanceControl():

    def __init__(self):
        # self.impedance_eq = 0
        self.force = 0
        self.UR = UR(0)

    def set_impedance_eq(self, x, F, tim):
        """
        M ddot(x(t)) + C dot(x(t)) + K x(t) = Fe
        """
        M = 0
        C = 0.9
        K = 80
        X = sy.Function('X')
        # eq = M * sy.diff(X, 2) + C * sy.diff( X ) + K * X - Fe
        t,Fe = sy.symbols('t,Fe')
        eq = M * sy.diff(X(t), t, 2) + C * sy.diff( X(t), t ) + K * X(t) - Fe
        if M != 0:
            eq_sol = sy.dsolve(eq, X(t))
            itc = {X(0):x, X(t).diff(t).subs(t, 0): 0, Fe:F, t:tim}
            sol = apply_ics(eq_sol, itc, t, [Fe])
            sol = sol.rhs.subs(Fe, F)  # get the right of the equation, for left use lhs
            sol = sol.subs(t, tim)
        elif C !=0:
            eq_sol = sy.dsolve(eq, X(t))
            itc = {X(0):x}
            sol = apply_ics(eq_sol, itc, t, [Fe])
            sol = sol.rhs.subs(Fe,F) # get the right of the equation, for left use lhs
            sol = sol.subs(t,tim)
        else:
            sol = sy.solve(eq,X(t))[0].subs(Fe,F)
        print(sol)
        return sol


    def impedance_eq_solver(self, F, Fd, pos, tim):
        # itc = {}
        # t, Fe, X = sy.symbols('t,Fe,X')
        # new_pos = self.set_impedance_eq(pos,F, tim)
        # return new_pos
        stiffness = 10000.0
        delta_F = Fd - F
        delta_y = delta_F/ stiffness
        pos = pos + delta_y
        return pos

    def set_path(self):
        T_list = []

    """ 1 dim force sensor"""
    def update(self, force, Fd, T, t):
        y = T[7]
        print("y",y)
        yd = y
        if force == 0 :
            yd -= 0.02 # forward 2mm
        else:
            # calculate the val of y by solving impedance equations
            yd = self.impedance_eq_solver( force, Fd, y, t)
        T[7] = yd
        print("yd:",yd)
        return T

def test():
    ur = UR(0)
    imCtr = ImpedanceControl()
    q = ur.get_q()
    pre_T = ur.kin.Forward(q)
    # T = [0.004025274891437544, 0.9989214784093534, -0.04625664422026579, -0.09727600042384893, -0.9973643664965092, 0.007361466490554639, 0.07218122508052671, -0.4251659020558615, 0.07244389280723103, 0.04584417938604993, 0.9963183194197295, 0.14566788765371075, 0.0, 0.0, 0.0, 1.0]
    T = imCtr.update( 100, 60, pre_T, 0.01 )
    print(T)
    # q = ur.get_q()
    qd = ur.solve_q(q,T)
    qd = getDegree(qd)
    print(q)
    print(qd)

def main():
    #1. make the EE vertical to the surface
    ur = UR(0)
    ur.Init_node()
    imCtr = ImpedanceControl()

    # imCtr.impedance_eq_solver()
    sensorT = SensorThread()
    sensorT.start()
    sensorT.sensor.initialize_sensor("/dev/ttyUSB0", 9600)
    sensorT.resume()

    force_Threshold = 70

    ratet = 30
    t = 0
    vel = 0.5
    ace = 50
    rate = rospy.Rate(ratet)

    while not rospy.is_shutdown() :
        print("sonic:", sensorT.sensor.sonic1)
        print("force:", sensorT.sensor.force1)
        force = sensorT.sensor.force1
        while force < force_Threshold:
            # 1. get data: ur T pos and force
            T = ur.get_T()
            q = ur.get_q()
            t = 1.0 / ratet
            # force = sensorT.sensor.force1
            imCtr.update(force, force_Threshold, T,  t )
            qd = ur.solve_q(q, T)
            ur.ur_move_to_point(ur.pub, qd)
            # imCtr.impedance_eq_solver()
                # time.sleep(0.009)
        rate.sleep()



if __name__ == '__main__':
    main()
    # test()