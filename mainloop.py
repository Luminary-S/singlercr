#!./venv/python
# import sympy
from ur_move_backup import *

def AutoLoop():

    def __init__(self, ur):
        feature_pos = [ 0,0,0 ]
        self.dist = 999
        self.UR = UR(0)

    def loop(self):
        #1. rcr goes to designate height, manually operating
        # rcr_get_height()
        #2. get sonic value
        self.dist = self.get_dist_to_window()
        #3. adjust base cam to get world coodinate
        base_coordinate = self.get_ur_base_xyzposition()
        #4. set ur init position


    def get_dist_to_window(self):
        pass

    def get_ur_base_xyzposition(self):
        pass


