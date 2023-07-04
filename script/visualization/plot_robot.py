import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), ".."))

from user_interface.matrixIO import read_data_from_txt

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

def read_obj(fname):
    with open(fname, mode='rb') as file:
        print(file.read())

if __name__ == "__main__":
    read_obj("link1.STL")
