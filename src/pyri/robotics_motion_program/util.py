from RobotRaconteurCompanion.Util.RobotUtil import RobotUtil
from RobotRaconteurCompanion.Util.GeometryUtil import GeometryUtil
import RobotRaconteur as RR
import numpy as np
import general_robotics_toolbox as rox
import pickle
import copy
from typing import NamedTuple

class Transform_all(object):
	def __init__(self, p_all, R_all):
		self.R_all=np.array(R_all)
		self.p_all=np.array(p_all)



def fill_curve_plot(ax, curve, nominal_curve = None):
    ax.plot(curve[:,0],curve[:,1],curve[:,2])
        
    pos_min = curve.min(axis=0)
    pos_max = curve.max(axis=0)
    if nominal_curve is not None:
        nom_pos_min = nominal_curve.min(axis=0)
        nom_pos_max = nominal_curve.max(axis=0)

        pos_min = np.vstack((pos_min,nom_pos_min)).min(axis=0)
        pos_max = np.vstack((pos_max,nom_pos_max)).max(axis=0)

        ax.plot(nominal_curve[:,0],nominal_curve[:,1],nominal_curve[:,2],color='red')
        ax.legend(["Output", "Nominal"])

    pos_diff = (pos_max - pos_min).max()
    pos_center = (pos_max + pos_min) / 2.0
    ax_min = pos_center - pos_diff/2.0
    ax_max = pos_center + pos_diff/2.0
    ax.set_xlim(left=ax_min[0],right=ax_max[0])
    ax.set_ylim(bottom=ax_min[1],top=ax_max[1])
    ax.set_zlim(bottom=ax_min[2],top=ax_max[2])