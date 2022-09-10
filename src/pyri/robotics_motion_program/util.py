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


class abb6640(object):
	#default tool paintgun
	def __init__(self,R_tool=rox.rot(rox.ey,np.radians(120)),p_tool=np.array([0.45,0,-0.05])*1000.,d=0,acc_dict_path=''):
		###ABB IRB 6640 180/2.55 Robot Definition
		self.H=np.concatenate(([rox.ez],[rox.ey],[rox.ey],[rox.ex],[rox.ey],[rox.ex]),axis=0).transpose()
		p0=np.array([[0],[0],[0.78]])
		p1=np.array([[0.32],[0],[0]])
		p2=np.array([[0.],[0],[1.075]])
		p3=np.array([[0],[0],[0.2]])   
		p4=np.array([[1.1425],[0],[0]])
		p5=np.array([[0.2],[0],[0]])
		p6=np.array([[0.0],[0],[0.0]])

		###fake link for fitting
		tcp_new=p_tool+np.dot(R_tool,np.array([0,0,d]))
		
		self.R_tool=R_tool
		self.p_tool=tcp_new


		self.P=np.concatenate((p0,p1,p2,p3,p4,p5,p6),axis=1)*1000.
		self.joint_type=np.zeros(6)
		
		###updated range&vel limit
		self.upper_limit=np.radians([170.,85.,70.,300.,120.,360.])
		self.lower_limit=np.radians([-170.,-65.,-180.,-300.,-120.,-360.])
		self.joint_vel_limit=np.radians([100,90,90,190,140,190])
		# self.joint_acc_limit=np.radians([312,292,418,2407,1547,3400])
		self.joint_acc_limit=np.array([-1,-1,-1,42.49102688076435,36.84030926197994,50.45298947544431])
		self.robot_def=rox.Robot(self.H,self.P,self.joint_type,joint_lower_limit = self.lower_limit, joint_upper_limit = self.upper_limit, joint_vel_limit=self.joint_vel_limit, R_tool=R_tool,p_tool=tcp_new)

		###acceleration table
		if len(acc_dict_path)>0:
			acc_dict= pickle.load(open(acc_dict_path,'rb'))
			q2_config=[]
			q3_config=[]
			q1_acc=[]
			q2_acc=[]
			q3_acc=[]
			for key, value in acc_dict.items():
			   q2_config.append(key[0])
			   q3_config.append(key[1])
			   q1_acc.append(value[0])
			   q2_acc.append(value[1])
			   q3_acc.append(value[2])
			self.q2q3_config=np.array([q2_config,q3_config]).T
			self.q1q2q3_acc=np.array([q1_acc,q2_acc,q3_acc]).T

	def get_acc(self,q_all):
		#if a single point
		if q_all.shape==(len(self.upper_limit),):
			###find closest q2q3 config, along with constant last 3 joints acc
			idx=np.argmin(np.linalg.norm(self.q2q3_config-q_all[1:3],axis=1))
			return np.append(self.q1q2q3_acc[idx],self.joint_acc_limit[-3:])
		else:
			acc_limit_all=[]
			for q in q_all:
				idx=np.argmin(np.linalg.norm(self.q2q3_config-q[1:3],axis=1))
				acc_limit_all.append(np.append(self.q1q2q3_acc[idx],self.joint_acc_limit[-3:]))

		return np.array(acc_limit_all)


	def jacobian(self,q):
		return rox.robotjacobian(self.robot_def,q)
	def fwd(self,q,base_R=np.eye(3),base_p=np.array([0,0,0]),qlim_override=False):
		if qlim_override:
			robot_def=copy.deepcopy(self.robot_def)
			robot_def.joint_upper_limit=999*np.ones(len(self.upper_limit))
			robot_def.joint_lower_limit=-999*np.ones(len(self.lower_limit))
			pose_temp=rox.fwdkin(robot_def,q)
		else:
			pose_temp=rox.fwdkin(self.robot_def,q)
		pose_temp.p=np.dot(base_R,pose_temp.p)+base_p
		pose_temp.R=np.dot(base_R,pose_temp.R)
		return pose_temp

	def fwd_all(self,q_all,base_R=np.eye(3),base_p=np.array([0,0,0])):
		pose_p_all=[]
		pose_R_all=[]
		for q in q_all:
			pose_temp=self.fwd(q,base_R,base_p)
			pose_p_all.append(pose_temp.p)
			pose_R_all.append(pose_temp.R)

		return Transform_all(pose_p_all,pose_R_all)

	def inv(self,p,R=np.eye(3),last_joints=None):
		pose=rox.Transform(R,p)
		q_all=rox.robot6_sphericalwrist_invkin(self.robot_def,pose,last_joints)
		return q_all


class confdata(NamedTuple):
    cf1: float
    cf4: float
    cf6: float
    cfx: float

class robtarget(NamedTuple):
    trans: np.ndarray # [x,y,z]
    rot: np.ndarray # [qw,qx,qy,qz]
    robconf: confdata # 
    extax: np.ndarray # shape=(6,)

class jointtarget(NamedTuple):
    robax: np.ndarray # shape=(6,)
    extax: np.ndarray # shape=(6,)

class pose(NamedTuple):
    trans: np.ndarray # [x,y,z]
    rot: np.ndarray # [qw,qx,qy,qz]

class loaddata(NamedTuple):
    mass: float
    cog: np.ndarray # shape=(3,)
    aom: np.ndarray # shape=(4,)
    ix: float
    iy: float
    iz: float

class tooldata(NamedTuple):
    robhold: bool
    tframe: pose
    tload : loaddata

quatR = rox.R2q(rox.rot([0,1,0],np.deg2rad(30)))
robot = abb6640(d=50)
R90=rox.rot([0,1,0],np.pi/2)  
tool1_abb=tool=tooldata(True,pose(R90.T@robot.p_tool,rox.R2q(robot.R_tool@R90.T)),loaddata(1,[0,0,0.001],[1,0,0,0],0,0,0))

class ConvertMotionProgram:

    def __init__(self, device_manager, robot, node = None, client_obj = None):
        self._robot_util = RobotUtil()
        self._geom_util = GeometryUtil()
        self.device_manager = device_manager

        self.node = node
        if self.node is None:
            self.node = RR.RobotRaconteurNode.s

        #self.tool_info = tool_info

        self.robot = robot

        #self.rox_robot = self._robot_util.robot_info_to_rox_robot(robot_info,0)

        self.robot_pose_type = self.node.GetStructureType("experimental.robotics.motion_program.RobotPose", obj=client_obj)
        self.moveabsj_type = self.node.GetStructureType("experimental.robotics.motion_program.MoveAbsJ", obj=client_obj)
        self.movej_type = self.node.GetStructureType("experimental.robotics.motion_program.MoveJ", obj=client_obj)
        self.movel_type = self.node.GetStructureType("experimental.robotics.motion_program.MoveL", obj=client_obj)
        self.movec_type = self.node.GetStructureType("experimental.robotics.motion_program.MoveC", obj=client_obj)
        self.wait_type = self.node.GetStructureType("experimental.robotics.motion_program.WaitTime", obj=client_obj)
        self.settool_type = self.node.GetStructureType("experimental.robotics.motion_program.SetTool", obj=client_obj)
        self.motionprogram_type = self.node.GetStructureType("experimental.robotics.motion_program.MotionProgram", obj=client_obj)
        self.toolinfo_type = self.node.GetStructureType("com.robotraconteur.robotics.tool.ToolInfo", obj=client_obj)
        self.transform_dt = self.node.GetNamedArrayDType("com.robotraconteur.geometry.Transform", obj=client_obj)
        self.spatialinertia_dt = self.node.GetNamedArrayDType("com.robotraconteur.geometry.SpatialInertia", obj=client_obj)
        self.pose_dt = self.node.GetNamedArrayDType("com.robotraconteur.geometry.Pose", obj=client_obj)

    def get_rox_robot(self, robot_local_device_name):
        return self.rox_robot

    def tool_info_from_pose(self,tool_pose,mass=0.01,com=[0,0,0.05]):
        #TODO: use a tool_info instead of asking user for tool_pose
        rr_toolinfo = self.toolinfo_type()
        rr_toolinfo.tcp = tool_pose
        rr_inertia = np.zeros((1,),dtype=self.spatialinertia_dt)
        rr_inertia[0]["m"] = mass
        rr_inertia[0]["com"]["x"] = com[0]
        rr_inertia[0]["com"]["y"] = com[1]
        rr_inertia[0]["com"]["z"] = com[2]
        rr_inertia[0]["ixx"] = 1.e-4
        rr_inertia[0]["iyy"] = 1.e-4
        rr_inertia[0]["izz"] = 1.e-4
        rr_toolinfo.inertia = rr_inertia
        return rr_toolinfo

    def convert_motion_program(self,primitives,breakpoints,p_bp,q_bp,velocity,blend_radius):
        
        mp_cmds = []
        settool = self.settool_type()
        #settool.tool_info = self.tool_info
        #TODO: Don't use hard coded tool info
        settool.tool_info = self.tooldata_to_rr_toolinfo(tool1_abb)
        mp_cmds.append(RR.VarValue(settool,"experimental.robotics.motion_program.SetTool"))
            
        for i in range(len(primitives)):
            motion = primitives[i]
            if motion == 'movel_fit':
                robot_pose = self.robot_pose_from_csv_row(self.robot,q_bp[i][0],p_bp[i][0])
                movel_command = self.movel(robot_pose, velocity, blend_radius, False)
                mp_cmds.append(movel_command)                

            elif motion == 'movec_fit':
                robot_pose1 = self.robot_pose_from_csv_row(self.robot,q_bp[i][0],p_bp[i][0])
                robot_pose2 = self.robot_pose_from_csv_row(self.robot,q_bp[i][1],p_bp[i][1])
                movec_command = self.movec(robot_pose2, robot_pose1, velocity, blend_radius, False)
                mp_cmds.append(movec_command)

            else: # movej_fit
                jointt = q_bp[i][0]
                if i==0:
                    mp_cmds.append(self.moveabsj(jointt,0.5,0.001,True))
                    mp_cmds.append(self.wait(1))
                    mp_cmds.append(self.moveabsj(jointt,0.5,0.001,True))
                    mp_cmds.append(self.wait(0.1))
                else:
                    mp_cmds.append(self.moveabsj(jointt,velocity,blend_radius,False))

        mp = self.motionprogram_type()

        mp.motion_program_commands = mp_cmds

        return mp

    def robot_pose_from_robtarget(self, abb_robtarget):
        p = abb_robtarget.trans
        q = abb_robtarget.rot
        conf = abb_robtarget.robconf
        
        ret = self.robot_pose_type()
        ret.tcp_pose[0]["orientation"]["w"] = q[0]
        ret.tcp_pose[0]["orientation"]["x"] = q[1]
        ret.tcp_pose[0]["orientation"]["y"] = q[2]
        ret.tcp_pose[0]["orientation"]["z"] = q[3]
        ret.tcp_pose[0]["position"]["x"] = p[0]*1e-3
        ret.tcp_pose[0]["position"]["y"] = p[1]*1e-3
        ret.tcp_pose[0]["position"]["z"] = p[2]*1e-3

        ret.joint_position_seed=np.zeros((6,))
        ret.joint_position_seed[0] = conf[0]*np.pi/2
        ret.joint_position_seed[3] = conf[1]*np.pi/2
        ret.joint_position_seed[5] = conf[2]*np.pi/2

        return ret

    def robot_pose_from_csv_row(self,robot,q,point):
        quat=rox.R2q(robot.fwd(q).R)
        ret = self.robot_pose_type()
        ret.tcp_pose[0]["orientation"]["w"] = quat[0]
        ret.tcp_pose[0]["orientation"]["x"] = quat[1]
        ret.tcp_pose[0]["orientation"]["y"] = quat[2]
        ret.tcp_pose[0]["orientation"]["z"] = quat[3]
        ret.tcp_pose[0]["position"]["x"] = point[0]*1e-3
        ret.tcp_pose[0]["position"]["y"] = point[1]*1e-3
        ret.tcp_pose[0]["position"]["z"] = point[2]*1e-3

        ret.joint_position_seed = np.copy(q)
        return ret

    def moveabsj(self,j,velocity,blend_radius,fine_point):
        cmd = self.moveabsj_type()
        cmd.joint_position = j
        cmd.tcp_velocity = velocity
        cmd.blend_radius = blend_radius
        cmd.fine_point = fine_point
        return RR.VarValue(cmd,"experimental.robotics.motion_program.MoveAbsJ")

    def movel(self, robot_pose,velocity,blend_radius,fine_point):
        cmd = self.movel_type()
        cmd.tcp_pose = robot_pose
        cmd.tcp_velocity = velocity
        cmd.blend_radius = blend_radius
        cmd.fine_point = fine_point
        return RR.VarValue(cmd,"experimental.robotics.motion_program.MoveL")

    def movej(self,robot_pose,velocity,blend_radius,fine_point):
        cmd = self.movej_type()
        cmd.tcp_pose = robot_pose
        cmd.tcp_velocity = velocity
        cmd.blend_radius = blend_radius
        cmd.fine_point = fine_point
        return RR.VarValue(cmd,"experimental.robotics.motion_program.MoveJ")

    def movec(self,robot_pose,robot_via_pose,velocity,blend_radius,fine_point):
        cmd = self.movec_type()
        cmd.tcp_pose = robot_pose
        cmd.tcp_via_pose = robot_via_pose
        cmd.tcp_velocity = velocity
        cmd.blend_radius = blend_radius
        cmd.fine_point = fine_point
        return RR.VarValue(cmd,"experimental.robotics.motion_program.MoveC")
    
    def wait(self, secs):
        cmd = self.wait_type()
        cmd.time = secs
        return RR.VarValue(cmd,"experimental.robotics.motion_program.WaitTime")

    def tooldata_to_rr_toolinfo(self, tooldata):
        rr_toolinfo = self.toolinfo_type()
        rr_toolinfo.tcp = self.abb_pose_to_rr_transform(tooldata.tframe)
        rr_inertia = np.zeros((1,),dtype=self.spatialinertia_dt)
        rr_inertia[0]["m"] = tooldata.tload.mass
        com = np.array(tooldata.tload.cog)*1e-3
        rr_inertia[0]["com"]["x"] = com[0]
        rr_inertia[0]["com"]["y"] = com[1]
        rr_inertia[0]["com"]["z"] = com[2]
        rr_inertia[0]["ixx"] = tooldata.tload.ix
        rr_inertia[0]["iyy"] = tooldata.tload.iy
        rr_inertia[0]["izz"] = tooldata.tload.iz
        rr_toolinfo.inertia = rr_inertia
        return rr_toolinfo

    def abb_pose_to_rr_transform(self,pose):
        quat = pose.rot
        point = pose.trans
        rr_pose = np.zeros((1,),dtype=self.transform_dt)        
        rr_pose[0]["rotation"]["w"] = quat[0]
        rr_pose[0]["rotation"]["x"] = quat[1]
        rr_pose[0]["rotation"]["y"] = quat[2]
        rr_pose[0]["rotation"]["z"] = quat[3]
        rr_pose[0]["translation"]["x"] = point[0]*1e-3
        rr_pose[0]["translation"]["y"] = point[1]*1e-3
        rr_pose[0]["translation"]["z"] = point[2]*1e-3
        return rr_pose

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