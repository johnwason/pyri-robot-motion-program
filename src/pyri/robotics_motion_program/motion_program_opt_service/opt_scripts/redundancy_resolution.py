import numpy as np
import sys, traceback
from general_robotics_toolbox import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from pathlib import Path
import pickle
from robot_motion_program_opt.redundancy_resolution import redundancy_resolution_baseline
from pyri.robotics_motion_program.motion_program_opt_service.opt_script_util import load_rox_robot_obj

#from pandas import *
import io

def main():
    
    opt_params_fname = Path(sys.argv[1])/ "motion_opt_input.pickle"

    with open(opt_params_fname,"rb") as f:
        opt_params = pickle.load(f)

    print(f"input curve.shape: {opt_params['curve'].shape}")

    ###read in curves
    
    curve = opt_params["curve"]

    robot = load_rox_robot_obj(opt_params)
    
    curve_base,curve_normal_base,curve_js,H=redundancy_resolution_baseline(curve, robot)

    ##############3D plots####################
    fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
    ax.plot3D(curve_base[:,0],curve_base[:,1],curve_base[:,2],'r.-')
    ax.quiver(curve_base[::20,0],curve_base[::20,1],curve_base[::20,2],curve_normal_base[::20,0],curve_normal_base[::20,1],curve_normal_base[::20,2],length=5, normalize=True)
    plt.title('Curve Base')
    curve_3d_io = io.BytesIO()
    plt.savefig(curve_3d_io,format="svg")

    curve_base2 = np.concatenate((curve_base,curve_normal_base),axis=1)

    print(f"len(curve_js): {len(curve_js)}")
    print(f"curve_base.shape: {curve_base2.shape}")
    output = {
        "curve": curve,
        "curve_js": curve_js,
        "curve_base": curve_base2,
        "curve_pose": H,
        "plots": {
            "curve_3d": curve_3d_io.getvalue()
        }
    }

    output_fname = Path(sys.argv[1])/ "motion_opt_output.pickle"

    with open(output_fname, "wb") as f:
        pickle.dump(output, f)

    ###save file
    # df=DataFrame({'x':curve_base[:,0],'y':curve_base[:,1], 'z':curve_base[:,2],'x_dir':curve_normal_base[:,0],'y_dir':curve_normal_base[:,1], 'z_dir':curve_normal_base[:,2]})
    # df.to_csv(data_dir+'Curve_in_base_frame.csv',header=False,index=False)
    # DataFrame(curve_js).to_csv(data_dir+'Curve_js.csv',header=False,index=False)
    # with open(data_dir+'blade_pose.yaml', 'w') as file:
    #     documents = yaml.dump({'H':H.tolist()}, file)

if __name__ == "__main__":
    main()