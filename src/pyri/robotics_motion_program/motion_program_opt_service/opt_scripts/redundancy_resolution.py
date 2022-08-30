import numpy as np
import sys, traceback
from general_robotics_toolbox import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
sys.path.append('../toolbox')
from robots_def import *
from utils import *
from lambda_calc import *
from pathlib import Path
import pickle
from baseline import *
#from pandas import *
import io

def main():
    
    opt_params_fname = Path(sys.argv[1])/ "motion_opt_input.pickle"

    with open(opt_params_fname,"rb") as f:
        opt_params = pickle.load(f)

    print(f"input curve.shape: {opt_params['curve'].shape}")

    ###read in curves
    curve = opt_params["curve"]
    lam=calc_lam_cs(curve)
    robot=abb6640(d=50)

    print("OPTIMIZING ON CURVE POSE")
    H=pose_opt(robot,curve[:,:3],curve[:,3:])
    print(H)

    
    curve_base,curve_normal_base=curve_frame_conversion(curve[:,:3],curve[:,3:],H)
    # visualize_curve_w_normal(curve_base,curve_normal_base,equal_axis=True)

    ###get all inv solutions
    print("FIND ALL POSSIBLE INV SOLUTIONS")
    curve_js_all=find_js(robot,curve_base,curve_normal_base)
    print('num solutions available: ',len(curve_js_all))
    if len(curve_js_all)==0:
        return

    ###get best with max(min(J_sing))
    print("FIND BEST CURVE_JS ON MIN(J_SINGULAR)")
    J_min=[]
    for i in range(len(curve_js_all)):
        J_min.append(find_j_min(robot,curve_js_all[i]))

    J_min=np.array(J_min)
    plt.figure()
    for i in range(len(J_min)):
        plt.plot(lam,J_min[i],label="inv choice "+str(i))


    ##############j_singular plot####################
    plt.legend()
    plt.title('Minimum J_SINGULAR')
    j_minimum_io = io.BytesIO()
    plt.savefig(j_minimum_io,format="svg")
    curve_js=curve_js_all[np.argmin(J_min.min(axis=1))]

    curve_base2 = np.concatenate((curve_base,curve_normal_base),axis=1)

    ##############3D plots####################
    fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
    ax.plot3D(curve_base[:,0],curve_base[:,1],curve_base[:,2],'r.-')
    ax.quiver(curve_base[::20,0],curve_base[::20,1],curve_base[::20,2],curve_normal_base[::20,0],curve_normal_base[::20,1],curve_normal_base[::20,2],length=5, normalize=True)
    plt.title('Curve Base')
    curve_3d_io = io.BytesIO()
    plt.savefig(curve_3d_io,format="svg")

    print(f"len(curve_js): {len(curve_js)}")
    print(f"curve_base.shape: {curve_base2.shape}")
    output = {
        "curve": curve,
        "curve_js": curve_js,
        "curve_base": curve_base2,
        "curve_pose": H,
        "plots": {
            "j_minimum": j_minimum_io.getvalue(),
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