    



    
def motion_program_generation(curve_js, robot, total_seg):
    step=int((len(curve_js)-1)/total_seg)

    breakpoints = [0]
    primitives = ['movej_fit']
    q_bp = [[curve_js[0]]]
    p_bp = [[robot.fwd(curve_js[0]).p]]
    for i in range(step,len(curve_js),step):
        breakpoints.append(i)
        primitives.append('movel_fit')
        q_bp.append([curve_js[i]])
        p_bp.append([robot.fwd(curve_js[i]).p])

    return breakpoints,primitives,q_bp,p_bp