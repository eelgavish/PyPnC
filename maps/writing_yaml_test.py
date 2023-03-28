import yaml

def write_yamls(points):
    for i in range(len(points)-1):
        pt1 = points[i]
        pt2 = points[i+1]

        doc = """solver: "ipopt"
locomotion_task:

initial_base_lin: [""" + str(float(pt1[0])) + """, """ + str(float(pt1[1])) + """, 0.98, 0., 0., 0.]
initial_base_ang: [0., 0., 0., 0., 0., 0.]

# Left Foot: 0, Right Foot: 1
initial_ee_motion_lin:
0: [0.003, 0.111, 0.0]
1: [0.003, -0.111, 0.0]
initial_ee_motion_ang:
0: [0., 0., 0.]
1: [0., 0., 0.]

final_base_lin: [""" + str(float(pt2[0])) + """, """ + str(float(pt2[1])) + """, 0.98, 0., 0., 0.]
final_base_ang: [0., 0., 0., 0., 0., 0.]

terrain_type: "terrain"

locomotion_param:

duration_base_polynomial: 0.1
force_polynomials_per_stance_phase: 3
ee_polynomials_per_swing_phase: 2
force_limit_in_normal_direction: 1000.
dt_constraint_range_of_motion: 0.08
dt_constraint_dynamic: 0.1
dt_constraint_base_motion: 0.025
b_optimize_timings: true
bound_phase_duration: [0.15, 0.15]

## =========================================================================
## Consistency with DCM Planner
## =========================================================================
# b : 0.279252 << sqrt(height / gravity)
# The foot lifting first : 1.5*ds
# The foot lifting later : 2.5*ds + ss
# Middle swing foot : ss
# Middle landing foot : 2*ds + ss
# The foot landing first : 2*ds + ss + b
# The foot landing later : ds + b
# For example
# LF: {1.5ds, ss, 2ds+ss, ss, 2ds+ss, ss, ds+b}
# RF: {2.5ds+ss, ss, 2ds+ss, ss, 2ds+ss+b}
ee_phase_durations:
0: [0.675, 0.75, 1.65, 0.75, 1.65, 0.75, 1.65, 0.75, 1.929252]
1: [1.875, 0.75, 1.65, 0.75, 1.65, 0.75, 1.65, 0.75, 0.729252]
ee_in_contact_at_start:
0: True
1: True

costs:
w_IntermediateBaseLinPosCost: [0, 0.1, 100.]
w_FinalBaseLinPosCost: [2., 2., 2.]
w_FinalBaseLinVelCost: [1., 1., 1.]
w_FinalBaseAngPosCost: [2., 2., 2.]
w_FinalBaseAngVelCost: [1., 1., 1.]
w_FinalEEMotionLinPosCost: [2., 2., 20.]
w_FinalEEMotionAngPosCost: [2., 2., 2.]
w_IntermediateBaseLinVelCost: [0.001, 0.001, 1.0]
w_IntermediateBaseAngVelCost: [0.01, 0.01, 0.01]
w_BaseLinVelDiffCost: [0.1, 0.1, 1.0]
w_BaseAngVelDiffCost: [0.1, 0.1, 0.1]
w_WrenchLinPosCost: [0.1, 0.1, 0.01]
w_WrenchAngPosCost: [0.1, 0.1, 0.01]
w_WrenchLinVelCost: [0.01, 0.01, 0.01]
w_WrenchAngVelCost: [0.01, 0.01, 0.01]
w_WrenchLinVelDiffCost: [0.001, 0.001, 0.001]
w_WrenchAngVelDiffCost: [0.001, 0.001, 0.001]
"""
        #yaml.dump(yaml.load(doc, Loader=yaml.CLoader), default_flow_style=False)
        with open('config/towr_plus/valkyrie_terrain' + str(i) + '.yaml', 'w') as f:
            yaml.dump(yaml.load(doc, Loader=yaml.CLoader), f, default_flow_style=False)