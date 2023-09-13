import yaml
import math

def write_yamls(points):
    zang2 = 0

    for i in range(len(points)-1):
        pt1 = points[i]
        pt2 = points[i+1]

        cmh = 0.98  # Center of Mass Height
        zang = 0    # Initial angle of orientation of whole body for this step, zang2 is the final angle
        footwidth = 0.137

        dx = pt2[0]-pt1[0]
        dy = pt2[1]-pt1[1]
        if dx > 0:
            zang = math.atan(dy/dx)
        elif dx < 0:
            zang = math.pi + math.atan(dy/dx)
        elif dx == 0 and dy < 0:
            zang = -1.57
        elif dx == 0 and dy > 0:
            zang = 1.57

        if i < len(points) - 2:
            pt3 = points[i+2]
            dx2 = pt3[0]-pt2[0]
            dy2 = pt3[1]-pt2[1]
            if dx2 > 0:
                zang2 = math.atan(dy2/dx2)
            elif dx2 < 0:
                zang2 = math.pi + math.atan(dy2/dx2)
            elif dx2 == 0 and dy2 < 0:
                zang2 = -1.57
            elif dx2 == 0 and dy2 > 0:
                zang2 = 1.57

        dist = math.sqrt(dy**2 + dx**2)
        numSwings = math.ceil(dist/0.5)

        leg1 = "    0:\n    - 0.5"
        leg2 = "    1:\n    - 2.0"
        for j in range(numSwings):
            if j%2 == 0:
                leg1 = leg1 + "\n    - 0.75\n    - 1.65"
            else:
                leg2 = leg2 + "\n    - 0.75\n    - 1.65"

        if numSwings%2 == 0:
            leg1 = leg1 + "\n    - 0.75\n    - 0.75"
        else:
            leg2 = leg2 + "\n    - 0.75\n    - 0.15"

        doc = """
locomotion_param:
  b_optimize_timings: false
  bound_phase_duration:
  - 0.15
  - 0.15
  costs:
    w_BaseAngVelDiffCost:
    - 0.1
    - 0.1
    - 0.1
    w_BaseLinVelDiffCost:
    - 0.1
    - 0.1
    - 1.0
    w_FinalBaseAngPosCost:
    - 2.0
    - 2.0
    - 2.0
    w_FinalBaseAngVelCost:
    - 1.0
    - 1.0
    - 1.0
    w_FinalBaseLinPosCost:
    - 2.0
    - 2.0
    - 2.0
    w_FinalBaseLinVelCost:
    - 1.0
    - 1.0
    - 1.0
    w_FinalEEMotionAngPosCost:
    - 2.0
    - 2.0
    - 2.0
    w_FinalEEMotionLinPosCost:
    - 2.0
    - 2.0
    - 20.0
    w_IntermediateBaseAngVelCost:
    - 0.01
    - 0.01
    - 0.01
    w_IntermediateBaseLinPosCost:
    - 0
    - 0.1
    - 100.0
    w_IntermediateBaseLinVelCost:
    - 0.001
    - 0.001
    - 1.0
    w_WrenchAngPosCost:
    - 0.1
    - 0.1
    - 0.01
    w_WrenchAngVelCost:
    - 0.01
    - 0.01
    - 0.01
    w_WrenchAngVelDiffCost:
    - 0.001
    - 0.001
    - 0.001
    w_WrenchLinPosCost:
    - 0.1
    - 0.1
    - 0.01
    w_WrenchLinVelCost:
    - 0.01
    - 0.01
    - 0.01
    w_WrenchLinVelDiffCost:
    - 0.001
    - 0.001
    - 0.001
  dt_constraint_base_motion: 0.025
  dt_constraint_dynamic: 0.1
  dt_constraint_range_of_motion: 0.08
  duration_base_polynomial: 0.1
  ee_in_contact_at_start:
    0: true
    1: true
  ee_phase_durations:
""" + leg1 + """
""" + leg2 + """
  ee_polynomials_per_swing_phase: 2
  force_limit_in_normal_direction: 1000.0
  force_polynomials_per_stance_phase: 3
locomotion_task:
  final_base_ang:
  - 0.0
  - 0.0
  - """ + str(float(zang2)) + """
  - 0.0
  - 0.0
  - 0.0
  final_base_lin:
  - """ + str(float(pt2[0])) + """
  - """ + str(float(pt2[1])) + """
  - """ + str(cmh + pt2[2]) + """
  - 0.0
  - 0.0
  - 0.0
  initial_base_ang:
  - 0.0
  - 0.0
  - """ + str(float(zang)) + """
  - 0.0
  - 0.0
  - 0.0
  initial_base_lin:
  - """ + str(float(pt1[0])) + """
  - """ + str(float(pt1[1])) + """
  - """ + str(cmh + pt1[2]) + """
  - 0.0
  - 0.0
  - 0.0
  initial_ee_motion_ang:
    0:
    - 0.0
    - 0.0
    - """ + str(float(zang)) + """
    1:
    - 0.0
    - 0.0
    - """ + str(float(zang)) + """
  initial_ee_motion_lin:
    0:
    - """ + str(float(pt1[0]) - math.sin(zang)*footwidth) + """
    - """ + str(float(pt1[1]) + math.cos(zang)*footwidth) + """
    - """ + str(pt1[2]) + """
    1:
    - """ + str(float(pt1[0]) + math.sin(zang)*footwidth) + """
    - """ + str(float(pt1[1]) - math.cos(zang)*footwidth) + """
    - """ + str(pt1[2]) + """
  terrain_type: terrain
solver: ipopt
"""
        #yaml.dump(yaml.load(doc, Loader=yaml.CLoader), default_flow_style=False)
        with open('config/towr_plus/valkyrie_simple_maze' + str(i) + '.yaml', 'w') as f:
            yaml.dump(yaml.load(doc, Loader=yaml.CLoader), f, default_flow_style=False)