from adatools import config_generator as cg


arm_conf = cg.get_robot_config_1(link1=0.145, link1_offset=0.0,
                                       link2=0.350, link2_offset=0.0,
                                       link3=0.330, link3_offset=0.0,
                                       link4=0.100, link4_offset=0.0)

print(arm_conf)
arm_conf.teach(arm_conf.q)