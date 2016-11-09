import os
import ikpy
import numpy as np
from ikpy import plot_utils

this_dir, this_filename = os.path.split(__file__)
urdffile = os.path.join(this_dir, "nextage", "NextageOpen.urdf")
my_chain = ikpy.chain.Chain.from_urdf_file(urdffile)


# import matplotlib.pyplot as plt
# ax = plot_utils.init_3d_figure()
# my_chain.plot(my_chain.inverse_kinematics(target_frame), ax, target=target_vector)
# plt.xlim(-0.1, 0.1)
# plt.ylim(-0.1, 0.1)

target_vector = [ 0.4, 0, 0]
target_frame = np.eye(4)
target_frame[:3, 3] = target_vector
print("The angles of each joints are : ", my_chain.inverse_kinematics(target_frame))