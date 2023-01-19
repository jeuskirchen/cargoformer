# same as run_cuboid_simulation except use mesh objects instead of cuboids
# https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/createVisualShape.py
# e.g. cube_small.urdf and then apply something like this but for collision shape? 
#   cube_trans = p.loadURDF('cube_small.urdf', basePosition=[0.0, 0.1, 0.025])
#   p.changeVisualShape(cube_trans, -1, rgbaColor=[1, 1, 1, 0.1])
# check out which urdf files there are in the pybullet library
