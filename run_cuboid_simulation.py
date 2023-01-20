import os
import pybullet as p
import pybullet_data
from squaternion import Quaternion
from time import sleep
import datetime as dt
import numpy as np
from typing import List
import sys


MODE = p.GUI
# MODE = p.DIRECT
SAVE = True
TIMESTEP = 1/120
NUM_STEPS = 500
EARLY_STOPPING = True  # stop if no significant change in positions
EARLY_STOP_AFTER_NO_CHANGE_STEPS = 100
EARLY_STOPPING_DELTA = 0.001
SAVE_EVERY_STEPS = 1  # "checkpoint"
NUM_ITEMS = 20
ITEM_MARGIN = 1.0
# NUM_SIMULATIONS = 1
NUM_SIMULATIONS = int(sys.argv[1]) if len(sys.argv) > 1 else 1
ORIENTATION_NOISE = 0.01
# Photo configuration:
TAKE_PHOTOS = False
PHOTO_EVERY_STEPS = 10
view_matrix = p.computeViewMatrix(cameraEyePosition=[1, 1, 1],
                                  cameraTargetPosition=[0, 0, 0],
                                  cameraUpVector=[0, 0, 1])
# https://bit.ly/3HecmQM
# https://bit.ly/3QJFd2N
# https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/getCameraImageTest.py
# https://pybullet.org/Bullet/phpBB3/viewtopic.php?t=13370
view_width = 128
view_height = 128
fov = 60  # !!
aspect = view_width/view_height
near_val = 0.2  # !!
far_val = 5  # !!
projection_matrix = p.computeProjectionMatrixFOV(fov=fov,
                                                 aspect=aspect,
                                                 nearVal=near_val,
                                                 farVal=far_val)


def generate_random_item():
    dim = np.random.uniform(0.1, 0.5, size=3).round(4)
    pos = [*np.random.uniform(-3.0, 3.0, size=2).round(4),
           max(np.round(np.random.uniform(0, 3.0), 4), dim[2]/2+0.05)]
    # Making sure that the pos_height is at least the dim_height/2, otherwise the item is stuck in the ground
    # re-order so it's the same ordering as in pybullet; only if I use squaternion above!
    # each item's vector looks like this: [x, y, z, w, d, h, q1, q2, q3, q4, m]
    # [x, y, z] is the position, [w, d, h] is the shape
    # Orientation
    q = np.array(Quaternion.from_euler(0., 0., 0.))
    q = q[[1, 2, 3, 0]]  # re-order so it's the same ordering as in pybullet; only if I use squaternion above!
    # add a little bit of noise to the orientation
    noise = np.random.normal(0, ORIENTATION_NOISE, size=4)
    q = (q + noise).round(4)
    orn = q
    # for now, assume zero orientation (they're all cuboids; I can change orientation by changing its dimensions)
    # for Euler orientation I would use notation [a, b, g] (alpha, beta, gamma)
    # Mass
    # to make things easy, mass is always simply proportional to volume and this proportionality coefficient is
    # randomly sampled to be between, say, 5 and 15, according to a uniform distribution
    m = (np.prod(dim)*np.random.uniform(5, 15)).round(4)
    # Each item's vector looks like this: [x, y, z, w, d, h, q1, q2, q3, q4, m]
    return np.array([*pos, *dim, *orn, m])


def intersects(item1, item2):
    # https://gamedev.stackexchange.com/questions/23748/testing-whether-two-cubes-are-touching-in-space
    # True if item1 and item2 intersect
    # Note that w, h, d are half-extents!!
    x1, y1, z1, w1, d1, h1, _, _, _, _, _ = item1
    x2, y2, z2, w2, d2, h2, _, _, _, _, _ = item2
    # Just to make sure that they are also not too close to each other, add a small "margin" to each item
    # only for comparison purposes
    w1, d1, h1 = np.array([w1, d1, h1]) + ITEM_MARGIN
    w2, d2, h2 = np.array([w2, d2, h2]) + ITEM_MARGIN
    min_x1, max_x1, min_y1, max_y1, min_z1, max_z1 = x1-w1, x1+w1, y1-d1, y1+d1, z1-h1, z1+h1
    min_x2, max_x2, min_y2, max_y2, min_z2, max_z2 = x2-w2, x2+w2, y2-d2, y2+d2, z2-h2, z2+h2
    return ((min_x1 < min_x2 < max_x1) or (min_x2 < min_x1 < max_x2)) and \
           ((min_y1 < min_y2 < max_y1) or (min_y2 < min_y1 < max_y2)) and \
           ((min_z1 < min_z2 < max_z1) or (min_z2 < min_z1 < max_z2))


def any_intersects(item1, items):
    for item in items:
        if intersects(item1, item):
            return True
        return False


def generate_random_example() -> List:
    items = []
    for _ in range(NUM_ITEMS):
        items.append(generate_random_item())
    return items


def generate_realistic_example() -> List:
    # TODO: Check if it's even possible to have an arrangement with this many items in the expected space
    #  Since the position is sampled from a normal distribution, it might eventually randomly sample a box
    #  that's far enough not to intersect
    items = []
    for _ in range(NUM_ITEMS):
        proposed_item = generate_random_item()
        if i > 0:
            while any_intersects(proposed_item, items):
                # print("trying again")
                proposed_item = generate_random_item()
        items.append(proposed_item)
        # print(proposed_item)
    return items


def calculate_delta(arr, t1=0, t2=-1, mean=True, axes=(0,1,2)):
    # arr is of shape (num_timesteps, num_items, 11)
    # axes: e.g. xy deltas would be axes=(0,1), full xyz delta would be axes=(0,1,2)
    # (t1, t2): delta between these timesteps, default is (0, -1) meaning the delta between initial and final states
    # FIXME: does it make sense to use the mean? just because there are 10 more items that, say, don't touch any other
    #  items and none of them move in the xy dimension doesn't make the other items more stable; but with the mean, this
    #  would drag down the overall delta
    a = arr[t1,:,axes].reshape(-1, len(axes))  # initial state
    b = arr[t2,:,axes].reshape(-1, len(axes))  # final state
    # Mean Euclidean distance between final and initial states
    delta = np.linalg.norm(b-a, axis=1).sum()
    if mean:
        delta /= len(a)
    return delta


def run_simulation(simulation_id: str = None) -> np.array:
    timestamp = str(dt.datetime.now()).replace(" ", "_").replace(":", "_").replace("-", "_").replace(".", "_")
    if simulation_id is not None:
        data = list(np.load(open(f"cuboid_simulations/{simulation_id}.npy", "rb"))[0])
        print("Re-running", simulation_id)
    else:
        # If no simulation_id is provided, generate new realistic example
        data = generate_realistic_example()

    print("Setting up physics engine")
    p.connect(MODE)
    # p.setPhysicsEngineParameter(numSolverIterations=10)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")
    # p.loadURDF("bicycle/bike.urdf", basePosition=[0, 0, 5])
    # p.loadURDF("table/table.urdf")  # already at correct location
    p.setGravity(0, 0, -10)
    p.setTimeStep(TIMESTEP)

    print("Setting up items")
    item_ids = []
    for item in data:
        x, y, z, w, d, h, q1, q2, q3, q4, m = item
        cs_id = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[w, d, h])
        vs_id = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[w, d, h])
        item_id = p.createMultiBody(baseMass=m,
                                    baseCollisionShapeIndex=cs_id,
                                    baseVisualShapeIndex=vs_id,
                                    basePosition=[x, y, z],
                                    baseOrientation=[q1, q2, q3, q4],
                                    flags=p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS)
        item_ids.append(item_id)

    print("Running simulation")
    simulation_data = []
    no_change_counter = 0
    # while True:
    for t in range(NUM_STEPS):
        if t % SAVE_EVERY_STEPS == 0:
            # "Checkpoint":
            timestep_data = []
            for item_id, item in zip(item_ids, data):
                _, _, _, w, d, h, _, _, _, _, m = item
                pos, orn = p.getBasePositionAndOrientation(item_id)
                # Note however that now the orientation is in quaternions, not Euler angles
                # I could convert it back to Euler angles
                # https://pybullet.org/Bullet/phpBB3/viewtopic.php?t=3404
                timestep_data.append([*pos, w, d, h, *orn, m])
                # print(t, (pos, orn))
            simulation_data.append(timestep_data)
            if EARLY_STOPPING:
                # get the 3d delta between last checkpoint and now, and if it's less than a small number, say 0.001, for
                # say 50 timesteps in a row, we can stop "early"
                delta = calculate_delta(np.array(simulation_data), t1=t-SAVE_EVERY_STEPS, t2=t, axes=(0,1,2))
                if delta < EARLY_STOPPING_DELTA:
                    # Counting subsequent timesteps without any significant 3d delta
                    no_change_counter += SAVE_EVERY_STEPS
                    if no_change_counter > EARLY_STOP_AFTER_NO_CHANGE_STEPS:
                        print(f"Early stopping after {t} timesteps")
                        break
                else:
                    # If there's a change, reset no_change_counter back to 0
                    no_change_counter = 0
        if TAKE_PHOTOS & t % PHOTO_EVERY_STEPS == 0:
            _, _, rgb_img, dep_img, seg_img = p.getCameraImage(width=view_width,
                                                               height=view_height,
                                                               viewMatrix=view_matrix,
                                                               projectionMatrix=projection_matrix,
                                                               shadow=True,
                                                               renderer=p.ER_BULLET_HARDWARE_OPENGL)
            # NOTE: the ordering of height and width change based on the conversion
            # 3 types of images: RGB, depth and segmentation
            rgb_img = np.reshape(rgb_img, (view_height, view_width, 4)) * 1/255
            dep_img = far_val * near_val / (far_val-(far_val-near_val) * np.reshape(dep_img, [view_width, view_height]))
            seg_img = np.reshape(seg_img, [view_width, view_height]) * 1/255
            if SAVE:
                np.save(open(f"screenshots/{timestamp}_t_{t}_rgb.npy", "wb"), rgb_img)
                np.save(open(f"screenshots/{timestamp}_t_{t}_dep.npy", "wb"), dep_img)
                np.save(open(f"screenshots/{timestamp}_t_{t}_seg.npy", "wb"), seg_img)
        if MODE == p.GUI:
            sleep(1/240)
        # check if stuff is changing, if not, break from loop
        #
        #
        p.stepSimulation()

    print("Exiting physics engine")
    p.disconnect()

    print("Creating final array")
    arr = np.array(simulation_data)
    if SAVE and simulation_id is None:
        # Only save if it's a new simulation
        filename = f"cuboid_simulations/{timestamp}.npy"
        print("Save to", filename)
        np.save(open(filename, "wb"), arr)

    print("Finished")

    return arr


def find_minimum_delta_simulation(axes=(0,1)):
    print("Finding minimum-delta simulation")
    deltas = []
    # min_simulation = None  # simulation corresponding to min_delta
    min_simulation_id = None
    for filename in os.listdir("cuboid_simulations"):
        if ".npy" in filename:
            v = np.load(open("cuboid_simulations/" + filename, "rb"))
            delta = calculate_delta(v, axes=axes)
            if len(deltas) > 0 and delta < min(deltas):
                # min_simulation = v
                min_simulation_id = filename.replace(".npy", "")
            deltas.append(delta)
    deltas = np.array(deltas)
    print(" ", deltas.min())
    print(" ", min_simulation_id)
    return min_simulation_id


if __name__ == "__main__":
    for i in range(NUM_SIMULATIONS):
        print("Simulation", i)
        sim = run_simulation(find_minimum_delta_simulation())
        # sim = run_simulation()
        print("Delta (xy):", np.round(calculate_delta(sim, axes=(0,1)), 4))
    # What's the minimum-delta simulation so far? out of all existing simulations
    # find_minimum_delta_simulation()
