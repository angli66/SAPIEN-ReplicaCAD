import sapien.core as sapien
from sapien.utils import Viewer

from frl_apt_0 import build_scene

# Set up engine and renderer
engine = sapien.Engine()
renderer = sapien.SapienRenderer()
engine.set_renderer(renderer)

# Enable ray tracing
# sapien.render_config.camera_shader_dir = 'rt'
# sapien.render_config.viewer_shader_dir = 'rt'
# sapien.render_config.rt_samples_per_pixel = 8
# sapien.render_config.rt_max_path_depth = 8
# sapien.render_config.rt_use_denoiser = True

scene = build_scene(engine)

# Robot
loader = scene.create_urdf_loader()
loader.fix_root_link = True
robot = loader.load("maniskill_robot/mobile_panda_single_arm.urdf")
robot.set_root_pose(sapien.Pose([3.5, 1, 0], [-0.7, 0, 0, 0.7]))

init_qpos = [0.0, 0.0, 0.0, 0.0, 1.6, 1.7, -0.1, 0.0, -2.9, 2.2, 0.6, 0.04, 0.04]
robot.set_qpos(init_qpos)

# target_qpos = [0.00011028391, -0.00064619776, -0.002439121, 0.00021359444, 0.34349692, 2.3877838, -1.4859253, 1.2807951, -2.225509, 2.1995654, 0.5999923, 0.04, 0.039992824]
target_qpos = [8.962338e-05, -5.733477, -0.002431985, 0.00018220858, 1.5999997, 1.6997387, -0.0999832, -0.06978253, -2.897293, 2.1995513, 0.5999932, 0.04, 0.03999293]
active_joints = robot.get_active_joints()
for joint_idx, joint in enumerate(active_joints):
    joint.set_drive_property(stiffness=1000, damping=200)
    joint.set_drive_target(target_qpos[joint_idx])

# Viewer
viewer = Viewer(renderer, resolutions=(1920, 1080))
viewer.set_scene(scene)
viewer.set_camera_xyz(x=-2.0, y=0, z=1.5)

# viewer.paused = True
while not viewer.closed:
    qf = robot.compute_passive_force(
        gravity=True,
        coriolis_and_centrifugal=True,
    )
    robot.set_qf(qf)
    
    scene.step()
    scene.update_render()
    viewer.render()

