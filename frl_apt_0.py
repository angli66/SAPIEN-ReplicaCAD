import sapien.core as sapien
from sapien.utils import Viewer

import numpy as np


def r2s(pose: sapien.Pose) -> sapien.Pose:
    """
    Transform pose in ReplicaCAD's original frame into SAPIEN's world frame.
    """
    mat44 = np.eye(4)
    mat44[:3, :3] = np.array([
        [0, -1, 0],
        [0, 0, 1],
        [-1, 0, 0]
    ]).T
    relative_pose = sapien.Pose.from_transformation_matrix(mat44)
    return relative_pose * pose


def main():
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

    # Set up scene
    scene = engine.create_scene()
    scene.set_timestep(1 / 100.0)

    # Stage
    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/stages/frl_apartment_stage.glb")
    builder.add_nonconvex_collision_from_file(
        filename="replica_cad/stages/frl_apartment_stage.glb")
    stage = builder.build_static(name='stage')
    stage.set_pose(r2s(sapien.Pose()))

    # Lighting
    scene.set_ambient_light([0.5, 0.5, 0.5])
    scene.add_point_light([1, 2, 2], [1, 1, 1], shadow=True)
    scene.add_point_light([1, -2, 2], [1, 1, 1], shadow=True)
    scene.add_point_light([-1, 0, 1], [1, 1, 1], shadow=True)

    # Objects
    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_basket.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_basket_cv_decomp.glb")
    basket = builder.build(name='basket')
    basket.set_pose(r2s(sapien.Pose(
        p=np.array([
            -1.9956579525706273,
            1.0839370509764081,
            0.057981376432922185
        ]),
        q=np.array([
            0.9846951961517334,
            -5.20254616276361e-07,
            0.17428532242774963,
            3.540688453540497e-07
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_bike_01.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_bike_01_cv_decomp.glb")
    bike_01 = builder.build(name='bike')
    bike_01.set_pose(r2s(sapien.Pose(
        p=np.array([
            4.107420409715755,
            0.4545634772665478,
            -0.9759617637063622
        ]),
        q=np.array([
            0.1531376838684082,
            0.07608834654092789,
            0.985193133354187,
            0.012405824847519398
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_bin_01.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_bin_01_cv_decomp.glb")
    bin_01 = builder.build(name='bin')
    bin_01.set_pose(r2s(sapien.Pose(
        p=np.array([
            -2.283838131332137,
            0.3135578462463949,
            -0.7448115889960697
        ]),
        q=np.array([
            0.7049616575241089,
            0.0031301460694521666,
            -0.708801805973053,
            -0.02488476037979126
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_bin_02.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_bin_02_cv_decomp.glb")
    bin_02 = builder.build(name='bin')
    bin_02.set_pose(r2s(sapien.Pose(
        p=np.array([
            -2.2172783992605987,
            0.4316010993063046,
            -0.4654735649683427
        ]),
        q=np.array([
            0.7108882069587708,
            -5.3508705605054274e-05,
            -0.7033050060272217,
            -3.79412122128997e-05
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_bin_03.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_bin_03_cv_decomp.glb")
    bin_03 = builder.build(name='bin')
    bin_03.set_pose(r2s(sapien.Pose(
        p=np.array([
            4.188153043316561,
            0.34794042236171663,
            7.111236203790895
        ]),
        q=np.array([
            1.0,
            0.0,
            0.0,
            0.0
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_wall_cabinet_01.glb")
    builder.add_nonconvex_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_wall_cabinet_01_cv_decomp.glb")
    wall_cabinet_01 = builder.build_kinematic(name='wall_cabinet')
    # builder.add_collision_from_file(
    #     filename="replica_cad/objects/convex/frl_apartment_wall_cabinet_01_cv_decomp.glb")
    # wall_cabinet_01 = builder.build(name='wall_cabinet')
    wall_cabinet_01.set_pose(r2s(sapien.Pose(
        p=np.array([
            4.164929883746648,
            1.052598610751887,
            3.9504594020091175
        ]),
        q=np.array([
            0.6936941742897034,
            -9.310095265391283e-06,
            0.7202696800231934,
            -6.524024229292991e-06
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_clock.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_clock_cv_decomp.glb")
    clock = builder.build(name='clock')
    clock.set_pose(r2s(sapien.Pose(
        p=np.array([
            4.174310779977897,
            0.5726996842237058,
            3.945805886231077
        ]),
        q=np.array([
            0.7016434073448181,
            -0.002240218920633197,
            0.7125226259231567,
            -0.0017542409477755427
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_stool_02.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_stool_02_cv_decomp.glb")
    stool_02 = builder.build(name='stool')
    stool_02.set_pose(r2s(sapien.Pose(
        p=np.array([
            4.026192399552099,
            0.29405463829261547,
            2.370249720036307
        ]),
        q=np.array([
            0.7304269075393677,
            -1.8721173091762466e-06,
            0.6829908490180969,
            5.651035735354526e-06
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_chair_05.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_chair_05_cv_decomp.glb")
    chair_05 = builder.build(name='chair')
    chair_05.set_pose(r2s(sapien.Pose(
        p=np.array([
            4.0896974160083115,
            0.5851505185292629,
            0.7366678872025914
        ]),
        q=np.array([
            0.707486093044281,
            7.237448471641983e-07,
            0.7067272663116455,
            2.0450647753023077e-06
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_chair_05.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_chair_05_cv_decomp.glb")
    chair_05 = builder.build(name='chair')
    chair_05.set_pose(r2s(sapien.Pose(
        p=np.array([
            4.133461584783093,
            0.5851506079213841,
            0.08737107748939381
        ]),
        q=np.array([
            0.7064372897148132,
            1.7505947802476385e-09,
            0.7077756524085999,
            -1.3640106999446289e-06
        ])
    )))

    # builder = scene.create_actor_builder()
    # builder.add_visual_from_file(
    #     filename="replica_cad/objects/frl_apartment_handbag.glb")
    # builder.add_collision_from_file(
    #     filename="replica_cad/objects/convex/frl_apartment_handbag_cv_decomp.glb")
    # handbag = builder.build(name='handbag')
    # handbag.set_pose(r2s(sapien.Pose(
    #     p=np.array([
    #         -2.323607023799576,
    #         1.8719510016430925,
    #         -2.079638359900044
    #     ]),
    #     q=np.array([
    #         0.6507107019424438,
    #         0.08490215986967087,
    #         -0.7538508772850037,
    #         0.03280230239033699
    #     ])
    # )))

    # Viewer
    viewer = Viewer(renderer, resolutions=(1920, 1080))
    viewer.set_scene(scene)
    viewer.set_camera_xyz(x=0, y=0, z=1.0)

    while not viewer.closed:
        scene.step()
        scene.update_render()
        viewer.render()


main()
