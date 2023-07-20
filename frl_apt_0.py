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
    wall_cabinet_01 = builder.build_static(name='wall_cabinet')
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

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_cushion_03.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_cushion_03_cv_decomp.glb")
    cushion_03 = builder.build(name='cushion')
    cushion_03.set_pose(r2s(sapien.Pose(
        p=np.array([
            3.7939371987166415,
            0.5235895790058842,
            4.760266177124348
        ]),
        q=np.array([
            -0.03655696287751198,
            0.6259751319885254,
            -0.0029340607579797506,
            0.7789802551269531
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_sofa.glb")
    builder.add_nonconvex_collision_from_file(
        filename="replica_cad/objects/frl_apartment_sofa.glb")
    sofa = builder.build_static(name='sofa')
    sofa.set_pose(r2s(sapien.Pose(
        p=np.array([
            3.9079832792159843,
            0.4008761325929932,
            5.274956086467376
        ]),
        q=np.array([
            0.7070962190628052,
            -2.3067398657872218e-08,
            0.7071173191070557,
            9.23581922052108e-08
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_cushion_03.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_cushion_03_cv_decomp.glb")
    cushion_03 = builder.build(name='cushion')
    cushion_03.set_pose(r2s(sapien.Pose(
        p=np.array([
            3.7301722800704185,
            0.5097065302983645,
            5.289146874492183
        ]),
        q=np.array([
            0.9990090727806091,
            -0.022869598120450974,
            -0.0008942271815612912,
            0.03817138075828552
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_cushion_03.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_cushion_03_cv_decomp.glb")
    cushion_03 = builder.build(name='cushion')
    cushion_03.set_pose(r2s(sapien.Pose(
        p=np.array([
            3.773915728528579,
            0.5235825700203084,
            5.837694250298792
        ]),
        q=np.array([
            -0.031219935044646263,
            0.0326530784368515,
            0.019851701334118843,
            0.9987818598747253
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_table_04.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_table_04_cv_decomp.glb")
    table_04 = builder.build(name='table')
    table_04.set_pose(r2s(sapien.Pose(
        p=np.array([
            4.1999296153881005,
            0.4377844035625458,
            6.624393450696516
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
        filename="replica_cad/objects/frl_apartment_indoor_plant_02.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_indoor_plant_02_cv_decomp.glb")
    indoor_plant_02 = builder.build(name='indoor_plant')
    indoor_plant_02.set_pose(r2s(sapien.Pose(
        p=np.array([
            4.088084328551301,
            0.989412917817415,
            7.591271953147692
        ]),
        q=np.array([
            5.669991151080467e-05,
            9.794412108021788e-07,
            1.0,
            -3.5480688893585466e-06
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_indoor_plant_02.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_indoor_plant_02_cv_decomp.glb")
    indoor_plant_02 = builder.build(name='indoor_plant')
    indoor_plant_02.set_pose(r2s(sapien.Pose(
        p=np.array([
            0.698601750053826,
            0.9896594244183926,
            4.491301494246492
        ]),
        q=np.array([
            0.9999780654907227,
            4.3472223296703305e-06,
            -0.0065522389486432076,
            -0.000967822503298521
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_rug_01.glb")
    builder.add_nonconvex_collision_from_file(
        filename="replica_cad/objects/frl_apartment_rug_01.glb")
    rug_01 = builder.build_static(name='rug')
    rug_01.set_pose(r2s(sapien.Pose(
        p=np.array([
            1.2956230274687652,
            -0.0013210961847170799,
            2.31760215753231
        ]),
        q=np.array([
            0.8900392055511475,
            5.960464477539063e-08,
            0.4558842182159424,
            -4.470348358154297e-08
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_rug_02.glb")
    builder.add_nonconvex_collision_from_file(
        filename="replica_cad/objects/frl_apartment_rug_02.glb")
    rug_02 = builder.build_static(name='rug')
    rug_02.set_pose(r2s(sapien.Pose(
        p=np.array([
            2.477570047490146,
            -0.0013582630361395715,
            5.639058509118264
        ]),
        q=np.array([
            1.0,
            8.940696716308594e-08,
            0.0,
            0.0
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_table_03.glb")
    builder.add_nonconvex_collision_from_file(
        filename="replica_cad/objects/frl_apartment_table_03.glb")
    table_03 = builder.build_static(name='table')
    table_03.set_pose(r2s(sapien.Pose(
        p=np.array([
            1.98938657626566,
            0.328899428627999,
            5.8215795328128355
        ]),
        q=np.array([
            0.9499984383583069,
            -4.320768766774563e-06,
            -0.3122546374797821,
            -2.3089407932275208e-06
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_cushion_01.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_cushion_01_cv_decomp.glb")
    cushion_01 = builder.build(name='cushion')
    cushion_01.set_pose(r2s(sapien.Pose(
        p=np.array([
            2.024985611883048,
            0.5476084927498879,
            7.447507726129283
        ]),
        q=np.array([
            0.09089621901512146,
            0.9222638607025146,
            0.19076724350452423,
            -0.3236900269985199
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_chair_01.glb")
    builder.add_nonconvex_collision_from_file(
        filename="replica_cad/objects/frl_apartment_chair_01.glb")
    chair_01 = builder.build_static(name='chair')
    chair_01.set_pose(r2s(sapien.Pose(
        p=np.array([
            1.9716448279676861,
            0.3648954460872759,
            7.533762805288695
        ]),
        q=np.array([
            0.9998177886009216,
            -4.720749711850658e-05,
            -0.019088922068476677,
            2.313570621481631e-05
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_chair_01.glb")
    builder.add_nonconvex_collision_from_file(
        filename="replica_cad/objects/frl_apartment_chair_01.glb")
    chair_01 = builder.build_static(name='chair')
    chair_01.set_pose(r2s(sapien.Pose(
        p=np.array([
            1.1580844416585023,
            0.3648966937325895,
            7.578433390706778
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
        filename="replica_cad/objects/frl_apartment_towel.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_towel_cv_decomp.glb")
    towel = builder.build(name='towel')
    towel.set_pose(r2s(sapien.Pose(
        p=np.array([
            1.1493457676737744,
            0.47306254148566707,
            7.42787559277667
        ]),
        q=np.array([
            0.03313228487968445,
            -0.058175984770059586,
            0.9977216124534607,
            -0.00833211187273264
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_lamp_01.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_lamp_01_cv_decomp.glb")
    lamp_01 = builder.build(name='lamp')
    lamp_01.set_pose(r2s(sapien.Pose(
        p=np.array([
            0.5354144371977808,
            0.7759982070792253,
            7.7025976869031645
        ]),
        q=np.array([
            0.8193490505218506,
            -0.00029130876646377146,
            0.5732948780059814,
            -0.00021113446564413607
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
            0.5659859003180567,
            0.29405346729303083,
            7.778309143315614
        ]),
        q=np.array([
            0.7070065140724182,
            -8.669299859320745e-06,
            -0.7072070240974426,
            -1.3410301562544191e-06
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_remote-control_01.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_remote-control_01_cv_decomp.glb")
    remote_control_01 = builder.build(name='remote_control')
    remote_control_01.set_pose(r2s(sapien.Pose(
        p=np.array([
            2.9747319273530493,
            0.6320663210434029,
            7.612372864074937
        ]),
        q=np.array([
            0.9869281649589539,
            -7.332592213060707e-05,
            -0.16116078197956085,
            4.758995146403322e-06
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_remote-control_01.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_remote-control_01_cv_decomp.glb")
    remote_control_01 = builder.build(name='remote_control')
    remote_control_01.set_pose(r2s(sapien.Pose(
        p=np.array([
            2.8196923746659297,
            0.6321219321975265,
            7.662763583385308
        ]),
        q=np.array([
            0.9999997019767761,
            0.0007720404537394643,
            0.0,
            0.0
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_tv_object.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_tv_object_cv_decomp.glb")
    tv_object = builder.build(name='tablet')
    tv_object.set_pose(r2s(sapien.Pose(
        p=np.array([
            3.581236342031576,
            0.649152358975896,
            7.674884828187777
        ]),
        q=np.array([
            0.993788480758667,
            -0.00017936652875505388,
            0.11128532886505127,
            -2.4090618353511672e-06
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_tv_screen.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_tv_screen_cv_decomp.glb")
    tv_screen = builder.build_static(name='tv_screen')
    tv_screen.set_pose(r2s(sapien.Pose(
        p=np.array([
            2.6130616384907626,
            1.3601730293239112,
            7.848809196723387
        ]),
        q=np.array([
            1.0,
            8.940696716308594e-08,
            0.0,
            0.0
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_wall_cabinet_02.glb")
    builder.add_nonconvex_collision_from_file(
        filename="replica_cad/objects/frl_apartment_wall_cabinet_02.glb")
    wall_cabinet_02 = builder.build_static(name='wall_cabinet')
    wall_cabinet_02.set_pose(r2s(sapien.Pose(
        p=np.array([
            0.543024370367041,
            1.0820363630109093,
            5.410867965575855
        ]),
        q=np.array([
            0.7071903944015503,
            -0.0001440457854187116,
            0.7070231437683105,
            0.0001687336916802451
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_picture_04.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/frl_apartment_picture_04.glb")
    picture_04 = builder.build(name='picture')
    picture_04.set_pose(r2s(sapien.Pose(
        p=np.array([
            0.5990803711981928,
            1.2333409906568122,
            5.39041174288141
        ]),
        q=np.array([
            0.7171033620834351,
            -0.0031032192055135965,
            -0.6969488263130188,
            -0.003943352960050106
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_book_02.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/frl_apartment_book_02.glb")
    book_02 = builder.build(name='book')
    book_02.set_pose(r2s(sapien.Pose(
        p=np.array([
            0.5279294364154339,
            1.2356480422313325,
            5.217218704579864
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
        filename="replica_cad/objects/frl_apartment_book_03.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/frl_apartment_book_03.glb")
    book_03 = builder.build(name='book')
    book_03.set_pose(r2s(sapien.Pose(
        p=np.array([
            0.5523298374990443,
            1.2088190608414795,
            5.138131119853175
        ]),
        q=np.array([
            0.9999945759773254,
            0.0020103156566619873,
            0.002608038019388914,
            -6.915728590684012e-05
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_book_04.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/frl_apartment_book_04.glb")
    book_04 = builder.build(name='book')
    book_04.set_pose(r2s(sapien.Pose(
        p=np.array([
            0.4901069205333909,
            1.184328469422195,
            5.742168417921027
        ]),
        q=np.array([
            0.9192952513694763,
            0.39356234669685364,
            0.00206455634906888,
            -0.0008176929550245404
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_book_06.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/frl_apartment_book_06.glb")
    book_06 = builder.build(name='book')
    book_06.set_pose(r2s(sapien.Pose(
        p=np.array([
            0.5814781167140576,
            1.4479360203725353,
            5.390846063112845
        ]),
        q=np.array([
            0.7071501016616821,
            0.7070558667182922,
            -0.0022497340105473995,
            0.0024011495988816023
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_book_05.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/frl_apartment_book_05.glb")
    book_05 = builder.build(name='book')
    book_05.set_pose(r2s(sapien.Pose(
        p=np.array([
            0.5770455012707786,
            1.4593083199361199,
            5.599870757938208
        ]),
        q=np.array([
            0.8598712086677551,
            -0.510503351688385,
            -0.0025743639562278986,
            -0.0010884676594287157
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_book_01.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/frl_apartment_book_01.glb")
    book_01 = builder.build(name='book')
    book_01.set_pose(r2s(sapien.Pose(
        p=np.array([
            0.5462155844418204,
            1.2165419161236166,
            5.070820428042975
        ]),
        q=np.array([
            0.999997615814209,
            0.00013577799836639315,
            -0.0021669920533895493,
            0.00023254477127920836
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_book_05.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/frl_apartment_book_05.glb")
    book_05 = builder.build(name='book')
    book_05.set_pose(r2s(sapien.Pose(
        p=np.array([
            0.5181468461523764,
            1.213434677803889,
            5.168222745909588
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
        filename="replica_cad/objects/frl_apartment_book_01.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/frl_apartment_book_01.glb")
    book_01 = builder.build(name='book')
    book_01.set_pose(r2s(sapien.Pose(
        p=np.array([
            0.5447984161544334,
            1.2165447405167809,
            5.041646835117987
        ]),
        q=np.array([
            0.9999992251396179,
            0.00014283423661254346,
            -0.0012190560810267925,
            0.0002078959805658087
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_book_01.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/frl_apartment_book_01.glb")
    book_01 = builder.build(name='book')
    book_01.set_pose(r2s(sapien.Pose(
        p=np.array([
            0.5463907981726915,
            1.2165413048331297,
            5.105510127477967
        ]),
        q=np.array([
            0.9999992251396179,
            -2.6207411792711355e-05,
            0.0012278396170586348,
            0.00020357640460133553
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_book_01.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/frl_apartment_book_01.glb")
    book_01 = builder.build(name='book')
    book_01.set_pose(r2s(sapien.Pose(
        p=np.array([
            0.5445772714234409,
            0.5909459140270678,
            5.3021365312469095
        ]),
        q=np.array([
            0.8190692663192749,
            -0.5230022668838501,
            -0.19883067905902863,
            -0.12673039734363556
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_book_05.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/frl_apartment_book_05.glb")
    book_05 = builder.build(name='book')
    book_05.set_pose(r2s(sapien.Pose(
        p=np.array([
            0.4400757871031084,
            0.5527911758492078,
            5.578849522951047
        ]),
        q=np.array([
            0.500917375087738,
            -0.5011126399040222,
            0.49887391924858093,
            0.49909183382987976
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_book_02.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/frl_apartment_book_02.glb")
    book_02 = builder.build(name='book')
    book_02.set_pose(r2s(sapien.Pose(
        p=np.array([
            0.5300189124623629,
            0.5695436590310216,
            5.155018205466123
        ]),
        q=np.array([
            0.328755646944046,
            0.3290249705314636,
            -0.6259019374847412,
            0.6260265111923218
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_book_06.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/frl_apartment_book_06.glb")
    book_06 = builder.build(name='book')
    book_06.set_pose(r2s(sapien.Pose(
        p=np.array([
            0.5816907324419388,
            0.3240568431774239,
            5.08593092290904
        ]),
        q=np.array([
            -2.6031337256426923e-05,
            0.9999998807907104,
            0.00021566955547314137,
            -0.000447602680651471
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_book_04.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/frl_apartment_book_04.glb")
    book_04 = builder.build(name='book')
    book_04.set_pose(r2s(sapien.Pose(
        p=np.array([
            0.5301890798443807,
            0.1974399028997863,
            5.512663551062584
        ]),
        q=np.array([
            0.6199036836624146,
            0.620073676109314,
            -0.3398945927619934,
            0.34014666080474854
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_book_03.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/frl_apartment_book_03.glb")
    book_03 = builder.build(name='book')
    book_03.set_pose(r2s(sapien.Pose(
        p=np.array([
            0.5525290459410067,
            0.2133696482019867,
            5.567515491605265
        ]),
        q=np.array([
            0.7071211934089661,
            -0.7070923447608948,
            -0.0003732276090886444,
            -7.459810876753181e-05
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_book_03.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/frl_apartment_book_03.glb")
    book_03 = builder.build(name='book')
    book_03.set_pose(r2s(sapien.Pose(
        p=np.array([
            0.4916593302123013,
            1.187537058918173,
            5.717546060394388
        ]),
        q=np.array([
            -0.4125613272190094,
            0.9109271764755249,
            -0.000858642568346113,
            -0.0020309488754719496
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_table_02.glb")
    builder.add_nonconvex_collision_from_file(
        filename="replica_cad/objects/frl_apartment_table_02.glb")
    table_02 = builder.build_static(name='table')
    table_02.set_pose(r2s(sapien.Pose(
        p=np.array([
            0.8955980928025115,
            0.6050039380843264,
            2.4133185315665733
        ]),
        q=np.array([
            0.3067869544029236,
            -3.286929859314114e-05,
            0.9517782330513,
            3.353481588419527e-05
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_bowl_07.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_bowl_07_cv_decomp.glb")
    bowl_07 = builder.build(name='bowl')
    bowl_07.set_pose(r2s(sapien.Pose(
        p=np.array([
            0.5239684729003784,
            0.6529611752447907,
            2.300429283186491
        ]),
        q=np.array([
            0.9999998211860657,
            0.0005980199202895164,
            0.0,
            0.0
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_kitchen_utensil_05.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_kitchen_utensil_05_cv_decomp.glb")
    kitchen_utensil_05 = builder.build(name='kitchen_utensil')
    kitchen_utensil_05.set_pose(r2s(sapien.Pose(
        p=np.array([
            0.9775079464226497,
            0.6560430243813424,
            2.6453982116452166
        ]),
        q=np.array([
            0.9494752287864685,
            0.0027169042732566595,
            -0.31381189823150635,
            -0.003393356455489993
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_pan_01.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_pan_01_cv_decomp.glb")
    pan_01 = builder.build(name='pan')
    pan_01.set_pose(r2s(sapien.Pose(
        p=np.array([
            0.5070147814913071,
            0.2753303231047769,
            2.2997903759476386
        ]),
        q=np.array([
            0.9999997615814209,
            0.0006905339541845024,
            0.0,
            0.0
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_kitchen_utensil_01.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_kitchen_utensil_01_cv_decomp.glb")
    kitchen_utensil_01 = builder.build(name='kitchen_utensil')
    kitchen_utensil_01.set_pose(r2s(sapien.Pose(
        p=np.array([
            1.056504680363681,
            0.31421184111688405,
            2.677142093626222
        ]),
        q=np.array([
            0.9446590542793274,
            0.00010474729788256809,
            -0.328053742647171,
            -0.00014383938105311245
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_choppingboard_02.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/frl_apartment_choppingboard_02.glb")
    choppingboard_02 = builder.build(name='chopping_board')
    choppingboard_02.set_pose(r2s(sapien.Pose(
        p=np.array([
            1.0512380365598288,
            0.24125624506094362,
            2.6805861263589583
        ]),
        q=np.array([
            0.9486736059188843,
            1.5354331480921246e-05,
            -0.3162568211555481,
            -3.497805209917715e-06
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
    viewer.set_camera_xyz(x=-2.0, y=0, z=1.5)

    while not viewer.closed:
        scene.step()
        scene.update_render()
        viewer.render()


main()
