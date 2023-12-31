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


def build_scene(engine: sapien.Engine):
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

    # Objects
    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_bike_01.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_bike_01_cv_decomp.glb")
    bike_01 = builder.build(name='bike')
    # bike_01.set_pose(r2s(sapien.Pose(
    #     p=np.array([
    #         4.107420409715755,
    #         0.4545634772665478,
    #         -0.9759617637063622
    #     ]),
    #     q=np.array([
    #         0.1531376838684082,
    #         0.07608834654092789,
    #         0.985193133354187,
    #         0.012405824847519398
    #     ])
    # )))
    bike_01.set_pose(sapien.Pose([0.483688, -3.78923, 0.444299],
                     [0.593394, 0.646121, 0.286293, 0.38529]))

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
        filename="replica_cad/objects/convex/frl_apartment_sofa_cv_decomp.glb")
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

    # builder = scene.create_actor_builder()
    # builder.add_visual_from_file(
    #     filename="replica_cad/objects/frl_apartment_indoor_plant_02.glb")
    # builder.add_collision_from_file(
    #     filename="replica_cad/objects/convex/frl_apartment_indoor_plant_02_cv_decomp.glb")
    # indoor_plant_02 = builder.build(name='indoor_plant')
    # indoor_plant_02.set_pose(r2s(sapien.Pose(
    #     p=np.array([
    #         4.088084328551301,
    #         0.989412917817415,
    #         7.591271953147692
    #     ]),
    #     q=np.array([
    #         5.669991151080467e-05,
    #         9.794412108021788e-07,
    #         1.0,
    #         -3.5480688893585466e-06
    #     ])
    # )))

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
        filename="replica_cad/objects/convex/frl_apartment_table_03_cv_decomp.glb")
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
        filename="replica_cad/objects/convex/frl_apartment_chair_01_cv_decomp.glb")
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
        filename="replica_cad/objects/convex/frl_apartment_chair_01_cv_decomp.glb")
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
        filename="replica_cad/objects/convex/frl_apartment_wall_cabinet_02_cv_decomp.glb")
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
        filename="replica_cad/objects/convex/frl_apartment_table_02_cv_decomp.glb")
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

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_chair_04.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_chair_04_cv_decomp.glb")
    chair_04 = builder.build(name='chair')
    chair_04.set_pose(r2s(sapien.Pose(
        p=np.array([
            -2.2401796989932157,
            0.46910255187718897,
            3.8506248502297877
        ]),
        q=np.array([
            0.7070959806442261,
            -6.070461040508235e-07,
            -0.7071175575256348,
            -2.59844313177382e-07
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_chair_04.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_chair_04_cv_decomp.glb")
    # chair_04 = builder.build(name='chair')
    chair_04 = builder.build_static(name='chair')
    chair_04.set_pose(r2s(sapien.Pose(
        p=np.array([
            -2.26952179929032,
            0.4704890177872069,
            -2.0904483131779408
        ]),
        q=np.array([
            0.6107712984085083,
            -0.0016085348324850202,
            -0.7918027639389038,
            -0.0020697491709142923
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_mat.glb")
    builder.add_nonconvex_collision_from_file(
        filename="replica_cad/objects/frl_apartment_mat.glb")
    mat = builder.build_static(name='mat')
    mat.set_pose(r2s(sapien.Pose(
        p=np.array([
            -1.550700879250923,
            0.003622475121917826,
            1.5342658990391274
        ]),
        q=np.array([
            0.7071068286895752,
            2.9802322387695312e-08,
            -0.7071068286895752,
            5.960464477539063e-08
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_indoor_plant_01.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_indoor_plant_01_cv_decomp.glb")
    indoor_plant_01 = builder.build(name='indoor_plant')
    # indoor_plant_01.set_pose(r2s(sapien.Pose(
    #     p=np.array([
    #         3.4197291647131647,
    #         1.692999413000504,
    #         -4.133856821097971
    #     ]),
    #     q=np.array([
    #         0.9999988079071045,
    #         -5.1615941629279405e-05,
    #         -0.00154321757145226,
    #         2.0375915710246773e-07
    #     ])
    # )))
    indoor_plant_01.set_pose(sapien.Pose(
        [4.07263, -3.31444, 1.68303], [0.427417, 0.42741, -0.563305, -0.563314]))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_table_01.glb")
    builder.add_nonconvex_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_table_01_cv_decomp.glb")
    table_01 = builder.build_static(name='table')
    table_01.set_pose(r2s(sapien.Pose(
        p=np.array([
            0.41439489989170397,
            0.6632212233962491,
            -0.17470338097700733
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
        filename="replica_cad/objects/frl_apartment_monitor.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_monitor_cv_decomp.glb")
    monitor = builder.build_static(name='monitor')
    monitor.set_pose(r2s(sapien.Pose(
        p=np.array([
            0.4372712999399257,
            1.543347558972872,
            -0.5273685490286963
        ]),
        q=np.array([
            -4.371138828673793e-08,
            -1.7763568394002505e-15,
            1.0,
            -8.940696716308594e-08
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_camera_02.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_camera_02_cv_decomp.glb")
    camera_02 = builder.build(name='camera')
    # camera_02.set_pose(r2s(sapien.Pose(
    #     p=np.array([
    #         0.4675843503131788,
    #         1.764217499666931,
    #         -0.5872958740340483
    #     ]),
    #     q=np.array([
    #         0.7091919779777527,
    #         -0.028345324099063873,
    #         -0.7040526270866394,
    #         0.023521307855844498
    #     ])
    # )))
    camera_02.set_pose(sapien.Pose(
        [0.583088, -0.467597, 1.79718], [-0.0786479, 6.26256e-05, -0.708008, -0.701811]))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_plate_01.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_plate_01_cv_decomp.glb")
    plate_01 = builder.build(name='plate')
    plate_01.set_pose(r2s(sapien.Pose(
        p=np.array([
            -2.0298574478178,
            0.9644865502930506,
            2.6329260477825285
        ]),
        q=np.array([
            0.9703373908996582,
            -8.416349373874255e-06,
            0.24175472557544708,
            8.566543669985549e-07
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_cup_03.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_cup_03_cv_decomp.glb")
    cup_03 = builder.build(name='cup')
    cup_03.set_pose(r2s(sapien.Pose(
        p=np.array([
            -2.0326713781909205,
            0.9989798296190667,
            2.6403462389054058
        ]),
        q=np.array([
            0.6334787011146545,
            0.06220954656600952,
            -0.771053671836853,
            -0.017633618786931038
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_cup_01.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_cup_01_cv_decomp.glb")
    cup_01 = builder.build(name='cup')
    cup_01.set_pose(r2s(sapien.Pose(
        p=np.array([
            -2.2408843935257448,
            0.983924483311843,
            2.5616791893638684
        ]),
        q=np.array([
            0.6068241596221924,
            -2.8294665753492154e-05,
            -0.7948361039161682,
            -1.4519568139803596e-05
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_plate_02.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_plate_02_cv_decomp.glb")
    plate_02 = builder.build(name='plate')
    plate_02.set_pose(r2s(sapien.Pose(
        p=np.array([
            -2.080934439552948,
            0.9903172720223665,
            2.2550046161632054
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
        filename="replica_cad/objects/frl_apartment_bowl_06.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_bowl_06_cv_decomp.glb")
    bowl_06 = builder.build(name='bowl')
    bowl_06.set_pose(r2s(sapien.Pose(
        p=np.array([
            -2.0212689069012413,
            0.9860159838062513,
            1.962052314089754
        ]),
        q=np.array([
            0.9999998807907104,
            0.00048828122089616954,
            0.0,
            0.0
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_cup_02.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_cup_02_cv_decomp.glb")
    cup_02 = builder.build(name='cup')
    cup_02.set_pose(r2s(sapien.Pose(
        p=np.array([
            -2.2144558426775047,
            0.9960266370638448,
            2.0379445562275342
        ]),
        q=np.array([
            0.7473792433738708,
            -1.2943657566211186e-05,
            0.664397656917572,
            1.2434369637048803e-05
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_sponge_dish.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_sponge_dish_cv_decomp.glb")
    sponge_dish = builder.build(name='kitchen_utensil')
    sponge_dish.set_pose(r2s(sapien.Pose(
        p=np.array([
            -2.3765419384008477,
            0.9901297256125796,
            2.0387958012602496
        ]),
        q=np.array([
            -0.08086394518613815,
            1.2481838894018438e-05,
            0.9967251420021057,
            -9.348681487608701e-05
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_kitchen_utensil_02.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_kitchen_utensil_02_cv_decomp.glb")
    kitchen_utensil_02 = builder.build(name='kitchen_utensil')
    kitchen_utensil_02.set_pose(r2s(sapien.Pose(
        p=np.array([
            -2.39422155468651,
            1.0234393756836653,
            1.9150788320466745
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
        filename="replica_cad/objects/frl_apartment_small_appliance_01.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_small_appliance_01_cv_decomp.glb")
    small_appliance_01 = builder.build(name='small_appliance')
    small_appliance_01.set_pose(r2s(sapien.Pose(
        p=np.array([
            -2.1577768278573473,
            1.094117187301409,
            1.1320104880828685
        ]),
        q=np.array([
            0.7160552144050598,
            -1.2546704056148883e-05,
            -0.6980436444282532,
            6.473955181718338e-06
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_kitchen_utensil_04.glb")
    builder.add_nonconvex_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_kitchen_utensil_04_cv_decomp.glb")
    kitchen_utensil_04 = builder.build_static(name='kitchen_utensil')
    # kitchen_utensil_04.set_pose(r2s(sapien.Pose(
    #     p=np.array([
    #         -2.393288374197755,
    #         0.9958334886794491,
    #         0.739746062509862
    #     ]),
    #     q=np.array([
    #         0.7067710161209106,
    #         0.00018358776287641376,
    #         -0.7074423432350159,
    #         0.0001732671371428296
    #     ])
    # )))
    kitchen_utensil_04.set_pose(sapien.Pose(
        [-0.739697, 2.3934, 0.916119], [-0.000382662, -0.000314354, -0.707182, -0.707032]))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_kitchen_utensil_03.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_kitchen_utensil_03_cv_decomp.glb")
    kitchen_utensil_03 = builder.build(name='kitchen_utensil')
    kitchen_utensil_03.set_pose(r2s(sapien.Pose(
        p=np.array([
            -2.444240719419895,
            1.0,
            0.8247272897509641
        ]),
        q=np.array([
            0.9999996423721313,
            0.0008457278599962592,
            0.0,
            0.0
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_kitchen_utensil_03.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_kitchen_utensil_03_cv_decomp.glb")
    kitchen_utensil_03 = builder.build(name='kitchen_utensil')
    kitchen_utensil_03.set_pose(r2s(sapien.Pose(
        p=np.array([
            -2.444071442228733,
            1.0,
            0.7699508810956104
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
        filename="replica_cad/objects/frl_apartment_kitchen_utensil_03.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_kitchen_utensil_03_cv_decomp.glb")
    kitchen_utensil_03 = builder.build(name='kitchen_utensil')
    kitchen_utensil_03.set_pose(r2s(sapien.Pose(
        p=np.array([
            -2.444080978971897,
            1.0,
            0.7153729571573297
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
        filename="replica_cad/objects/frl_apartment_kitchen_utensil_03.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_kitchen_utensil_03_cv_decomp.glb")
    kitchen_utensil_03 = builder.build(name='kitchen_utensil')
    kitchen_utensil_03.set_pose(r2s(sapien.Pose(
        p=np.array([
            -2.439621359449802,
            1.0,
            0.6612882268538393
        ]),
        q=np.array([
            0.9999999403953552,
            0.00034526700619608164,
            0.0,
            0.0
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_kitchen_utensil_03.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_kitchen_utensil_03_cv_decomp.glb")
    kitchen_utensil_03 = builder.build(name='kitchen_utensil')
    kitchen_utensil_03.set_pose(r2s(sapien.Pose(
        p=np.array([
            -2.3755842748703344,
            0.97,
            0.8185636176340143
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
        filename="replica_cad/objects/frl_apartment_kitchen_utensil_03.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_kitchen_utensil_03_cv_decomp.glb")
    kitchen_utensil_03 = builder.build(name='kitchen_utensil')
    kitchen_utensil_03.set_pose(r2s(sapien.Pose(
        p=np.array([
            -2.3752742529920505,
            0.97,
            0.7614319548317205
        ]),
        q=np.array([
            0.999969482421875,
            -7.394045678665861e-05,
            -0.007803070824593306,
            0.00037530294503085315
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_kitchen_utensil_03.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_kitchen_utensil_03_cv_decomp.glb")
    kitchen_utensil_03 = builder.build(name='kitchen_utensil')
    kitchen_utensil_03.set_pose(r2s(sapien.Pose(
        p=np.array([
            -2.3752459747597836,
            0.97,
            0.7068356780545527
        ]),
        q=np.array([
            0.999983549118042,
            -0.0001030511484714225,
            -0.005734181962907314,
            -0.00010014435974881053
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_kitchen_utensil_03.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_kitchen_utensil_03_cv_decomp.glb")
    kitchen_utensil_03 = builder.build(name='kitchen_utensil')
    kitchen_utensil_03.set_pose(r2s(sapien.Pose(
        p=np.array([
            -2.3753357865870286,
            0.97,
            0.6512400181799256
        ]),
        q=np.array([
            0.9999990463256836,
            -0.00012087525828974321,
            -0.0013741395669057965,
            6.691049202345312e-05
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_kitchen_utensil_06.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_kitchen_utensil_06_cv_decomp.glb")
    kitchen_utensil_06 = builder.build(name='kitchen_utensil')
    kitchen_utensil_06.set_pose(r2s(sapien.Pose(
        p=np.array([
            -2.37639330363218,
            1.0064355578506365,
            0.5168919279967668
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
        filename="replica_cad/objects/frl_apartment_choppingboard_02.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/frl_apartment_choppingboard_02.glb")
    choppingboard_02 = builder.build(name='chopping_board')
    choppingboard_02.set_pose(r2s(sapien.Pose(
        p=np.array([
            -2.462873620451536,
            1.080273484338531,
            0.31750580947430523
        ]),
        q=np.array([
            0.5708795189857483,
            0.41823890805244446,
            0.5699824094772339,
            -0.41748398542404175
        ])
    )))

    # builder = scene.create_actor_builder()
    # builder.add_visual_from_file(
    #     filename="replica_cad/objects/frl_apartment_picture_03.glb")
    # builder.add_collision_from_file(
    #     filename="replica_cad/objects/frl_apartment_picture_03.glb")
    # picture_03 = builder.build(name='picture')
    # picture_03.set_pose(r2s(sapien.Pose(
    #     p=np.array([
    #         -2.2339015470141543,
    #         1.0536260587741557,
    #         0.3960886158897984
    #     ]),
    #     q=np.array([
    #         -0.09706857055425644,
    #         -0.01689460314810276,
    #         0.9803974032402039,
    #         -0.1706264317035675
    #     ])
    # )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_knifeblock.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/frl_apartment_knifeblock.glb")
    knifeblock = builder.build(name='knife_block')
    knifeblock.set_pose(r2s(sapien.Pose(
        p=np.array([
            -2.3510213018811834,
            1.0769024740322775,
            0.3315389076025292
        ]),
        q=np.array([
            0.9999838471412659,
            1.4381769233295927e-06,
            0.005683788564056158,
            7.434350095536502e-07
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_kitchen_utensil_08.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_kitchen_utensil_08_cv_decomp.glb")
    kitchen_utensil_08 = builder.build(name='kitchen_utensil')
    kitchen_utensil_08.set_pose(r2s(sapien.Pose(
        p=np.array([
            -2.1922357271053294,
            1.0207934350929246,
            0.31032127808501464
        ]),
        q=np.array([
            -0.22345943748950958,
            -6.918581675563473e-06,
            0.9747132062911987,
            -6.492099146271357e-06
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_kitchen_utensil_09.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_kitchen_utensil_09_cv_decomp.glb")
    kitchen_utensil_09 = builder.build(name='kitchen_utensil')
    kitchen_utensil_09.set_pose(r2s(sapien.Pose(
        p=np.array([
            -2.2383486407085798,
            1.0432830064032899,
            0.20288169660460803
        ]),
        q=np.array([
            0.9999967813491821,
            -7.581105455756187e-05,
            -0.0025360439904034138,
            4.968914254277479e-06
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_small_appliance_02.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_small_appliance_02_cv_decomp.glb")
    small_appliance_02 = builder.build(name='small_appliance')
    small_appliance_02.set_pose(r2s(sapien.Pose(
        p=np.array([
            -2.323086477026056,
            1.0883071898357457,
            0.000176499952074239
        ]),
        q=np.array([
            0.7071254849433899,
            -8.414540388912428e-06,
            -0.7070881128311157,
            -5.5371383496094495e-06
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_setupbox.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_setupbox_cv_decomp.glb")
    setupbox = builder.build_static(name='tablet')
    setupbox.set_pose(r2s(sapien.Pose(
        p=np.array([
            0.4426179806521304,
            1.0454591548309102,
            -0.5731640165488877
        ]),
        q=np.array([
            0.5566916465759277,
            0.8307193517684937,
            0.0,
            0.0
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_picture_02.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/frl_apartment_picture_02.glb")
    picture_02 = builder.build(name='picture')
    picture_02.set_pose(r2s(sapien.Pose(
        p=np.array([
            -2.422472662890697,
            1.5306948527030095,
            -1.5477773880428713
        ]),
        q=np.array([
            0.6994363069534302,
            0.09628245234489441,
            -0.701562225818634,
            0.0965871587395668
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_handbag.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_handbag_cv_decomp.glb")
    handbag = builder.build(name='handbag')
    handbag.set_pose(r2s(sapien.Pose(
        p=np.array([
            -2.323607023799576,
            1.8719510016430925,
            -2.079638359900044
        ]),
        q=np.array([
            0.6507107019424438,
            0.08490215986967087,
            -0.7538508772850037,
            0.03280230239033699
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_rack_01.glb")
    builder.add_nonconvex_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_rack_01_cv_decomp.glb")
    rack_01 = builder.build_static(name='rack')
    rack_01.set_pose(r2s(sapien.Pose(
        p=np.array([
            -2.0790531625404522,
            0.31483264369793473,
            -2.5388489370048997
        ]),
        q=np.array([
            0.9999990463256836,
            -5.110719598633295e-07,
            0.0013810491655021906,
            -7.098410605976824e-06
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_umbrella.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_umbrella_cv_decomp.glb")
    umbrella = builder.build(name='umbrella')
    umbrella.set_pose(r2s(sapien.Pose(
        p=np.array([
            -2.2892425340503277,
            0.5602593977738366,
            -2.7617138196780906
        ]),
        q=np.array([
            -0.0806984230875969,
            0.00957900658249855,
            0.002710412722080946,
            0.9966889023780823
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_shoe_01.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_shoe_01_cv_decomp.glb")
    shoe_01 = builder.build(name='shoe')
    shoe_01.set_pose(r2s(sapien.Pose(
        p=np.array([
            -1.8199461568230866,
            0.3602124978289363,
            -2.5655168133811905
        ]),
        q=np.array([
            0.008195980452001095,
            -0.0001868897525127977,
            0.9999253749847412,
            0.009062045253813267
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_shoe_02.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_shoe_02_cv_decomp.glb")
    shoe_02 = builder.build(name='shoe')
    shoe_02.set_pose(r2s(sapien.Pose(
        p=np.array([
            -2.093289691405659,
            0.3602185008550056,
            -2.5665119720285725
        ]),
        q=np.array([
            -0.00042287903488613665,
            -0.00020568784384522587,
            0.9999608993530273,
            0.008827954530715942
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_shoe_03.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_shoe_03_cv_decomp.glb")
    shoe_03 = builder.build(name='shoe')
    shoe_03.set_pose(r2s(sapien.Pose(
        p=np.array([
            -2.333083215536946,
            0.357759296656825,
            -2.5114650538477123
        ]),
        q=np.array([
            0.03995947539806366,
            -0.0018959534354507923,
            0.9990471601486206,
            0.017451461404561996
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_shoe_04.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_shoe_04_cv_decomp.glb")
    shoe_04 = builder.build(name='shoe')
    shoe_04.set_pose(r2s(sapien.Pose(
        p=np.array([
            -1.8625753647610575,
            0.16332561435833237,
            -2.559375085535354
        ]),
        q=np.array([
            0.0128908921033144,
            -0.002723709214478731,
            0.9998583197593689,
            -0.010479584336280823
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_cloth_02.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_cloth_02_cv_decomp.glb")
    cloth_02 = builder.build_static(name='cloth')
    cloth_02.set_pose(r2s(sapien.Pose(
        p=np.array([
            -2.206169113093824,
            0.9419560058682306,
            -2.332082291237297
        ]),
        q=np.array([
            0.049986131489276886,
            5.587935447692871e-09,
            0.9987499713897705,
            -5.960464477539063e-08
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_picture_01.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/frl_apartment_picture_01.glb")
    picture_01 = builder.build_static(name='picture')
    picture_01.set_pose(r2s(sapien.Pose(
        p=np.array([
            -0.3670506930272897,
            1.3813003051753292,
            -2.975669314398404
        ]),
        q=np.array([
            0.7149065732955933,
            8.940696716308594e-08,
            0.6992200613021851,
            -5.960464477539063e-08
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_beanbag.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_beanbag_cv_decomp.glb")
    beanbag = builder.build(name='beanbag')
    beanbag.set_pose(r2s(sapien.Pose(
        p=np.array([
            3.7999427476839047,
            0.3372600305818876,
            1.5678577269893819
        ]),
        q=np.array([
            0.43730679154396057,
            -0.001723677385598421,
            0.8989261388778687,
            0.02630022168159485
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_beanbag.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_beanbag_cv_decomp.glb")
    beanbag = builder.build(name='beanbag')
    beanbag.set_pose(r2s(sapien.Pose(
        p=np.array([
            3.8958638610694893,
            0.33725961521804765,
            3.1576456328782148
        ]),
        q=np.array([
            0.22654053568840027,
            0.0041658030822873116,
            0.9736451506614685,
            0.026024524122476578
        ])
    )))

    # builder = scene.create_actor_builder()
    # builder.add_visual_from_file(
    #     filename="replica_cad/objects/frl_apartment_bike_02.glb")
    # builder.add_collision_from_file(
    #     filename="replica_cad/objects/convex/frl_apartment_bike_02_cv_decomp.glb")
    # bike_02 = builder.build(name='bike')
    # bike_02.set_pose(r2s(sapien.Pose(
    #     p=np.array([
    #         -0.551209939169045,
    #         0.4536360645402108,
    #         -1.0196206032539004
    #     ]),
    #     q=np.array([
    #         -0.01878906413912773,
    #         0.08465909212827682,
    #         0.9962324500083923,
    #         -0.0009783057030290365
    #     ])
    # )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_lamp_02.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_lamp_02_cv_decomp.glb")
    lamp_02 = builder.build(name='lamp')
    lamp_02.set_pose(r2s(sapien.Pose(
        p=np.array([
            0.6497997234400827,
            1.1433340348303318,
            2.177875831235724
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
        filename="replica_cad/objects/frl_apartment_lamp_02.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_lamp_02_cv_decomp.glb")
    lamp_02 = builder.build(name='lamp')
    lamp_02.set_pose(r2s(sapien.Pose(
        p=np.array([
            1.1436370800074656,
            1.1433689631521702,
            2.5776448990000063
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
        filename="replica_cad/objects/frl_apartment_cloth_01.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_cloth_01_cv_decomp.glb")
    cloth_01 = builder.build_static(name='cloth')
    cloth_01.set_pose(r2s(sapien.Pose(
        p=np.array([
            -2.2343876322829805,
            1.0719663597103843,
            -1.8839233074713455
        ]),
        q=np.array([
            -4.371138828673793e-08,
            -1.7763568394002505e-15,
            1.0,
            -8.940696716308594e-08
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_cloth_03.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_cloth_03_cv_decomp.glb")
    cloth_03 = builder.build_static(name='cloth')
    cloth_03.set_pose(r2s(sapien.Pose(
        p=np.array([
            -2.245216549769531,
            1.202465687575864,
            -2.1152903009276547
        ]),
        q=np.array([
            0.0425836481153965,
            3.725290298461914e-09,
            0.9990930557250977,
            -8.940696716308594e-08
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_shoebox_01.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_shoebox_01_cv_decomp.glb")
    shoebox_01 = builder.build(name='box')
    shoebox_01.set_pose(r2s(sapien.Pose(
        p=np.array([
            -2.302423760735717,
            1.3857124848291278,
            -1.083843636032615
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
        filename="replica_cad/objects/frl_apartment_monitor_stand.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_monitor_stand_cv_decomp.glb")
    monitor_stand = builder.build_static(name='monitor')
    monitor_stand.set_pose(r2s(sapien.Pose(
        p=np.array([
            0.44275546192896376,
            1.3677609300247515,
            -0.6202413967664994
        ]),
        q=np.array([
            -4.371138828673793e-08,
            -1.7763568394002505e-15,
            1.0,
            -8.940696716308594e-08
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_box.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_box_cv_decomp.glb")
    box = builder.build(name='box')
    box.set_pose(r2s(sapien.Pose(
        p=np.array([
            4.180875379491428,
            1.7446024328401433,
            3.9581493301553787
        ]),
        q=np.array([
            0.7071492075920105,
            -0.009717144072055817,
            0.7069830298423767,
            -0.004533467348664999
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_box.glb")
    builder.add_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_box_cv_decomp.glb")
    box = builder.build(name='box')
    box.set_pose(r2s(sapien.Pose(
        p=np.array([
            0.5360783920138471,
            0.8969253691870651,
            5.64833215915369
        ]),
        q=np.array([
            0.9999961256980896,
            0.001716894330456853,
            3.311691034468822e-05,
            -0.002190836938098073
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
            4.19816869132963,
            1.4229390890756874,
            3.8625429417406565
        ]),
        q=np.array([
            0.0001892606378532946,
            -0.00021255071624182165,
            0.8249385952949524,
            0.5652222633361816
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
            4.197600338038609,
            1.4214713667526906,
            4.014418425396742
        ]),
        q=np.array([
            0.00022776523837819695,
            -0.0003542584308888763,
            0.717333197593689,
            0.6967301964759827
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
            4.197846850437962,
            1.3944192199414716,
            4.014852383860682
        ]),
        q=np.array([
            -0.0002748211263678968,
            0.00022432950208894908,
            0.7173177003860474,
            0.6967461705207825
        ])
    )))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        filename="replica_cad/objects/frl_apartment_tvstand.glb")
    builder.add_nonconvex_collision_from_file(
        filename="replica_cad/objects/convex/frl_apartment_tvstand_cv_decomp.glb")
    tvstand = builder.build_static(name='tv_stand')
    tvstand.set_pose(r2s(sapien.Pose(
        p=np.array([
            3.151346543163527,
            0.37619612738490105,
            7.716032639145851
        ]),
        q=np.array([
            1.0,
            0.0,
            0.0,
            0.0
        ])
    )))

    # Articulated objects
    loader = scene.create_urdf_loader()
    loader.fix_root_link = True
    fridge = loader.load("replica_cad/urdf/fridge/fridge_dynamic.urdf")
    fridge.set_root_pose(r2s(sapien.Pose(
        p=np.array([
            -2.1782121658325195,
            0.9755649566650391,
            3.2299728393554688
        ]),
        q=np.array([
            1,
            0,
            0,
            0
        ])
    )))

    loader = scene.create_urdf_loader()
    loader.fix_root_link = True
    loader.scale = 0.98
    kitchen_counter = loader.load(
        "replica_cad/urdf/kitchen_counter/kitchen_counter.urdf")
    kitchen_counter.set_root_pose(r2s(sapien.Pose(
        p=np.array([
            -2,
            0.04,
            1.2346
        ]),
        q=np.array([
            1,
            0,
            0,
            0
        ])
    )))

    loader = scene.create_urdf_loader()
    loader.fix_root_link = True
    loader.scale = 0.38
    kitchen_cupboard = loader.load(
        "replica_cad/urdf/kitchen_cupboards/kitchenCupboard_01.urdf")
    kitchen_cupboard.set_root_pose(r2s(sapien.Pose(
        p=np.array([
            -2.3,
            1.54,
            1.2346
        ]),
        q=np.array([
            -0.707,
            0.707,
            0,
            0
        ])
    )))

    loader = scene.create_urdf_loader()
    loader.fix_root_link = True
    loader.scale = 0.4
    chest_of_drawer = loader.load(
        "replica_cad/urdf/chest_of_drawers/chestOfDrawers_01_dynamic.urdf")
    chest_of_drawer.set_root_pose(r2s(sapien.Pose(
        p=np.array([
            -2.318626880645752,
            0,
            -1.3263527154922485
        ]),
        q=np.array([
            -0.707,
            0.707,
            0,
            0
        ])
    )))

    loader = scene.create_urdf_loader()
    loader.fix_root_link = True
    cabinet = loader.load("replica_cad/urdf/cabinet/cabinet_dynamic.urdf")
    cabinet.set_root_pose(r2s(sapien.Pose(
        p=np.array([
            1.317476749420166,
            0.05,
            1.8777843713760376
        ]),
        q=np.array([
            0.315322,
            0,
            0.948985,
            0
        ])
    )))

    loader = scene.create_urdf_loader()
    loader.fix_root_link = True
    door = loader.load("replica_cad/urdf/doors/door2.urdf")
    door.set_root_pose(r2s(sapien.Pose(
        p=np.array([
            -0.35,
            2.4,
            -2.65
        ]),
        q=np.array([
            1,
            0,
            0,
            0
        ])
    )))

    return scene


if __name__ == '__main__':
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

    # Viewer
    viewer = Viewer(renderer, resolutions=(1920, 1080))
    viewer.set_scene(scene)
    viewer.set_camera_xyz(x=-2.0, y=0, z=1.5)

    while not viewer.closed:
        scene.step()
        scene.update_render()
        viewer.render()
