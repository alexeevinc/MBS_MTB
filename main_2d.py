import sys
import os
import Adams
import math as m
import numpy as np
import pandas as pd
import toml

sys.path.append(os.path.abspath('C:/1_A/HSPF/repo_1'))

from src.utils.datamanager import load_config, load_plot_settings
from src.utils.custom_tools import rel_loc_custom, rel_orient_custom, loc_mirror_zx, ori_along_axis, ori_one_axis

config = load_config()

class MODEL():
    def __init__(self):
                ### STARTER PACK ###
                self.counter_solids = 0
                self.solids = []
                self.contacts_2d = []
                # model base
                Adams.defaults.units.setUnits(length='mm',mass='kg',time = 'second',angle='degrees',force='newton', frequency='hz')
                Adams.execute_cmd("default unit orient=body123") # changes the rotation sequence from default 313 to 123
                self.main_model = Adams.Models.create(name='CHAIN_TEST')
                self.main_model.size_of_icons = config.soi_model
                self.ground = self.main_model.ground_part
                gravity = self.main_model.Forces.createGravity(name='gravity', xyz_component_gravity = [0, 0, -9807])

                self.mar_ground = self.ground.Markers.create(name="MAR_GROUND")
                measure_cpu = self.main_model.Measures.createFunction(name= "CPU_MEA", function="CPU", units="no_units", create_measure_display="no")
                # Adams.read_command_file("src/modeling/cmd_files/callback.cmd")
                # solver settings
                s=Adams.defaults.model.settings.solver
                s.threads=4
                # s.integrator = "Newmark"

                self.df = pd.read_pickle('src/data/versuch25.pkl')

    def frame(self):
                self.body_frame = self.main_model.Parts.createRigidBody(name="body_frame")
                self.mar_frame_ctr = self.body_frame.Markers.create(name="MAR_FRAME_CENTER",
                        location=[0, 200, 0], orientation=[0,0,0])
                self.mar_frame_logger = self.body_frame.Markers.create(name="MAR_FRAME_LOGGER",
                        location=config.logger_loc, orientation=config.logger_ori)
                self.mar_logger_g = self.ground.Markers.create(name="MAR_FRAME_LOGGER_G",
                        location=config.logger_loc, orientation=config.logger_ori)
                self.mar_fr_axle_rot = self.ground.Markers.create(name="MAR_ORIGIN_ROT",
                        location=[0,371.85,706.35], orientation=[-56.606, 0, 0])
                self.mar_frame_origin = self.body_frame.Markers.create(name="MAR_FRAME_ORIGIN",
                        location=[0,0,0], orientation=[0,0,0])
                
                mar_frame_cm = self.body_frame.Markers.create(name="MAR_FRAME_CM", location = [0, -250, 0])
                self.body_frame.cm = mar_frame_cm
                self.body_frame.mass = 5
                self.body_frame.inertia_values = [50, 50.6, 50.6, 0,0,0] 

                frame2grnd = self.main_model.Constraints.createRevolute(name="FRAME_GRND_REV",
                    i_part=self.body_frame, j_part=self.ground, location=config.front_axle_loc, orientation=[0,90,0])
                
                gyro_y = self.df['GYRO_Y']*0.017 # deg to rad
                time = self.df['Time']
                t1 = 9191
                t2 = 3*1000
                gyro_y=gyro_y[t1:t1+t2]
                gyro_y.iloc[0]=0
                zeros = pd.Series([0] * 500)
                gyro_y = pd.concat([zeros, gyro_y], ignore_index=True)
                time=time[:t2+500]
                list4matrix = pd.concat([time, gyro_y], ignore_index=True)

                # matrix_input = self.main_model.DataElements.createMatrixFull(name = "MATRIX_INPUT",
                # values= list4matrix, input_order = "by_column", row_count= 15200, column_count= 2, units = "angular_acceleration")

                frame_logger_spline = self.main_model.DataElements.createSpline(name = "INPUT_SPLINE_FRAME",
                        x=time, y=gyro_y, x_units="time", y_units="angular_acceleration") 
                
                self.main_model.Constraints.createPointMotion(name='MOTION_LOGGER', i_marker=self.mar_frame_logger, j_marker=self.mar_logger_g, 
                        axis="b2", function="AKISPL(Time,0,INPUT_SPLINE_FRAME, 0)", time_derivative="velocity") 

    def rocker(self):
                self.body_rocker = self.main_model.Parts.createRigidBody(name="body_rocker")
                self.mar_rocker_ctr = self.body_rocker.Markers.create(name="MAR_ROCKER_CENTER",
                        location=config.rocker_mid_loc, orientation=[0,0,0])
                mar_rocker_cm = self.body_rocker.Markers.create(name="MAR_ROCKER_CM", location = [0, -250, 0])
                self.body_rocker.cm = mar_rocker_cm
                self.body_rocker.mass = 0.2
                self.body_rocker.inertia_values = [50, 50.6, 50.6, 0,0,0] 

                joint_rocker_sph = self.main_model.Constraints.createRevolute(name="ROCKER_REV",
                    i_part=self.body_rocker, j_part=self.body_frame, location=config.rocker_mid_loc, orientation=[0,90,0])
                # just visuslization 
                self.body_rocker.Geometries.createPolyline(name="ROCKER_LINE",
                        location=[config.rocker_rear_loc[0],config.rocker_rear_loc[1],config.rocker_rear_loc[2],
                                  config.rocker_front_loc[0],config.rocker_front_loc[1],config.rocker_front_loc[2],
                                  config.rocker_mid_loc[0], config.rocker_mid_loc[1], config.rocker_mid_loc[2],
                                  config.rocker_rear_loc[0],config.rocker_rear_loc[1],config.rocker_rear_loc[2]])

    def sus_link_lwr(self):
                self.body_sus_link_lwr = self.main_model.Parts.createRigidBody(name="body_sus_link_lwr")
                self.mar_sus_link_lwr_ctr = self.body_sus_link_lwr.Markers.create(name="MAR_LINK_LWR_CENTER",
                        location=config.sprocket_upr_loc, orientation=[0,0,0])
                mar_sus_link_lwr_cm = self.body_sus_link_lwr.Markers.create(name="MAR_ROCKER_CM", location = [0, -250, 0])
                self.body_sus_link_lwr.cm = mar_sus_link_lwr_cm
                self.body_sus_link_lwr.mass = 1
                self.body_sus_link_lwr.inertia_values = [50, 50.6, 50.6, 0,0,0] 

                joint_link_rev = self.main_model.Constraints.createRevolute(name="SUS_LINK_LWR_REV",
                    i_part=self.body_sus_link_lwr, j_part=self.body_frame, location=config.sprocket_upr_loc, orientation=[0,90,0])
                
                # just visuslization
                self.body_sus_link_lwr.Geometries.createPolyline(name="LINK_LWR_LINE",
                        location=[config.link_lwr_rear_loc[0],config.link_lwr_rear_loc[1],config.link_lwr_rear_loc[2],
                                  config.link_lwr_front_loc[0], config.link_lwr_front_loc[1]-50, config.link_lwr_front_loc[2]-50,
                                  config.link_lwr_front_loc[0], config.link_lwr_front_loc[1], config.link_lwr_front_loc[2]])

    def sus_link_upr(self):
                self.body_sus_link_upr = self.main_model.Parts.createRigidBody(name="body_sus_link_upr")
                self.mar_sus_link_upr_ctr = self.body_sus_link_upr.Markers.create(name="MAR_SUS_LINK_UPR_CENTER",
                        location=[0, 200, 0], orientation=[0,0,0])
                mar_rear_fr_cm = self.body_sus_link_upr.Markers.create(name="MAR_SUS_LINK_UPR_CM", location = [0, -250, 0])
                self.body_sus_link_upr.cm = mar_rear_fr_cm
                self.body_sus_link_upr.mass = 4
                self.body_sus_link_upr.inertia_values = [834.25, 417.6, 417.6, 0,0,0]

                joint_link2rocker_sph = self.main_model.Constraints.createSpherical(name="LINK2ROCKER_SPH",
                    i_part=self.body_sus_link_upr, j_part=self.body_rocker,
                    location=config.rocker_rear_loc, orientation=[0,90,0])
                
                joint_sus_links_rev = self.main_model.Constraints.createRevolute(name="SUS_LINKS_REV",
                    i_part=self.body_sus_link_upr, j_part=self.body_sus_link_lwr,
                    location=config.link_lwr_rear_loc, orientation=[0,90,0])
                
                # just visuslization
                self.body_sus_link_upr.Geometries.createPolyline(name="LINK_UPR_LINE",
                        location=[config.rocker_rear_loc[0],config.rocker_rear_loc[1],config.rocker_rear_loc[2],
                                  config.cassette_loc[0], config.cassette_loc[1], config.cassette_loc[2],
                                  config.link_lwr_rear_loc[0], config.link_lwr_rear_loc[1], config.link_lwr_rear_loc[2]])
                
    def damper_cylinder(self):
                self.body_damper_cyl = self.main_model.Parts.createRigidBody(name="body_damper_cyl")
                self.mar_damper_cyl_ctr = self.body_damper_cyl.Markers.create(name="MAR_DAMPER_CYL_CENTER",
                        location=config.rocker_front_loc, orientation=[0,0,0])
                
                self.mar_tr_ori_i = self.body_damper_cyl.Markers.create(name="MAR_TR_ORI_J",
                        location=config.rocker_front_loc, 
                        orientation=ori_one_axis(config.rocker_front_loc, config.damper_lwr_loc, "z"))

                self.mar_tr_ori_j2 = self.body_damper_cyl.Markers.create(name="MAR_TR_ORI_J2",
                        location=[0,0,50], orientation=[0,0,0], relative_to=self.mar_tr_ori_i )
                
                mar_damper_cyl_cm = self.body_damper_cyl.Markers.create(name="MAR_DAMPER_CYL_CM", location = [0, 0, 50], relative_to=self.mar_damper_cyl_ctr)
                self.body_damper_cyl.cm = mar_damper_cyl_cm
                self.body_damper_cyl.mass = 0.2
                self.body_damper_cyl.inertia_values = [50, 50.6, 50.6, 0,0,0]                
                joint_damper_upr_rev = self.main_model.Constraints.createSpherical(name="DAMPER_CYL_SPH",
                    i_part=self.body_damper_cyl, j_part=self.body_rocker, location=config.rocker_front_loc)
                                
                # just visuslization
                # self.body_damper_cyl.Geometries.createOutline(name="DAMPER_LINE1", marker =[self.mar_tr_ori_j, self.mar_tr_ori_j2], visibility_between_markers="on"
                #                                               )
                    
    def damper_piston(self):
                self.body_damper_pis = self.main_model.Parts.createRigidBody(name="body_damper_pis")
                self.mar_damper_pis_ctr = self.body_damper_pis.Markers.create(name="MAR_DAMPER_PIS_CENTER",
                        location=config.damper_lwr_loc)
                
                mar_damper_pis_cm = self.body_damper_pis.Markers.create(name="MAR_DAMPER_PIS_CM", location = [0, 0, -50], relative_to=self.mar_damper_pis_ctr)
                self.body_damper_pis.cm = mar_damper_pis_cm
                self.body_damper_pis.mass = 0.2
                self.body_damper_pis.inertia_values = [50, 50.6, 50.6, 0,0,0]     

                self.mar_tr_ori_j = self.body_damper_pis.Markers.create(name="MAR_TR_ORI_I",
                        location=config.damper_lwr_loc, 
                        orientation=ori_one_axis(self.mar_damper_cyl_ctr.location, self.mar_damper_pis_ctr.location, "z"))
                
                self.mar_tr_ori_i2 = self.body_damper_pis.Markers.create(name="MAR_TR_ORI_I2",
                        location=[0,0,-50], orientation=[0,0,0], relative_to=self.mar_tr_ori_i)
                
                joint_damper_lwr_sph = self.main_model.Constraints.createSpherical(name="DAMPER_PIS_SPH",
                    i_part=self.body_damper_pis, j_part=self.body_frame, location=config.damper_lwr_loc)

                joint_damper_tr = self.main_model.Constraints.createTranslational(name="DAMPER_TR",
                    i_part_name=self.body_damper_pis.name, j_part_name=self.body_damper_cyl.name,
                      location=self.mar_tr_ori_j2.location, orientation=self.mar_tr_ori_j2.orientation)
                
                sus_rear = self.df['SUS_rear']/-1
                time = self.df['Time']
                t1 = 9191
                t2 = 3*1000
                sus_rear=sus_rear[t1:t1+3*1000]
                sus_rear += abs(sus_rear.iloc[772])
                sus_rear[0:772]=0
                zeros = pd.Series([0] * 500)
                sus_rear = pd.concat([zeros, sus_rear], ignore_index=True)
                sus_rear_conv = np.convolve(sus_rear, np.ones(3) / 3, mode='same')
                sus_rear_conv_dt = np.diff(sus_rear_conv)
                sus_rear_conv_dt = np.append(sus_rear_conv_dt, 0)

                time=time[:t2+500]
                damper_spline = self.main_model.DataElements.createSpline(name = "INPUT_SPLINE_SUS",
                        x=time, y=sus_rear_conv_dt, x_units="time", y_units="velocity") 
                self.main_model.Constraints.createMotionT(name='DAMPER_MOTION', joint=joint_damper_tr, 
                        function="AKISPL(Time,0,INPUT_SPLINE_SUS, 0)", time_derivative="velocity") 

                
                # just visuslization
                # self.body_damper_pis.Geometries.createOutline(name="DAMPER_LINE2", marker =[self.mar_tr_ori_i, self.mar_tr_ori_i2])


    def sprocket_upr(self):
            self.solids_spr_upr = []
            ### SPROCKET ###
            self.body_sprocket = self.main_model.Parts.createRigidBody(name="body_sprocket_upr")
            self.mar_sprocket_ctr = self.body_sprocket.Markers.create(name="MAR_SPROCKET_CENTER",
                    location=config.sprocket_upr_loc, orientation=config.sprocket_upr_ori)
            mar_sprocket_cm = self.body_sprocket.Markers.create(name="MAR_SPROCKET_CM")
            mar_sprocket_cm.location = rel_loc_custom(self.mar_sprocket_ctr, [0, 0, 0])
            self.body_sprocket.cm = mar_sprocket_cm
            self.body_sprocket.mass = 0.040
            self.body_sprocket.inertia_values = [0,0,0, 0,0,0]

            joint_spr_upr_rev = self.main_model.Constraints.createRevolute(name="SPR_UPR_REV",
                    i_part=self.body_sprocket, j_part=self.ground,
                    location=[0,0,0], orientation=[0,90,0], relative_to=self.mar_sprocket_ctr)

        #     joint_motion_object = self.main_model.Constraints.createJointMotion(name="SPR_UPR_REV_MOTION",
        #             joint=joint_spr_upr_rev, function="5d*time")

            ##### MOTION
            # general point motion for sprocket:
            # this type of point motion exists only as UDE, therefore no pythonic method here
            # Adams.read_command_file("src/modeling/cmd_files/motion.cmd")
            #####

            # self.main_model.Constraints.createAtPoint
            parasolids_path = 'src/parasolids/sprocket_upper_segment.x_t'
            psmar_list = []

            # add parasolid teeth sections & rename and orient the autogenerated markers
            for i in range(1,17):
                    Adams.read_parasolid_file(file_name=parasolids_path, part_name="CHAIN_TEST.body_sprocket_upr")
                    marker = self.body_sprocket.Markers.__getitem__("PSMAR")
                    marker.name = "PSMAR_S" + str(i)
                    self.counter_solids += 1
                    solid = self.body_sprocket.Geometries.__getitem__("SOLID"+ str(self.counter_solids))
                    solid.name = "SOLID_SPROCKET_" + str(self.counter_solids)
                    solid.color_name = "white"
                    self.solids_spr_upr.append(solid)

                    if i == 1:
                            marker.location = rel_loc_custom(self.mar_sprocket_ctr, [0, 0, 0])
                            marker.orientation = rel_orient_custom(self.mar_sprocket_ctr, [0, 0, 0])
                    else:
                            idx = i - 2
                            marker.location = rel_loc_custom(psmar_list[idx], [0, 0, 0])
                            marker.orientation = rel_orient_custom(psmar_list[idx], [0, 22.5, 0])
                    psmar_list.append(marker)

            mar_curve_ctr = self.body_sprocket.Markers.create(name="MAR_CURVE_DATA",
            location = rel_loc_custom(self.mar_sprocket_ctr, [0, 0, 0]),
            orientation = rel_orient_custom(self.mar_sprocket_ctr, [-90, 90, 90]))

            sprocket_loc_arr = np.loadtxt('sprocket_profile_xy.txt', skiprows=2)
            sprocket_loc_ready = sprocket_loc_arr.flatten().tolist()

            matrix_sprocket = self.main_model.DataElements.createMatrixFull(name = "MATRIX_SPROCKET",
            values= sprocket_loc_ready, input_order = "by_row", row_count= 2945, column_count= 3,
            units = "length")

            sprocket_curve_data = self.main_model.DataElements.createCurveData(name = "DATA_CURVE_SPROCKET",
                            matrix_name = matrix_sprocket.name, fit_type = "curve_points")

        #     self.sprocket_bspline = self.body_sprocket.Geometries.createBSpline(name= "BSPLINE_SPROCKET",
        #                                             ref_curve_name=sprocket_curve_data.name,
        #                                             ref_marker_name=mar_curve_ctr.name,
        #                                             segment_count="100")


    def sprocket_upr_single(self):
            self.solids_spr_upr = []
            ### SPROCKET ###
            self.body_sprocket = self.main_model.Parts.createRigidBody(name="body_sprocket_upr")
            self.mar_sprocket_ctr = self.body_sprocket.Markers.create(name="MAR_SPROCKET_CENTER",
                    location=config.sprocket_upr_loc, orientation=config.sprocket_upr_ori)
            mar_sprocket_cm = self.body_sprocket.Markers.create(name="MAR_SPROCKET_CM")
            mar_sprocket_cm.location = rel_loc_custom(self.mar_sprocket_ctr, [0, 0, 0])
            self.body_sprocket.cm = mar_sprocket_cm
            self.body_sprocket.mass = 0.040
            self.body_sprocket.inertia_values = [834.25,417.6, 417.6, 0,0,0]

            joint_spr_upr_rev = self.main_model.Constraints.createRevolute(name="SPR_UPR_REV",
                    i_part=self.body_sprocket, j_part=self.body_frame,
                    location=[0,0,0], orientation=[0,90,0], relative_to=self.mar_sprocket_ctr)

        #     joint_motion_object = self.main_model.Constraints.createJointMotion(name="SPR_UPR_REV_MOTION",
        #             joint=joint_spr_upr_rev, function="5d*time")

            ##### MOTION
            # general point motion for sprocket:
            # this type of point motion exists only as UDE, therefore no pythonic method here
            # Adams.read_command_file("src/modeling/cmd_files/motion.cmd")
            #####

            # self.main_model.Constraints.createAtPoint
            parasolids_path = 'src/parasolids/su_reduced.x_t'

            Adams.read_parasolid_file(file_name=parasolids_path, part_name="CHAIN_TEST.body_sprocket_upr")
            marker = self.body_sprocket.Markers.__getitem__("PSMAR")
            marker.name = "PSMAR_S" + str(1)
            self.counter_solids += 1
            solid = self.body_sprocket.Geometries.__getitem__("SOLID"+ str(self.counter_solids))
            solid.name = "SOLID_SPROCKET_" + str(self.counter_solids)
            solid.color_name = "white"
            self.solids_spr_upr.append(solid)

            marker.location = rel_loc_custom(self.mar_sprocket_ctr, [0, 0, 0])
            marker.orientation = rel_orient_custom(self.mar_sprocket_ctr, [0, 0, 0])

        #     mar_curve_ctr = self.body_sprocket.Markers.create(name="MAR_CURVE_DATA",
        #     location = rel_loc_custom(self.mar_sprocket_ctr, [0, 0, 0]),
        #     orientation = rel_orient_custom(self.mar_sprocket_ctr, [-90, 90, 90]))

        #     sprocket_loc_arr = np.loadtxt('sprocket_profile_xy.txt', skiprows=2)
        #     sprocket_loc_ready = sprocket_loc_arr.flatten().tolist()

        #     matrix_sprocket = self.main_model.DataElements.createMatrixFull(name = "MATRIX_SPROCKET",
        #     values= sprocket_loc_ready, input_order = "by_row", row_count= 2945, column_count= 3,
        #     units = "length")

        #     sprocket_curve_data = self.main_model.DataElements.createCurveData(name = "DATA_CURVE_SPROCKET",
        #                     matrix_name = matrix_sprocket.name, fit_type = "curve_points")

        #     self.sprocket_bspline = self.body_sprocket.Geometries.createBSpline(name= "BSPLINE_SPROCKET",
        #                                             ref_curve_name=sprocket_curve_data.name,
        #                                             ref_marker_name=mar_curve_ctr.name,
        #                                             segment_count="100")


    def cassette(self):
            config = load_config()
            self.solids_cassette_teeth = []
            self.solids_cassette_convex = []
            ### CASSETTE ###
            self.body_cassette = self.main_model.Parts.createRigidBody(name="body_cassette")
            self.mar_cassette_ctr = self.body_cassette.Markers.create(name="MAR_CASSETTE_CENTER",
                    location=config.cassette_loc, orientation=config.cassette_ori)
            self.mar_cassette_ctr_axis = self.body_cassette.Markers.create(name="MAR_CASSETTE_CENTER_AXIS",
                    location=rel_loc_custom(self.mar_cassette_ctr, [10, 0, 0]))
            mar_cassette_cm = self.body_cassette.Markers.create(name="MAR_CASSETTE_CM",
                    location = rel_loc_custom(self.mar_cassette_ctr, [0, 0, 0]))
            self.body_cassette.cm = mar_cassette_cm
            self.body_cassette.mass = 0.9932
            self.body_cassette.inertia_values = [1704.5, 1697.8, 3123.8, 0,0,0]

            joint_cassette_rev = self.main_model.Constraints.createRevolute(name="JCK_CST_REV",
                    i_part=self.body_cassette, j_part=self.body_sus_link_upr,
                    location=[0,0,0], orientation=[0,90,0], relative_to=self.mar_cassette_ctr)

            parasolid_teeth_1 = 'src/parasolids/cas_reduced_4.x_t'

            psmar_list = []

            # add parasolid teeth sections & rename and orient the autogenerated markers
            Adams.read_parasolid_file(file_name=parasolid_teeth_1, part_name="CHAIN_TEST.body_cassette")
            self.counter_solids += 1
            solid = self.body_cassette.Geometries.__getitem__("SOLID"+ str(self.counter_solids))
            solid.name = "SOLID_CASSETTE_TOOTH1_" + str(self.counter_solids)
            self.solids_cassette_teeth.append(solid)

            marker = self.body_cassette.Markers.__getitem__("PSMAR")
            marker.name = "PSMAR_S_1_" + str(1)
            marker.location = rel_loc_custom(self.mar_cassette_ctr, [0, 0, 0])
            marker.orientation = rel_orient_custom(self.mar_cassette_ctr, [0, 0, 0])


    def sprocket_main(self):
            config = load_config()
            self.solids_main_teeth = []
            self.solids_main_convex_l = []
            self.solids_main_convex_r = []
            psmar_list = []
            parasolid_sprocket_main1 = 'src/parasolids/sprocket_main_1.x_t'
            parasolid_sprocket_main2 = 'src/parasolids/sprocket_main_2.x_t'
            parasolid_sprocket_main3 = 'src/parasolids/sprocket_main_conv.x_t'
            ### CASSETTE ###
            self.body_sprocket_main = self.main_model.Parts.createRigidBody(name="body_sprocket_main")
            self.mar_sprocket_main_ctr = self.body_sprocket_main.Markers.create(name="MAR_SPROCKET_MAIN_CENTER",
                    location=[config.sprocket_main_loc_x,config.sprocket_main_loc_y,config.sprocket_main_loc_z],
                    orientation=[config.sprocket_main_ori_x-10.5883, 0,config.sprocket_main_ori_z])
            mar_sprocket_main_cm = self.body_sprocket_main.Markers.create(name="MAR_SPROCKET_MAIN_CM",
                    location = rel_loc_custom(self.mar_sprocket_main_ctr, [0, 0, 0]))
            self.body_sprocket_main.cm = mar_sprocket_main_cm
            self.body_sprocket_main.mass = 0.2

            joint_spr_main_rev = self.main_model.Constraints.createFixed(name="SPR_M_REV",
                    i_part=self.body_sprocket_main, j_part=self.body_frame,
                    location=[0,0,0], orientation=[0,90,0], relative_to=self.mar_sprocket_main_ctr)

            # joint_motion_object = self.main_model.Constraints.createJointMotion(name="SPR_M_REV_MOTION",
            #         joint=joint_spr_main_rev, function="5d*time")

            for i in range(1,18):
                    Adams.read_parasolid_file(file_name=parasolid_sprocket_main1,
                            part_name="CHAIN_TEST.body_sprocket_main")
                    self.counter_solids += 1
                    solid = self.body_sprocket_main.Geometries.__getitem__("SOLID"+ str(self.counter_solids))
                    solid.name = "SOLID_MAIN_TOOTH_" + str(self.counter_solids)
                    self.solids_main_teeth.append(solid)
                    marker = self.body_sprocket_main.Markers.__getitem__("PSMAR")
                    marker.name = "PSMAR_SM_1_" + str(i)

                    if i==1:
                            marker.location = rel_loc_custom(self.mar_sprocket_main_ctr, [0, 0, 0])
                            marker.orientation = rel_orient_custom(self.mar_sprocket_main_ctr, [0, 0, 0])
                    else:
                            marker.location = rel_loc_custom(psmar_list[-1], [0, 0, 0])
                            marker.orientation = rel_orient_custom(psmar_list[-1], [0, 21.1765, 0])

                    psmar_list.append(marker)


                    Adams.read_parasolid_file(file_name=parasolid_sprocket_main3,
                            part_name="CHAIN_TEST.body_sprocket_main")
                    self.counter_solids += 1
                    solid = self.body_sprocket_main.Geometries.__getitem__("SOLID"+ str(self.counter_solids))
                    solid.name = "SOLID_MAIN_CONVEX_L" + str(self.counter_solids)
                    self.solids_main_convex_l.append(solid)
                    marker = self.body_sprocket_main.Markers.__getitem__("PSMAR")
                    marker.name = "PSMAR_SM_C1_" + str(i)
                    marker.location = rel_loc_custom(psmar_list[-1], [0, 0, 0])
                    marker.orientation = rel_orient_custom(psmar_list[-1], [0, 0, 0])

                    Adams.read_parasolid_file(file_name=parasolid_sprocket_main3,
                            part_name="CHAIN_TEST.body_sprocket_main")
                    self.counter_solids += 1
                    solid = self.body_sprocket_main.Geometries.__getitem__("SOLID"+ str(self.counter_solids))
                    solid.name = "SOLID_MAIN_CONVEX_R" + str(self.counter_solids)
                    self.solids_main_convex_r.append(solid)
                    marker = self.body_sprocket_main.Markers.__getitem__("PSMAR")
                    marker.name = "PSMAR_SM_C2_" + str(i)
                    marker.location = rel_loc_custom(psmar_list[-1], [0, 0, 0])
                    marker.orientation = rel_orient_custom(psmar_list[-1], [180, 0, 0])

                    Adams.read_parasolid_file(file_name=parasolid_sprocket_main2,
                            part_name="CHAIN_TEST.body_sprocket_main")
                    self.counter_solids += 1
                    solid = self.body_sprocket_main.Geometries.__getitem__("SOLID"+ str(self.counter_solids))
                    solid.name = "SOLID_MAIN_TOOTH_" + str(self.counter_solids)
                    self.solids_main_teeth.append(solid)
                    marker = self.body_sprocket_main.Markers.__getitem__("PSMAR")
                    marker.name = "PSMAR_SM_2_" + str(i)
                    marker.location = rel_loc_custom(psmar_list[-1], [0, 0, 0])
                    marker.orientation = rel_orient_custom(psmar_list[-1], [0, 0, 0])
                    psmar_list.append(marker)


    def sprocket_main_single(self):
            config = load_config()
            self.solids_main_teeth = []
            self.solids_main_convex_l = []
            self.solids_main_convex_r = []
            parasolid_sprocket_main1 = 'src/parasolids/sm_reduced.x_t'

            ### CASSETTE ###
            self.body_sprocket_main = self.main_model.Parts.createRigidBody(name="body_sprocket_main")
            self.mar_sprocket_main_ctr = self.body_sprocket_main.Markers.create(name="MAR_SPROCKET_MAIN_CENTER",
                    location=config.sprocket_main_loc,
                    orientation=[config.sprocket_main_ori_x, 0,config.sprocket_main_ori_z])
            mar_sprocket_main_cm = self.body_sprocket_main.Markers.create(name="MAR_SPROCKET_MAIN_CM",
                    location = rel_loc_custom(self.mar_sprocket_main_ctr, [0, 0, 0]))
            self.body_sprocket_main.cm = mar_sprocket_main_cm
            self.body_sprocket_main.mass = 0.2
            self.body_sprocket_main.inertia_values = [834.25,417.6, 417.6, 0,0,0]

            joint_spr_main_rev = self.main_model.Constraints.createFixed(name="SPR_M_REV",
                    i_part=self.body_sprocket_main, j_part=self.body_frame,
                    location=[0,0,0], orientation=[0,90,0], relative_to=self.mar_sprocket_main_ctr)

            # joint_motion_object = self.main_model.Constraints.createJointMotion(name="SPR_M_REV_MOTION",
            #         joint=joint_spr_main_rev, function="5d*time")

            Adams.read_parasolid_file(file_name=parasolid_sprocket_main1,
                    part_name="CHAIN_TEST.body_sprocket_main")
            self.counter_solids += 1
            solid = self.body_sprocket_main.Geometries.__getitem__("SOLID"+ str(self.counter_solids))
            solid.name = "SOLID_MAIN_TOOTH_" + str(self.counter_solids)
            self.solids_main_teeth.append(solid)
            marker = self.body_sprocket_main.Markers.__getitem__("PSMAR")
            marker.name = "PSMAR_SM_1_" + str(1)
            marker.location = rel_loc_custom(self.mar_sprocket_main_ctr, [0, 0, 0])
            marker.orientation = rel_orient_custom(self.mar_sprocket_main_ctr, [0, 0, 0])


    def derailleur_stat(self):
            config = load_config()
            ### DERAILLEUR STATIC PART ###
            self.body_der_stat = self.main_model.Parts.createRigidBody(name="body_der_stat")
            self.mar_der_stat_ctr = self.body_der_stat.Markers.create(name="MAR_DER_STAT_CENTER",
                    location=config.der_stat_loc, orientation=config.der_stat_ori)
            mar_der_stat_cm = self.body_der_stat.Markers.create(name="MAR_DER_STAT_CM",
                    relative_to=self.mar_der_stat_ctr, location = config.der_stat_cm, orientation=[0,0,0])
            self.body_der_stat.cm = mar_der_stat_cm
            self.body_der_stat.mass = 0.2058
            self.body_der_stat.inertia_values = config.der_stat_inertia
            joint_der_stat_fixed = self.main_model.Constraints.createFixed(name="DER_FIX",
                    i_part=self.body_der_stat, j_part=self.body_sus_link_upr,
                    location=[0,0,0], relative_to=self.mar_der_stat_ctr)


    def link_inr(self):
            config = load_config()
            ### INNER LINK ###
            self.body_link_inr = self.main_model.Parts.createRigidBody(name="body_link_inr")
            self.mar_link_inr_ctr = self.body_link_inr.Markers.create(name="MAR_LINK_INR_CENTER",
                    location=config.link_inr_loc, orientation=config.link_inr_ori)

            mar_link_inr_cm = self.body_link_inr.Markers.create(name="MAR_LINK_INR_CM",
                    relative_to=self.mar_link_inr_ctr, location = config.link_inr_cm, orientation= [0,0,0])
            self.body_link_inr.cm = mar_link_inr_cm
            self.body_link_inr.mass = 0.026
            self.body_link_inr.inertia_values = config.link_inr_inertia

            self.mar_link_inr_2kn = self.body_link_inr.Markers.create(name="MAR_LINK_INR_2KN",
                    location= [20.533, -405.09, 2.23], orientation=[-38.617, 7.658, 9.471])

            joint_link_inr_fixed = self.main_model.Constraints.createFixed(name="LINK_INR_FIX",
                    i_part=self.body_link_inr, j_part=self.body_sus_link_upr,
                    location=[0,0,0], relative_to=self.mar_link_inr_ctr)

            # just visualization with polylines
        #     der_line1 = self.body_der_stat.Geometries.createPolyline(name= "DER_POL1",
        #             location=[self.mar_der_stat_ctr.location[0], self.mar_der_stat_ctr.location[1], self.mar_der_stat_ctr.location[2],
        #                         self.mar_link_inr_ctr.location[0], self.mar_link_inr_ctr.location[1], self.mar_link_inr_ctr.location[2]])
        #     link_inr_line = self.body_der_stat.Geometries.createPolyline(name= "LINK_INR_LINE",
        #             location=[self.mar_link_inr_2kn.location[0], self.mar_link_inr_2kn.location[1], self.mar_link_inr_2kn.location[2],
        #                         self.mar_link_inr_ctr.location[0], self.mar_link_inr_ctr.location[1], self.mar_link_inr_ctr.location[2]])


    def fbar_calc(self):
            config = load_config()
            # calculation of four-bar linkage parameters
            BD = m.sqrt(config.fbar_AB**2 + config.fbar_AD**2 - 2*config.fbar_AB*config.fbar_AD*m.cos(m.radians(config.fbar_input_init+config.fbar_input)))
            ADB = m.acos((config.fbar_AD**2+BD**2-config.fbar_AB**2)/(2*config.fbar_AD*BD))
            BDC = m.acos((config.fbar_CD**2+BD**2-config.fbar_BC**2)/(2*config.fbar_CD*BD))
            self.ADC = m.degrees(ADB+BDC)
            ABD = m.radians(180-config.fbar_input_init-config.fbar_input)-ADB
            DBC = m.acos((config.fbar_BC**2+BD**2-config.fbar_CD**2)/(2*config.fbar_BC*BD))
            self.ABC = m.degrees(ABD+DBC)


    def link_out(self):
            config = load_config()
            ### OUTER LINK ###
            self.body_link_out = self.main_model.Parts.createRigidBody(name="body_link_out")
            self.mar_link_out_ctr = self.body_link_out.Markers.create(name="MAR_LINK_OUT_CENTER",
                    location=config.link_out_loc, orientation=config.link_out_ori)
            mar_link_out_cm=self.body_link_out.Markers.create(name="MAR_LINK_OUT_CM",
                    relative_to=self.mar_link_out_ctr, location=config.link_out_cm, orientation=[0,0,0])
            self.body_link_out.cm = mar_link_out_cm
            self.body_link_out.mass = 0.0334
            self.body_link_out.inertia_values = config.link_out_inertia

            self.mar_link_out_2kn = self.body_link_out.Markers.create(name="MAR_LINK_OUT_2KN",
                    location=[0, 48,0], orientation=[0,0,0], relative_to=self.mar_link_out_ctr)

            joint_link_out_fixed = self.main_model.Constraints.createFixed(name="LINK_OUT_FIX",
                    i_part=self.body_link_out, j_part=self.body_sus_link_upr,
                    location=[0,0,0], relative_to=self.mar_link_out_ctr)

        #     der_line2 = self.body_der_stat.Geometries.createPolyline(name= "DER_POL2",
        #             location=[self.mar_der_stat_ctr.location[0], self.mar_der_stat_ctr.location[1], self.mar_der_stat_ctr.location[2],
        #                         self.mar_link_out_ctr.location[0], self.mar_link_out_ctr.location[1], self.mar_link_out_ctr.location[2]])
        #     link_out_line = self.body_der_stat.Geometries.createPolyline(name= "LINK_OUT_LINE",
        #             location=[self.mar_link_out_2kn.location[0], self.mar_link_out_2kn.location[1], self.mar_link_out_2kn.location[2],
        #                         self.mar_link_out_ctr.location[0], self.mar_link_out_ctr.location[1], self.mar_link_out_ctr.location[2]])


    def knuckle(self):
            config = load_config()
            ### KNUCKLE ###
            self.body_knuckle = self.main_model.Parts.createRigidBody(name="body_knuckle")
            # self.mar_fbar_coupling = self.body_knuckle.Markers.create(name="MAR_FBAR_COUPLING",
            #         location=rel_loc_custom(self.mar_link_inr_2kn, [0, 0, 0]),
            #         orientation=rel_orient_custom(self.mar_link_inr_2kn,[self.ABC - config.fbar_output_init,0,0]))

            self.mar_knuckle_ctr = self.body_knuckle.Markers.create(name="MAR_KNUCKLE_CENTER",
                    location=config.knuckle_loc, orientation=config.knuckle_ori)
            
            self.mar_sd_knuckle = self.body_knuckle.Markers.create(name="MAR_KNUCKLE_SD",
                    location=[0,0,0], orientation=[0,90,0], relative_to=self.mar_knuckle_ctr)
            
            self.mar_knuckle_2_cage = self.body_knuckle.Markers.create(name="MAR_KNUCKLE_2_CAGE",
                    location=config.knuckle_loc, orientation=rel_orient_custom(self.mar_knuckle_ctr, [0, -10, 0]))

            mar_knuckle_cm=self.body_knuckle.Markers.create(name="MAR_KNUCKLE_CM",
                    relative_to=self.mar_knuckle_ctr, location=config.knuckle_cm, orientation=[0,0,0])
            self.body_knuckle.cm = mar_knuckle_cm
            self.body_knuckle.mass = 0.0316
            self.body_knuckle.inertia_values = config.knuckle_inertia

            joint_knuckle_fixed = self.main_model.Constraints.createFixed(name="KN_FIX",
                    i_part=self.body_knuckle, j_part=self.body_der_stat, location=[0,0,0], relative_to=mar_knuckle_cm)

        #     knuckle_line = self.body_knuckle.Geometries.createPolyline(name= "KNUCKLE_LINE",
        #             location=[self.mar_knuckle_ctr.location[0], self.mar_knuckle_ctr.location[1], self.mar_knuckle_ctr.location[2],
        #                         self.mar_link_out_2kn.location[0], self.mar_link_out_2kn.location[1], self.mar_link_out_2kn.location[2],
        #                         self.mar_link_inr_2kn.location[0], self.mar_link_inr_2kn.location[1], self.mar_link_inr_2kn.location[2],
        #                         self.mar_knuckle_ctr.location[0], self.mar_knuckle_ctr.location[1], self.mar_knuckle_ctr.location[2]])


    def der_cage(self):
            config = load_config()
            ### DERAILLEUR CAGE ###
            self.body_der_cage = self.main_model.Parts.createRigidBody(name="body_der_cage")
            self.mar_der_cage_ctr = self.body_der_cage.Markers.create(name="MAR_DER_CAGE_CENTER",
                    location=config.der_cage_loc, orientation=config.der_cage_ori, relative_to=self.mar_knuckle_ctr)
            mar_der_cage_cm=self.body_der_cage.Markers.create(name="MAR_DER_CAGE_CM",
                    relative_to=self.mar_der_cage_ctr, location=config.der_cage_cm, orientation=[0,0,0])
            self.body_der_cage.cm = mar_der_cage_cm
            self.body_der_cage.mass = 0.1577
            self.body_der_cage.inertia_values = config.der_cage_inertia

            joint_der_cage_rev = self.main_model.Constraints.createRevolute(name="DER_CAGE_REV",
                    i_part=self.body_der_cage, j_part=self.body_knuckle,
                    location=[0,0,0], orientation=[0,90,0], relative_to=self.mar_der_cage_ctr)

            statevariable = self.main_model.DataElements.createStateVariable(name="VARIABLE_1", function="AX(MAR_DER_CAGE_CENTER, MAR_KNUCKLE_2_CAGE)*RTOD")
            # diffequation = self.main_model.SystemElements.createDifferentialEquation(name ="DIFF_1", function="-1*(DIF(.CHAIN_TEST.DIFF_1)*RTOD-1)", static_hold="off")
        #     joint_der_cage_rev_motion = self.main_model.Constraints.createJointMotion(name="DER_CAGE_MOTION", joint=joint_der_cage_rev, time_derivative="displacement", 
        #                             # function="-100*( AX(MAR_DER_CAGE_CENTER,MAR_KNUCKLE_CENTER)*RTOD-(0-DIF(.CHAIN_TEST.DIFF_1))) * time")  
        #                             function="-0.1*VARVAL(VARIABLE_1) * time") 
            mar_sd_cage = self.body_der_cage.Markers.create(name="MAR_DER_CAGE_SD",
                    location=[0,0,0], orientation=[0,90,0], relative_to=self.mar_der_cage_ctr)
            sd = self.main_model.Forces.createRotationalSpringDamper(name='RSD_CAGE', i_marker=self.mar_sd_knuckle, j_marker=mar_sd_cage, 
                                                                r_stiff=10, r_damp=10, torque_preload=5000, displacement_at_preload=40.0)
            # general motion function for Y
            # -100*( DY(MAR_LINK_OUT_CTR_10,MAR_Y_TARGET,MAR_Y_TARGET)-(0-DIF(.CHAIN_SIMPLE.DIFF_1))) * time         


    def jockey_upr(self):
            config = load_config()
            self.solids_jockey_upr_teeth = []
            self.solids_jockey_upr_convex_l = []
            self.solids_jockey_upr_convex_r = []
            psmar_list = []
            parasolid_jockey_upr_tooth1 = 'src/parasolids/ju_reduced.x_t'

            ### JOCKEY UPPER ###
            self.body_jockey_upr = self.main_model.Parts.createRigidBody(name="body_jockey_upr")
            self.mar_jockey_upr_ctr_gl = self.body_jockey_upr.Markers.create(name="MAR_JOCKEY_UPR_CENTER_GL",
                    location=config.jockey_upr_loc_gl, orientation=config.jockey_upr_ori_gl)

            self.mar_jockey_upr_ctr_lcl = self.body_jockey_upr.Markers.create(name="MAR_JOCKEY_UPR_CENTER_LCL",
                    location=rel_loc_custom(self.mar_jockey_upr_ctr_gl, config.jockey_upr_loc_lcl),
                    orientation=rel_orient_custom(self.mar_jockey_upr_ctr_gl, config.jockey_upr_ori_lcl))

            self.mar_jockey_upr_ctr_axis = self.body_jockey_upr.Markers.create(name="MAR_JOCKEY_UPR_CENTER_AXIS",
                    location=rel_loc_custom(self.mar_jockey_upr_ctr_lcl, [10, 0, 0]))

            mar_jockey_upr_cm = self.body_jockey_upr.Markers.create(name="MAR_JOCKEY_UPR_CM",
                    relative_to=self.mar_jockey_upr_ctr_lcl, location = config.jockey_upr_cm, orientation=[0,0,0])
            self.body_jockey_upr.cm = mar_jockey_upr_cm
            self.body_jockey_upr.mass = 0.0131
            self.body_jockey_upr.inertia_values = config.jockey_upr_inertia

            joint_jockey_upr_rev = self.main_model.Constraints.createRevolute(name="JCK_UPR_REV",
                    i_part=self.body_jockey_upr, j_part=self.body_der_cage,
                    location=[0,0,0], orientation=[0,90,0], relative_to=self.mar_jockey_upr_ctr_lcl)

            Adams.read_parasolid_file(file_name=parasolid_jockey_upr_tooth1,
                    part_name="CHAIN_TEST.body_jockey_upr")
            self.counter_solids += 1
            solid = self.body_jockey_upr.Geometries.__getitem__("SOLID"+ str(self.counter_solids))
            solid.name = "SOLID_JU_T1_" + str(self.counter_solids)
            self.solids_jockey_upr_teeth.append(solid)
            marker = self.body_jockey_upr.Markers.__getitem__("PSMAR")
            marker.name = "PSMAR_JU_T1_" + str(1)
            marker.location = rel_loc_custom(self.mar_jockey_upr_ctr_lcl, [0, 0, 0])
            marker.orientation = rel_orient_custom(self.mar_jockey_upr_ctr_lcl, [0, 0, 0])
            psmar_list.append(marker)


    def jockey_lwr(self):
            config = load_config()
            self.solids_jockey_lwr_teeth = []
            self.solids_jockey_lwr_convex_l = []
            self.solids_jockey_lwr_convex_r = []
            psmar_list = []
            parasolid_jockey_lwr_tooth1 = 'src/parasolids/jl_reduced.x_t'


            ### JOCKEY LOWER ###
            self.body_jockey_lwr = self.main_model.Parts.createRigidBody(name="body_jockey_lwr")
            self.mar_jockey_lwr_ctr_gl = self.body_jockey_lwr.Markers.create(name="MAR_JOCKEY_LWR_CENTER_GL",
                    location=config.jockey_lwr_loc_gl, orientation=config.jockey_lwr_ori_gl)

            self.mar_jockey_lwr_ctr_axis = self.body_jockey_lwr.Markers.create(name="MAR_JOCKEY_LWR_CENTER_AXIS",
                    location=rel_loc_custom(self.mar_jockey_lwr_ctr_gl, [10,0,0]),
                    orientation=rel_orient_custom(self.mar_jockey_lwr_ctr_gl, [0,0,0]))

            self.mar_jockey_lwr_ctr_loc = self.body_jockey_lwr.Markers.create(name="MAR_JOCKEY_LWR_CENTER_LCL",
                    location=rel_loc_custom(self.mar_jockey_lwr_ctr_gl, config.jockey_lwr_loc_lcl),
                    orientation=rel_orient_custom(self.mar_jockey_lwr_ctr_gl, config.jockey_lwr_ori_lcl))

            mar_jockey_lwr_cm = self.body_jockey_lwr.Markers.create(name="MAR_JOCKEY_LWR_CM",
                    relative_to=self.mar_jockey_lwr_ctr_loc, location = [0,0,0], orientation=[0,0,0])
            self.body_jockey_lwr.cm = mar_jockey_lwr_cm
            self.body_jockey_lwr.mass = 0.0102
            self.body_jockey_lwr.inertia_values = config.jockey_lwr_inertia

            joint_jockey_lwr_rev = self.main_model.Constraints.createRevolute(name="JCK_LWR_REV",
                    i_part=self.body_jockey_lwr, j_part=self.body_der_cage,
                    location=[0,0,0], orientation=[0,90,0], relative_to=self.mar_jockey_lwr_ctr_loc)

            Adams.read_parasolid_file(file_name=parasolid_jockey_lwr_tooth1,
                    part_name="CHAIN_TEST.body_jockey_lwr")
            self.counter_solids += 1
            solid = self.body_jockey_lwr.Geometries.__getitem__("SOLID"+ str(self.counter_solids))
            solid.name = "SOLID_JL_T1_" + str(self.counter_solids)
            self.solids_jockey_lwr_teeth.append(solid)
            marker = self.body_jockey_lwr.Markers.__getitem__("PSMAR")
            marker.name = "PSMAR_JL_T1_" + str(1)

            marker.location = rel_loc_custom(self.mar_jockey_lwr_ctr_loc, [0, 0, 0])
            marker.orientation = rel_orient_custom(self.mar_jockey_lwr_ctr_loc, [0, 0, 0])

            cage_line = self.body_der_cage.Geometries.createPolyline(name= "CAGE_LINE",
            location=[self.mar_knuckle_ctr.location[0],self.mar_knuckle_ctr.location[1],self.mar_knuckle_ctr.location[2],
                    self.mar_jockey_lwr_ctr_loc.location[0], self.mar_jockey_lwr_ctr_loc.location[1], self.mar_jockey_lwr_ctr_loc.location[2],
                    self.mar_jockey_upr_ctr_lcl.location[0], self.mar_jockey_upr_ctr_lcl.location[1], self.mar_jockey_upr_ctr_lcl.location[2],
                    self.mar_knuckle_ctr.location[0],self.mar_knuckle_ctr.location[1],self.mar_knuckle_ctr.location[2]])

            # print([s.name for s in self.solids_jockey_lwr_teeth])
            # print([s.name for s in self.solids_jockey_lwr_convex_l])
            # print([s.name for s in self.solids_jockey_lwr_convex_r])


    def chain_2d(self):
                config = load_config()

                def roller_circle_simple(marker, i):
                        self.ground.Geometries.createCircle(name="CIRCLE_"+ str(i),
                                center_marker=marker, radius=3.965)
                                        
                pins_y = []
                pins_z = []

                with open('src/data/pins_xy.txt', 'r') as f:
                        for line in f:
                                x, y = line.strip().split('\t')
                                pins_y.append(float(x))
                                pins_z.append(float(y))

                self.var_contact_k = self.main_model.DesignVariables.createReal(name = "tab_cont_k",
                        value = 100, units="no_units", delta_type="relative", range=[0,0], use_allowed_values = "no")

                self.var_contact_e = self.main_model.DesignVariables.createReal(name = "tab_cont_e",
                        value = 1.2, units="no_units", delta_type="relative", range=[0,0], use_allowed_values = "no")

                self.var_contact_c = self.main_model.DesignVariables.createReal(name = "tab_cont_c",
                        value = 50, units="no_units", delta_type="relative", range=[0,0], use_allowed_values = "no")

                self.var_contact_d = self.main_model.DesignVariables.createReal(name = "tab_cont_d",
                        value = 0.05, units="no_units", delta_type="relative", range=[0,0], use_allowed_values = "no")

                self.bodies_inner_links = []
                markers_links_in_ctr = []
                markers_links_in_end = []

                self.bodies_outer_links = []
                markers_links_out_ctr = []
                markers_links_out_end = []

                self.markers_link_cm = []

                self.contact_group = []
                self.measure_id_count = 0
                self.sensor_id_count = 0
                self.contacts_count = 0

                def roller_circle(body_link_in, marker_link_ctr, marker_link_end, i):
                        mar_roller_circle_ctr = self.bodies_inner_links[-1].Markers.create(name="MAR_ROLLER_CTR_"+ str(i),
                                location = rel_loc_custom(marker_link_ctr, [0,0,0]),
                                orientation=rel_orient_custom(marker_link_ctr, [-90, 90, 90]),
                                size_of_icons=config.soi_arcs_small)

                        mar_roller_cyl_side1 = self.bodies_inner_links[-1].Markers.create(name="MAR_ROLLER_CYL_SIDE1_"+ str(i),
                                location=rel_loc_custom(mar_roller_circle_ctr, [0,0,-1*(config.roll_cyl_len/2)]),
                                orientation=rel_orient_custom(mar_roller_circle_ctr, [0,0,0]),size_of_icons=config.soi_arcs_small)

                        roller_cylinder1 = self.bodies_inner_links[-1].Geometries.createCylinder(name="ROLLER_CYL1_"+ str(i),
                                center_marker = mar_roller_cyl_side1, length=config.roll_cyl_len, radius=config.roll_radius)

                        mar_roller_circle_end = self.bodies_inner_links[-1].Markers.create(name="MAR_ROLLER_END_"+ str(i),
                                location = rel_loc_custom(marker_link_end, [0,0,0]),
                                orientation=rel_orient_custom(marker_link_end, [-90, 90, 90]),
                                size_of_icons=config.soi_arcs_small)

                        mar_roller_cyl_side2 = self.bodies_inner_links[-1].Markers.create(name="MAR_ROLLER_CYL_SIDE2_"+ str(i),
                                location=rel_loc_custom(mar_roller_circle_end, [0,0,-1*(config.roll_cyl_len/2)]),
                                orientation=rel_orient_custom(mar_roller_circle_end, [0,0,0]),size_of_icons=config.soi_arcs_small)

                        roller_cylinder2 = self.bodies_inner_links[-1].Geometries.createCylinder(name="ROLLER_CYL2_"+ str(i),
                                center_marker = mar_roller_cyl_side2, length=config.roll_cyl_len, radius=config.roll_radius)

                        for s in self.solids_spr_upr:  # upper sprocket teeth
                                self.contacts_count+=1
                                contact2sprocket_upr=self.main_model.Contacts.createSolidToSolid(
                                        name=str(roller_cylinder2.name)+"_SPRU"+str(self.solids_spr_upr.index(s)), adams_id=self.contacts_count,#+"_"+str(i),
                                        i_geometry = s, j_geometry = roller_cylinder2,
                                        stiffness=self.var_contact_k.value[0], damping=self.var_contact_c.value[0],
                                        dmax=self.var_contact_d.value[0], exponent=self.var_contact_e.value[0], no_friction=True)
                                        # coulomb_friction="on", mu_dynamic=config.sph_mu_dyn, mu_static=config.sph_mu_st,
                                        # stiction_transition_velocity=config.sph_mu_stv, friction_transition_velocity=config.sph_mu_tvc,
                                        # max_stiction_deformation=config.sph_mu_max_st_def)
                                self.contact_group.append(contact2sprocket_upr)

                        for s in self.solids_spr_upr:  # upper sprocket teeth
                                self.contacts_count+=1
                                contact2sprocket_upr=self.main_model.Contacts.createSolidToSolid(
                                        name=str(roller_cylinder1.name)+"_SPRU"+str(self.solids_spr_upr.index(s)), adams_id=self.contacts_count,#+"_"+str(i),
                                        i_geometry = s, j_geometry = roller_cylinder1,
                                        stiffness=self.var_contact_k.value[0], damping=self.var_contact_c.value[0],
                                        dmax=self.var_contact_d.value[0], exponent=self.var_contact_e.value[0], no_friction=True)
                                        # coulomb_friction="on", mu_dynamic=config.sph_mu_dyn, mu_static=config.sph_mu_st,
                                        # stiction_transition_velocity=config.sph_mu_stv, friction_transition_velocity=config.sph_mu_tvc,
                                        # max_stiction_deformation=config.sph_mu_max_st_def)
                                self.contact_group.append(contact2sprocket_upr)

                        for s in self.solids_main_teeth: # main sprocket teeth
                                self.contacts_count+=1
                                contact2sprocket_main_t=self.main_model.Contacts.createSolidToSolid(
                                        name=str(roller_cylinder2.name)+"_SPRM_T"+str(self.solids_main_teeth.index(s)),adams_id=self.contacts_count,#+"_"+str(i),
                                        i_geometry = s, j_geometry = roller_cylinder2,
                                        stiffness=self.var_contact_k.value[0], damping=self.var_contact_c.value[0],
                                        dmax=self.var_contact_d.value[0], exponent=self.var_contact_e.value[0], no_friction=True)
                                        # coulomb_friction="on", mu_dynamic=config.sph_mu_dyn, mu_static=config.sph_mu_st,
                                        # stiction_transition_velocity=config.sph_mu_stv, friction_transition_velocity=config.sph_mu_tvc,
                                        # max_stiction_deformation=config.sph_mu_max_st_def)
                                self.contact_group.append(contact2sprocket_main_t)

                        for s in self.solids_main_teeth: # main sprocket teeth
                                self.contacts_count+=1
                                contact2sprocket_main_t=self.main_model.Contacts.createSolidToSolid(
                                        name=str(roller_cylinder1.name)+"_SPRM_T"+str(self.solids_main_teeth.index(s)),adams_id=self.contacts_count,#+"_"+str(i),
                                        i_geometry = s, j_geometry = roller_cylinder1,
                                        stiffness=self.var_contact_k.value[0], damping=self.var_contact_c.value[0],
                                        dmax=self.var_contact_d.value[0], exponent=self.var_contact_e.value[0], no_friction=True)
                                        # coulomb_friction="on", mu_dynamic=config.sph_mu_dyn, mu_static=config.sph_mu_st,
                                        # stiction_transition_velocity=config.sph_mu_stv, friction_transition_velocity=config.sph_mu_tvc,
                                        # max_stiction_deformation=config.sph_mu_max_st_def)
                                self.contact_group.append(contact2sprocket_main_t)

                        for s in self.solids_jockey_lwr_teeth: # jockey lower teeth
                                contact2jockey_lwr_t=self.main_model.Contacts.createSolidToSolid(
                                        name=str(roller_cylinder2.name)+"_JL_T"+str(self.solids_jockey_lwr_teeth.index(s)),#+"_"+str(i),
                                        i_geometry = s, j_geometry = roller_cylinder2,
                                        stiffness=self.var_contact_k.value[0], damping=self.var_contact_c.value[0],
                                        dmax=self.var_contact_d.value[0], exponent=self.var_contact_e.value[0], no_friction=True)
                                        # coulomb_friction="on", mu_dynamic=config.sph_mu_dyn, mu_static=config.sph_mu_st,
                                        # stiction_transition_velocity=config.sph_mu_stv, friction_transition_velocity=config.sph_mu_tvc,
                                        # max_stiction_deformation=config.sph_mu_max_st_def)
                                self.contact_group.append(contact2jockey_lwr_t)


                        for s in self.solids_jockey_lwr_teeth: # jockey lower teeth
                                contact2jockey_lwr_t=self.main_model.Contacts.createSolidToSolid(
                                        name=str(roller_cylinder1.name)+"_JL_T"+str(self.solids_jockey_lwr_teeth.index(s)),#+"_"+str(i),
                                        i_geometry = s, j_geometry = roller_cylinder1,
                                        stiffness=self.var_contact_k.value[0], damping=self.var_contact_c.value[0],
                                        dmax=self.var_contact_d.value[0], exponent=self.var_contact_e.value[0], no_friction=True)
                                        # coulomb_friction="on", mu_dynamic=config.sph_mu_dyn, mu_static=config.sph_mu_st,
                                        # stiction_transition_velocity=config.sph_mu_stv, friction_transition_velocity=config.sph_mu_tvc,
                                        # max_stiction_deformation=config.sph_mu_max_st_def)
                                self.contact_group.append(contact2jockey_lwr_t)

                        for s in self.solids_jockey_upr_teeth: # jockey upper teeth
                                contact2jockey_upr_t=self.main_model.Contacts.createSolidToSolid(
                                        name=str(roller_cylinder2.name)+"_JU_T"+str(self.solids_jockey_upr_teeth.index(s)),#+"_"+str(i),
                                        i_geometry = s, j_geometry = roller_cylinder2,
                                        stiffness=self.var_contact_k.value[0], damping=self.var_contact_c.value[0],
                                        dmax=self.var_contact_d.value[0], exponent=self.var_contact_e.value[0], no_friction=True)
                                        # coulomb_friction="on", mu_dynamic=config.sph_mu_dyn, mu_static=config.sph_mu_st,
                                        # stiction_transition_velocity=config.sph_mu_stv, friction_transition_velocity=config.sph_mu_tvc,
                                        # max_stiction_deformation=config.sph_mu_max_st_def)
                                self.contact_group.append(contact2jockey_upr_t)

                        for s in self.solids_jockey_upr_teeth: # jockey upper teeth
                                contact2jockey_upr_t=self.main_model.Contacts.createSolidToSolid(
                                        name=str(roller_cylinder1.name)+"_JU_T"+str(self.solids_jockey_upr_teeth.index(s)),#+"_"+str(i),
                                        i_geometry = s, j_geometry = roller_cylinder1,
                                        stiffness=self.var_contact_k.value[0], damping=self.var_contact_c.value[0],
                                        dmax=self.var_contact_d.value[0], exponent=self.var_contact_e.value[0], no_friction=True)
                                        # coulomb_friction="on", mu_dynamic=config.sph_mu_dyn, mu_static=config.sph_mu_st,
                                        # stiction_transition_velocity=config.sph_mu_stv, friction_transition_velocity=config.sph_mu_tvc,
                                        # max_stiction_deformation=config.sph_mu_max_st_def)
                                self.contact_group.append(contact2jockey_upr_t)

                        for s in self.solids_cassette_teeth: # cassette teeth
                                contact2cassette_t=self.main_model.Contacts.createSolidToSolid(
                                        name=str(roller_cylinder2.name)+"_CAS_T"+str(self.solids_cassette_teeth.index(s)),#+"_"+str(i),
                                        i_geometry = s, j_geometry = roller_cylinder2,
                                        stiffness=self.var_contact_k.value[0], damping=self.var_contact_c.value[0],
                                        dmax=self.var_contact_d.value[0], exponent=self.var_contact_e.value[0], no_friction=True)
                                        # coulomb_friction="on", mu_dynamic=config.sph_mu_dyn, mu_static=config.sph_mu_st,
                                        # stiction_transition_velocity=config.sph_mu_stv, friction_transition_velocity=config.sph_mu_tvc,
                                        # max_stiction_deformation=config.sph_mu_max_st_def)
                                self.contact_group.append(contact2cassette_t)

                        for s in self.solids_cassette_teeth: # cassette teeth
                                contact2cassette_t=self.main_model.Contacts.createSolidToSolid(
                                        name=str(roller_cylinder1.name)+"_CAS_T"+str(self.solids_cassette_teeth.index(s)),#+"_"+str(i),
                                        i_geometry = s, j_geometry = roller_cylinder1,
                                        stiffness=self.var_contact_k.value[0], damping=self.var_contact_c.value[0],
                                        dmax=self.var_contact_d.value[0], exponent=self.var_contact_e.value[0], no_friction=True)
                                        # coulomb_friction="on", mu_dynamic=config.sph_mu_dyn, mu_static=config.sph_mu_st,
                                        # stiction_transition_velocity=config.sph_mu_stv, friction_transition_velocity=config.sph_mu_tvc,
                                        # max_stiction_deformation=config.sph_mu_max_st_def)
                                self.contact_group.append(contact2cassette_t)

                      
                def links_cm(body_link, marker_link_ctr):
                        marker_link_cm = body_link.Markers.create(name="MAR_CM_"+body_link.name,
                                location = rel_loc_custom(marker_link_ctr, [0, config.rollers_dist/2, 0]),
                                orientation=rel_orient_custom(marker_link_ctr, [0, 0, 0]),
                                size_of_icons=config.soi_arcs_small)

                        self.markers_link_cm.append(marker_link_cm)

                        body_link.cm = marker_link_cm
                        body_link.mass = 0.005
                        body_link.inertia_values = [0.09, 0.142, 0.08, 0,0,0]

                def tab_contacts(side, body_link, tab_solid):
                        # contacts for inner links
                        if '_in' in body_link.name:
                                for s in self.solids_spr_upr:  # upper sprocket teeth
                                        self.contacts_count+=1
                                        contact2sprocket_upr=self.main_model.Contacts.createSolidToSolid(
                                                name=str(tab_solid.name)+"_SPRU"+str(self.solids_spr_upr.index(s)),adams_id=self.contacts_count,#+"_"+str(i),
                                                i_geometry = s, j_geometry = tab_solid,
                                                stiffness=self.var_contact_k.value[0], damping=self.var_contact_c.value[0],
                                                dmax=self.var_contact_d.value[0], exponent=self.var_contact_e.value[0], no_friction=True)
                                                # coulomb_friction="on", mu_dynamic=config.sph_mu_dyn/5, mu_static=config.sph_mu_st/5,
                                                # stiction_transition_velocity=config.sph_mu_stv, friction_transition_velocity=config.sph_mu_tvc,
                                                # max_stiction_deformation=config.sph_mu_max_st_def)
                                        self.contact_group.append(contact2sprocket_upr)

                                for s in self.solids_main_teeth: # main sprocket teeth
                                        self.contacts_count+=1
                                        contact2sprocket_main_t=self.main_model.Contacts.createSolidToSolid(
                                                name=str(tab_solid.name)+"_SPRM_T"+str(self.solids_main_teeth.index(s)),adams_id=self.contacts_count,#+"_"+str(i),
                                                i_geometry = s, j_geometry = tab_solid,
                                                stiffness=self.var_contact_k.value[0], damping=self.var_contact_c.value[0],
                                                dmax=self.var_contact_d.value[0], exponent=self.var_contact_e.value[0], no_friction=True)
                                                # coulomb_friction="on", mu_dynamic=config.sph_mu_dyn/5, mu_static=config.sph_mu_st/5,
                                                # stiction_transition_velocity=config.sph_mu_stv, friction_transition_velocity=config.sph_mu_tvc,
                                                # max_stiction_deformation=config.sph_mu_max_st_def)
                                        self.contact_group.append(contact2sprocket_main_t)

                                for s in self.solids_jockey_lwr_teeth: # jockey lower teeth
                                        contact2jockey_lwr_t=self.main_model.Contacts.createSolidToSolid(
                                                name=str(tab_solid.name)+"_JL_T"+str(self.solids_jockey_lwr_teeth.index(s)),#+"_"+str(i),
                                                i_geometry = s, j_geometry = tab_solid,
                                                stiffness=self.var_contact_k.value[0], damping=self.var_contact_c.value[0],
                                                dmax=self.var_contact_d.value[0], exponent=self.var_contact_e.value[0], no_friction=True)
                                        # coulomb_friction="on", mu_dynamic=config.sph_mu_dyn, mu_static=config.sph_mu_st,
                                        # stiction_transition_velocity=config.sph_mu_stv, friction_transition_velocity=config.sph_mu_tvc,
                                        # max_stiction_deformation=config.sph_mu_max_st_def)
                                        self.contact_group.append(contact2jockey_lwr_t)

                                for s in self.solids_jockey_upr_teeth: # jockey upper teeth
                                        contact2jockey_upr_t=self.main_model.Contacts.createSolidToSolid(
                                                name=str(tab_solid.name)+"_JU_T"+str(self.solids_jockey_upr_teeth.index(s)),#+"_"+str(i),
                                                i_geometry = s, j_geometry = tab_solid,
                                                stiffness=self.var_contact_k.value[0], damping=self.var_contact_c.value[0],
                                                dmax=self.var_contact_d.value[0], exponent=self.var_contact_e.value[0], no_friction=True)
                                        # coulomb_friction="on", mu_dynamic=config.sph_mu_dyn, mu_static=config.sph_mu_st,
                                        # stiction_transition_velocity=config.sph_mu_stv, friction_transition_velocity=config.sph_mu_tvc,
                                        # max_stiction_deformation=config.sph_mu_max_st_def)
                                        self.contact_group.append(contact2jockey_upr_t)

                                for s in self.solids_cassette_teeth: # cassette teeth
                                        contact2cassette_t=self.main_model.Contacts.createSolidToSolid(
                                                name=str(tab_solid.name)+"_CAS_T"+str(self.solids_cassette_teeth.index(s)),#+"_"+str(i),
                                                i_geometry = s, j_geometry = tab_solid,
                                                stiffness=self.var_contact_k.value[0], damping=self.var_contact_c.value[0],
                                                dmax=self.var_contact_d.value[0], exponent=self.var_contact_e.value[0], no_friction=True)
                                        # coulomb_friction="on", mu_dynamic=config.sph_mu_dyn, mu_static=config.sph_mu_st,
                                        # stiction_transition_velocity=config.sph_mu_stv, friction_transition_velocity=config.sph_mu_tvc,
                                        # max_stiction_deformation=config.sph_mu_max_st_def)
                                        self.contact_group.append(contact2cassette_t)


                        # contacts for outer links
                        if '_out' in body_link.name:
                                if side == "L":
                                        for s in self.solids_main_teeth: # main sprocket teeth
                                                self.contacts_count+=1
                                                contact2sprocket_main_t=self.main_model.Contacts.createSolidToSolid(
                                                name=str(tab_solid.name)+"_SPRM_TL"+str(self.solids_main_teeth.index(s)),adams_id=self.contacts_count,#+"_"+str(i),
                                                i_geometry = s, j_geometry = tab_solid,
                                                stiffness=self.var_contact_k.value[0], damping=self.var_contact_c.value[0],
                                                dmax=self.var_contact_d.value[0], exponent=self.var_contact_e.value[0],  no_friction=True)
                                                # coulomb_friction="on", mu_dynamic=config.sph_mu_dyn/5, mu_static=config.sph_mu_st/5,
                                                # stiction_transition_velocity=config.sph_mu_stv, friction_transition_velocity=config.sph_mu_tvc,
                                                # max_stiction_deformation=config.sph_mu_max_st_def)
                                        self.contact_group.append(contact2sprocket_main_t)

                                        for s in self.solids_jockey_lwr_teeth: # jockey lower teeth
                                                contact2jockey_lwr_t=self.main_model.Contacts.createSolidToSolid(
                                                        name=str(tab_solid.name)+"_JL_T"+str(self.solids_jockey_lwr_teeth.index(s)),#+"_"+str(i),
                                                        i_geometry = s, j_geometry = tab_solid,
                                                        stiffness=self.var_contact_k.value[0], damping=self.var_contact_c.value[0],
                                                        dmax=self.var_contact_d.value[0], exponent=self.var_contact_e.value[0],  no_friction=True)
                                                # coulomb_friction="on", mu_dynamic=config.sph_mu_dyn, mu_static=config.sph_mu_st,
                                                # stiction_transition_velocity=config.sph_mu_stv, friction_transition_velocity=config.sph_mu_tvc,
                                                # max_stiction_deformation=config.sph_mu_max_st_def)
                                                self.contact_group.append(contact2jockey_lwr_t)

                                        for s in self.solids_jockey_upr_teeth: # jockey upper teeth
                                                contact2jockey_upr_t=self.main_model.Contacts.createSolidToSolid(
                                                        name=str(tab_solid.name)+"_JU_T"+str(self.solids_jockey_upr_teeth.index(s)),#+"_"+str(i),
                                                        i_geometry = s, j_geometry = tab_solid,
                                                        stiffness=self.var_contact_k.value[0], damping=self.var_contact_c.value[0],
                                                        dmax=self.var_contact_d.value[0], exponent=self.var_contact_e.value[0], no_friction=True)
                                                # coulomb_friction="on", mu_dynamic=config.sph_mu_dyn, mu_static=config.sph_mu_st,
                                                # stiction_transition_velocity=config.sph_mu_stv, friction_transition_velocity=config.sph_mu_tvc,
                                                # max_stiction_deformation=config.sph_mu_max_st_def)
                                                self.contact_group.append(contact2jockey_upr_t)

                                        for s in self.solids_cassette_teeth: # cassette teeth
                                                contact2cassette_t=self.main_model.Contacts.createSolidToSolid(
                                                        name=str(tab_solid.name)+"_CAS_T"+str(self.solids_cassette_teeth.index(s)),#+"_"+str(i),
                                                        i_geometry = s, j_geometry = tab_solid,
                                                        stiffness=self.var_contact_k.value[0], damping=self.var_contact_c.value[0],
                                                        dmax=self.var_contact_d.value[0], exponent=self.var_contact_e.value[0], no_friction=True)
                                                # coulomb_friction="on", mu_dynamic=config.sph_mu_dyn, mu_static=config.sph_mu_st,  
                                                # stiction_transition_velocity=config.sph_mu_stv, friction_transition_velocity=config.sph_mu_tvc,
                                                # max_stiction_deformation=config.sph_mu_max_st_def)
                                                self.contact_group.append(contact2cassette_t)


                                else:
                                        for s in self.solids_main_teeth: # main sprocket teeth
                                                self.contacts_count+=1
                                                contact2sprocket_main_t=self.main_model.Contacts.createSolidToSolid(
                                                name=str(tab_solid.name)+"_SPRM_TR"+str(self.solids_main_teeth.index(s)), adams_id=self.contacts_count, #+"_"+str(i),
                                                i_geometry = s, j_geometry = tab_solid,
                                                stiffness=self.var_contact_k.value[0], damping=self.var_contact_c.value[0],
                                                dmax=self.var_contact_d.value[0], exponent=self.var_contact_e.value[0], no_friction=True)
                                                # coulomb_friction="on", mu_dynamic=config.sph_mu_dyn/5, mu_static=config.sph_mu_st/5,
                                                # stiction_transition_velocity=config.sph_mu_stv, friction_transition_velocity=config.sph_mu_tvc,
                                                # max_stiction_deformation=config.sph_mu_max_st_def)
                                        self.contact_group.append(contact2sprocket_main_t)

                                        for s in self.solids_jockey_lwr_teeth: # jockey lower teeth
                                                contact2jockey_lwr_t=self.main_model.Contacts.createSolidToSolid(
                                                        name=str(tab_solid.name)+"_JL_T"+str(self.solids_jockey_lwr_teeth.index(s)),#+"_"+str(i),
                                                        i_geometry = s, j_geometry = tab_solid,
                                                        stiffness=self.var_contact_k.value[0], damping=self.var_contact_c.value[0],
                                                        dmax=self.var_contact_d.value[0], exponent=self.var_contact_e.value[0], no_friction=True)
                                                # coulomb_friction="on", mu_dynamic=config.sph_mu_dyn, mu_static=config.sph_mu_st,
                                                # stiction_transition_velocity=config.sph_mu_stv, friction_transition_velocity=config.sph_mu_tvc,
                                                # max_stiction_deformation=config.sph_mu_max_st_def)
                                                self.contact_group.append(contact2jockey_lwr_t)

                                        for s in self.solids_jockey_upr_teeth: # jockey upper teeth
                                                contact2jockey_upr_t=self.main_model.Contacts.createSolidToSolid(
                                                        name=str(tab_solid.name)+"_JU_T"+str(self.solids_jockey_upr_teeth.index(s)),#+"_"+str(i),
                                                        i_geometry = s, j_geometry = tab_solid,
                                                        stiffness=self.var_contact_k.value[0], damping=self.var_contact_c.value[0],
                                                        dmax=self.var_contact_d.value[0], exponent=self.var_contact_e.value[0], no_friction=True)
                                                # coulomb_friction="on", mu_dynamic=config.sph_mu_dyn, mu_static=config.sph_mu_st,
                                                # stiction_transition_velocity=config.sph_mu_stv, friction_transition_velocity=config.sph_mu_tvc,
                                                # max_stiction_deformation=config.sph_mu_max_st_def)
                                                self.contact_group.append(contact2jockey_upr_t)

                                        for s in self.solids_cassette_teeth: # cassette teeth
                                                contact2cassette_t=self.main_model.Contacts.createSolidToSolid(
                                                        name=str(tab_solid.name)+"_CAS_T"+str(self.solids_cassette_teeth.index(s)),#+"_"+str(i),
                                                        i_geometry = s, j_geometry = tab_solid,
                                                        stiffness=self.var_contact_k.value[0], damping=self.var_contact_c.value[0],
                                                        dmax=self.var_contact_d.value[0], exponent=self.var_contact_e.value[0], no_friction=True)
                                                # coulomb_friction="on", mu_dynamic=config.sph_mu_dyn, mu_static=config.sph_mu_st,
                                                # stiction_transition_velocity=config.sph_mu_stv, friction_transition_velocity=config.sph_mu_tvc,
                                                # max_stiction_deformation=config.sph_mu_max_st_def)
                                                self.contact_group.append(contact2cassette_t)

                        # group= self.main_model.Groups.create(name="CONTACTS_"+body_link.name+side, objects=self.contact_group)

                def tab(side, body_link, marker_loc, marker_ori, loc_x, ori_z, color):
                        self.counter_solids += 1
                        parasolids_path_tab = 'src/parasolids/chain_tab.x_t'
                        Adams.read_parasolid_file(file_name=parasolids_path_tab, part_name=body_link.name)

                        tab_solid = body_link.Geometries.__getitem__("SOLID"+ str(self.counter_solids))
                        tab_solid.name = "SOLID_LINK_"+ side + str(self.counter_solids)
                        tab_solid.color_name = color

                        psmar = body_link.Markers.__getitem__("PSMAR")
                        psmar.name = "PS" + tab_solid.name
                        psmar.location = rel_loc_custom(marker_loc, [loc_x, 0, 0])
                        psmar.orientation = rel_orient_custom(marker_ori, [ori_z, 0, 0])

                        # tab_contacts(side, body_link, tab_solid)

                def joint_rev(body_i, body_j, marker_4_loc, i):
                        joint_sph = self.main_model.Constraints.createRevolute(name="JOINT_REV_"+ str(i) +"_"+ str(i+1),
                                i_part=body_i, j_part=body_j, location=rel_loc_custom(marker_4_loc, [0, 0, 0]), orientation=[0,90,0])

                def joint_sph(body_i, body_j, marker_4_loc, i):
                        joint_sph = self.main_model.Constraints.createSpherical(name="JOINT_SPH_"+ str(i) +"_"+ str(i+1),
                                i_part=body_i, j_part=body_j, location=rel_loc_custom(marker_4_loc, [0, 0, 0]), orientation=[0,90,0])
                        
                        joint_friction = self.main_model.Forces.createFriction(name="FR_"+joint_sph.name, joint=joint_sph,
                                formulation="original", mu_dynamic=config.sph_mu_dyn, mu_static=config.sph_mu_st,
                                stiction_transition_velocity=config.sph_mu_stv, transition_velocity_coefficient=config.sph_mu_tvc,
                                max_stiction_deformation=config.sph_mu_max_st_def, ball_radius=config.sph_mu_ball_r, effect="all",
                                friction_torque_preload=config.sph_mu_trq_pr)

                # FIRST LINK
                body_link_1_out = self.main_model.Parts.createRigidBody(name="body_link_1_out")

                mar_link_1_out = body_link_1_out.Markers.create(name="MAR_LINK_1_END",
                        location =[0, pins_y[0], pins_z[0]], orientation=[0,0,0])
                markers_links_out_ctr.append(mar_link_1_out)
                self.bodies_outer_links.append(body_link_1_out)
                mar_link_1_out_ctr = body_link_1_out.Markers.create(name="MAR_LINK_1_CTR",
                        location = [0, pins_y[-1], pins_z[-1]], orientation=ori_one_axis([0, pins_y[-1], pins_z[-1]], mar_link_1_out.location, "y"))
                links_cm(body_link_1_out, mar_link_1_out_ctr)
                # tab("L", body_link_1_out, mar_link_1_out_ctr, mar_link_1_out_ctr, config.tab_out_ctr_loc_x, 0, config.color_link_out)
                # tab("R", body_link_1_out, mar_link_1_out, mar_link_1_out_ctr, -config.tab_out_end_loc_x, config.tab_out_end_ori_z, config.color_link_out)
                
                for i in range(1,len(pins_y)):
                        point_loc = [0, pins_y[i], pins_z[i]]
                        if i % 2 != 0:
                                body_link = self.main_model.Parts.createRigidBody(name="body_link_"+str(i+1)+"_in")
                                self.bodies_inner_links.append(body_link)
                                mar_link_in_ctr = body_link.Markers.create(name="MAR_LINK_IN_" + str(i+1) +"_CTR",
                                        location =point_loc, orientation=ori_one_axis(point_loc, markers_links_out_ctr[-1].location, "y"))
                                # if mar_link_in_ctr.orientation[2]==180:
                                #         mar_link_in_ctr.orientation = [mar_link_in_ctr.orientation[0], mar_link_in_ctr.orientation[1], 0]
                                markers_links_in_ctr.append(mar_link_in_ctr)
                                links_cm(body_link, mar_link_in_ctr)
                                # tab("L", body_link, mar_link_in_ctr, mar_link_in_ctr, config.tab_inr_ctr_loc_x, 0, config.color_link_inr)
                                # tab("R", body_link, markers_links_out_ctr[-1], mar_link_in_ctr, config.tab_inr_end_loc_x, config.tab_inr_end_ori_z, config.color_link_inr)
                                if i ==len(pins_y)-1:
                                        joint_sph(body_link, self.bodies_outer_links[-1], markers_links_out_ctr[-1], i)
                                else:
                                        joint_rev(body_link, self.bodies_outer_links[-1], markers_links_out_ctr[-1], i)
                                roller_circle(body_link, mar_link_in_ctr, markers_links_out_ctr[-1], i)
                                
                        else:   
                                body_link = self.main_model.Parts.createRigidBody(name="body_link_"+str(i+1)+"_out")
                                self.bodies_outer_links.append(body_link)
                                mar_link_out_ctr = body_link.Markers.create(name="MAR_LINK_OUT_" + str(i+1) +"_CTR",
                                        location =point_loc, orientation=ori_one_axis(point_loc, markers_links_in_ctr[-1].location, "y"))
                                # if mar_link_out_ctr.orientation[2]==180:
                                #         mar_link_out_ctr.orientation = [mar_link_out_ctr.orientation[0], mar_link_out_ctr.orientation[1], 0]
                                markers_links_out_ctr.append(mar_link_out_ctr)
                                links_cm(body_link, mar_link_out_ctr)
                                # tab("L", body_link, mar_link_out_ctr, mar_link_out_ctr, config.tab_out_ctr_loc_x, 0, config.color_link_out)
                                # tab("R", body_link, markers_links_in_ctr[-1], mar_link_out_ctr, config.tab_out_end_loc_x, config.tab_out_end_ori_z, config.color_link_out)
                                joint_rev(body_link, self.bodies_inner_links[-1], markers_links_in_ctr[-1], i)
                
                joint_sph(body_link_1_out, self.bodies_inner_links[-1], mar_link_1_out_ctr, 122)
                self.main_model.Constraints.createPlanar(name= 'INPLANE_1_OUT', i_part=body_link_1_out, j_part=self.ground,
                                location=mar_link_1_out_ctr.location, orientation=[0,90,0]) 

                # markers_links_in_ctr.append(mar_link_1_ctr)

                # mar_link_1_end = body_link_1_in.Markers.create(name="MAR_LINK_1_END",
                #                 location = rel_loc_custom(mar_link_1_ctr, [0, config.rollers_dist, 0]),
                #                 orientation=rel_orient_custom(mar_link_1_ctr, [0, 0, 0]),
                #                 size_of_icons=config.soi_arcs_small)

                # self.bodies_inner_links.append(body_link_1_in)
                # markers_links_in_end.append(mar_link_1_end)

                # roller_circle(body_link_1_in, mar_link_1_ctr, mar_link_1_end, 1)
                # links_cm(body_link_1_in, mar_link_1_ctr)
                # tab("L", body_link_1_in, mar_link_1_ctr, config.tab_inr_ctr_loc_x, 0, 0, config.color_link_inr)
                # tab("R", body_link_1_in, mar_link_1_end, config.tab_inr_end_loc_x, config.tab_inr_end_ori_z, 0, config.color_link_inr)
                

                # for i in range(2,4):

                #         if i % 2 != 0:

                #                 body_link_in = self.main_model.Parts.createRigidBody(name="body_link_"+ str(i) + "_in")
                #                 self.bodies_inner_links.append(body_link_in)

                #                 marker_link_ctr = self.bodies_inner_links[-1].Markers.create(name="MAR_LINK_IN_CTR_"+ str(i),
                #                         location = rel_loc_custom(markers_links_out_end[-1], [0, 0, 0]),
                #                         orientation=rel_orient_custom(markers_links_out_end[-1], [0, -config.links_init_angle, 0]),
                #                         size_of_icons=config.soi_arcs_small)
                #                 markers_links_in_ctr.append(marker_link_ctr)


                #                 marker_link_end = self.bodies_inner_links[-1].Markers.create(name="MAR_LINK_IN_END_"+ str(i),
                #                         location = rel_loc_custom(marker_link_ctr, [0, config.rollers_dist, 0]),
                #                         orientation=rel_orient_custom(marker_link_ctr, [0, 0, 0]),
                #                         size_of_icons=config.soi_arcs_small)
                #                 markers_links_in_end.append(marker_link_end)

                #                 roller_circle(body_link_in, marker_link_ctr, marker_link_end, i)
                #                 links_cm(body_link_in, marker_link_ctr)
                #                 tab("L", body_link_in, marker_link_ctr, config.tab_inr_ctr_loc_x, 0, i, config.color_link_inr)
                #                 tab("R", body_link_in, marker_link_end, config.tab_inr_end_loc_x, config.tab_inr_end_ori_z, i, config.color_link_inr)
                #                 joint_sph(body_link_in, self.bodies_outer_links[-1], marker_link_ctr, i)


                #         else:
                #                 body_link_out = self.main_model.Parts.createRigidBody(name="body_link_"+ str(i) + "_out")
                #                 self.bodies_outer_links.append(body_link_out)

                #                 marker_link_ctr = self.bodies_outer_links[-1].Markers.create(name="MAR_LINK_OUT_CTR_"+ str(i),
                #                         location = rel_loc_custom(markers_links_in_end[-1], [0, 0, 0]),
                #                         orientation=rel_orient_custom(markers_links_in_end[-1], [0, -config.links_init_angle, 0]),
                #                         size_of_icons=config.soi_arcs_small)
                #                 markers_links_out_ctr.append(marker_link_ctr)

                #                 marker_link_end = self.bodies_outer_links[-1].Markers.create(name="MAR_LINK_IN_END_"+ str(i),
                #                         location = rel_loc_custom(marker_link_ctr, [0, config.rollers_dist, 0]),
                #                         orientation=rel_orient_custom(marker_link_ctr, [0, 0, 0]),
                #                         size_of_icons=config.soi_arcs_small)
                #                 markers_links_out_end.append(marker_link_end)

                #                 links_cm(body_link_out, marker_link_ctr)
                #                 tab("L", body_link_out, marker_link_ctr, config.tab_out_ctr_loc_x, 0, i, config.color_link_out)
                #                 tab("R", body_link_out, marker_link_end, config.tab_out_end_loc_x, config.tab_out_end_ori_z, i, config.color_link_out)
                #                 joint_sph(body_link_out, self.bodies_inner_links[-1], marker_link_ctr, i)

    def set_simscript(self):
            sim_script=["integrator/I3,newmark,error=1.0E-04",
                        "simulate/dynamics, end=1, dtout=0.1",
                        "springdamper/1, ct=10",
                        "simulate/dynamics, end=3, dtout=0.01",
                        "stop"]
            self.main_model.Simulations.create(name="SIM1", script_type='solver_commands', script=sim_script)
               

    def simulate(self):
            import time

            simulation = self.main_model.Simulations.create(name="SIM1", duration=5, number_of_steps=500)
            Adams.read_command_file("src/modeling/cmd_files/verify.cmd")
            simulation.simulate()
            # self.visualize()
            self.tsbo_cost()

            # Specify the file to monitor
            file_to_monitor = 'src/matlab/TSBO/output.txt'
            file_break_py = 'src/matlab/TSBO/break_py.txt'

            # Initialize a variable to keep track of the number of lines
            previous_num_lines = 0

            print("Monitoring the file for new lines...")

            while True:

                    with open(file_break_py, 'r') as file:
                    # Read all lines and take the last one
                            lines = file.readlines()
                            last_line = lines[-1] if lines else ""

                    # Check if "break" is in the last line
                    if "break" in last_line:
                            print("break the monitoring")
                            break

                    with open(file_to_monitor, "r") as file:
                            lines = file.readlines()

                    # Count the number of lines in the file
                    current_num_lines = len(lines)

                    # Check if new lines have been added
                    if current_num_lines > previous_num_lines:
                            # New lines detected
                            print("New line(s) detected!")
                            # Process the new lines
                            new_lines = lines[previous_num_lines:current_num_lines]
                            print("New line content:")
                            for line in new_lines:
                                    print(line.strip())

                            output_file = 'src/matlab/TSBO/output.txt'
                            for contact in self.contacts_2d:
                                    with open(output_file, "r") as file:
                                            lines = file.readlines()
                                            actual_vals = lines[-1]
                                            contact.stiffness=float(actual_vals.split()[0])
                                            contact.exponent=float(actual_vals.split()[1])
                                            contact.damping=float(actual_vals.split()[2])
                                            contact.dmax=float(actual_vals.split()[3])

                                            self.var_contact_k.value = float(actual_vals.split()[0])
                                            self.var_contact_e.value = float(actual_vals.split()[1])
                                            self.var_contact_c.value = float(actual_vals.split()[2])
                                            self.var_contact_d.value = float(actual_vals.split()[3])

                                            self.contact_dummy.normal_function = [self.var_contact_k.value[0], self.var_contact_e.value[0],
                                                                            self.var_contact_c.value[0], self.var_contact_d.value[0]]
                            simulation.simulate()
                            self.tsbo_cost()
                            # self.visualize()

                            # Update the previous line count
                            previous_num_lines = current_num_lines

            # Pause for a short interval to avoid excessive CPU usage
            time.sleep(1)

def visualize():
        from plotly.subplots import make_subplots
        import plotly.graph_objects as go
        set = load_plot_settings()
        model =  Adams.stoo(".CHAIN_TEST")
        analysis = model.Analyses["Last_Run"]
        cpu_mea = analysis.results["CPU_MEA"]["Q"].values
        res_time = analysis.results["TIME"].values

        with open("src/data/submodel1_data.txt", "r") as file:
                cpu1 = [float(line.strip()) for line in file]

        x = res_time
        fig1 = make_subplots(rows=1, cols=1, shared_xaxes=True)
        fig1.add_trace(go.Scatter(x=x, y=cpu_mea,
                marker=dict(color=set.TUMBlue), name='<span style="font: Arial; font-size: 11pt"> Modell',
                legendgroup='1'), row=1, col=1)
        fig1.add_trace(go.Scatter(x=x, y=cpu1,
                marker=dict(color=set.TUMOrange), name='<span style="font: Arial; font-size: 11pt"> Modell',
                legendgroup='1'), row=1, col=1)

        fig1.update_layout(
                yaxis_title=dict(text='<span style="font-family: Arial; font-size: 11pt"'
                                '> Weg  <i>DZ</i> <' # type here
                                '/span>'
                                '<span style="font-family: Arial; font-size: 11pt"'
                                '> in <' # type here
                                '/span>'
                                '<span style="font-family: Sitka Heading; font-size: 13pt"'
                                '> mm <' # type here
                                '/span>'),
                xaxis_title=dict(text='<span style="font-family: Arial; font-size: 11pt"'
                                '>Zeit in der Simulation in <' # type here
                                '/span> <span style="font-family: Sitka Heading; font-size: 13pt"'
                                '>s<' # type here
                                '/span>'),

                xaxis1=dict(tickfont=dict(family="Cambria", size=13)),
                yaxis1=dict(tickfont=dict(family="Cambria", size=13)),
                plot_bgcolor='white',
        )

        fig1.update_layout(plot_bgcolor='white', legend_tracegroupgap=60)
        fig1.update_xaxes(
                mirror=True,
                ticks='inside',
                ticklen=10,
                showline=True,
                zeroline=True,
                zerolinewidth=1,
                zerolinecolor='lightgrey',
                linecolor='black',
                gridcolor='lightgrey',
                gridwidth=1,
        )
        fig1.update_yaxes(
                mirror=True,
                ticks='inside',
                ticklen=10,
                showline=True,
                zeroline=True,
                zerolinewidth=1,
                zerolinecolor='lightgrey',
                linecolor='black',
                gridcolor='lightgrey',
                gridwidth=1,
                exponentformat="none",
                separatethousands=True,
                # title_standoff=90
                # dtick=3
        )

        fig1.update_layout(
                legend=dict(x=0.9, y=0.95, xanchor='right', yanchor='top', font=dict(family="Arial", size=29),itemsizing='constant'),
                legend_bordercolor='#000000',
                legend_borderwidth=1,
                font_color="black",
                legend_tracegroupgap=320
        )

        fig1.update_layout(title='<span style="font-family: Arial; font-size: 11pt"'
                            '> Weg des letzten Kettenglieds in Z-Richtung<' # type here
                            '/span>')
        
        fig1.update_layout(
        title={
                'y':0.75,
                'x':0.5,
                'xanchor': 'center',
                'yanchor': 'top'})

        fig1.update_layout(autosize=False, width=642, height=300)
        fig1.show()
        # fig1.write_image("fig1.jpg", width=642, height=300, scale=5)

def write_res():
        model =  Adams.stoo(".CHAIN_TEST")
        analysis = model.Analyses["Last_Run"]
        cpu_mea = analysis.results["CPU_MEA"]["Q"].values

        with open("src/data/submodel1_data.txt", "w") as file:
                for num in cpu_mea:
                        file.write(f"{num}\n")


model = MODEL()
model.frame()
model.rocker()
model.sus_link_lwr()
model.sus_link_upr()
model.damper_cylinder()
model.damper_piston()
# model.fbar_calc()
# model.sprocket_main()
model.sprocket_main_single()
# model.sprocket_upr()
model.sprocket_upr_single()

model.cassette()
model.derailleur_stat()
model.link_inr()
model.link_out()
model.knuckle()
model.der_cage()
model.jockey_upr()
model.jockey_lwr()

model.chain_2d()

Adams.read_command_file("src/modeling/cmd_files/verify.cmd")

# model.set_simscript()
# model.simulate()
                        
# write_res()
# visualize()
