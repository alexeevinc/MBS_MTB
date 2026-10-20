import sys
import os
import Adams
import math as m
import numpy as np
import toml


sys.path.append(os.path.abspath('C:/1_A/TUM/MA2/Adams/repo_1'))

from src.utils.datamanager import load_config, load_plot_settings
from src.utils.custom_tools import rel_loc_custom, rel_orient_custom, loc_mirror_zx

config = load_config()


class MODEL():
        def __init__(self):
                ### STARTER PACK ### 
                self.counter_solids = 0
                self.solids = []
                self.contacts_2d = []
                # MODEL BASE
                Adams.defaults.units.setUnits(length='mm',mass='kg',time = 'second',angle='degrees',force='newton', frequency='hz')
                Adams.execute_cmd("default unit orient=body123") # changes the rotation sequence from default 313 to 123
                self.main_model = Adams.Models.create(name='CHAIN_SIMPLE') 
                self.main_model.size_of_icons = config.soi_model
                self.ground = self.main_model.ground_part
                gravity = self.main_model.Forces.createGravity(name='gravity', xyz_component_gravity = [0, 0, -9807 ])

                self.mar_ground = self.ground.Markers.create(name="MAR_GROUND")

                measure_cpu = self.main_model.Measures.createFunction(name= "CPU_MEA", function="CPU", units="no_units", create_measure_display="no") 
                
                
                #creates a request for study of contact properties
                # Adams.read_command_file("src/modeling/cmd_files/request.cmd")
                # design variables for contacts
                self.var_contact_k = self.main_model.DesignVariables.createReal(name = "grip_cont_k", 
                        value = 1.07E+01, units="no_units", range=[1.0, 1.0E+08], use_allowed_values = "no")
                self.var_contact_e = self.main_model.DesignVariables.createReal(name = "grip_cont_e", 
                        value = 1.5, units="no_units", range=[1.1, 2.4], use_allowed_values = "no")
                self.var_contact_c = self.main_model.DesignVariables.createReal(name = "grip_cont_c", 
                        value = 10, units="no_units", range=[0.0, 1.0E+04], use_allowed_values = "no")
                self.var_contact_d = self.main_model.DesignVariables.createReal(name = "grip_cont_d", 
                        value = 0.1, units="no_units", range=[1.0E-05, 1.0], use_allowed_values = "no")
                

        def chain(self):
                        config = load_config()

                        # TARGET END POSITION
                        mar_y_target = self.ground.Markers.create(name="MAR_Y_TARGET",
                                location =[0, 70, 0], orientation=[0, 0, 0],
                                size_of_icons=config.soi_arcs_small)

                        
                        # PARAMETERS FOR BISTOP                
                        self.main_model.DesignVariables.createReal(name = "bistop_penetration_d", 
                                value = 1.07E-03, units="angle", delta_type="relative", range=[0,0], use_allowed_values = "no")
                        
                        self.main_model.DesignVariables.createReal(name = "bistop_force_deform_exp_e", 
                                value = 1.43, units="no_units", delta_type="relative", range=[0,0], use_allowed_values = "no")
                        
                        self.main_model.DesignVariables.createReal(name = "bistop_max_damping_cmax", 
                                value = 1.92, units="damping_per_angle", delta_type="relative", range=[0,0], use_allowed_values = "no")
                        
                        self.main_model.DesignVariables.createReal(name = "bistop_stiffness_k", 
                                value = 380, units="force_per_angle", delta_type="relative", range=[0,0], use_allowed_values = "no")

                        self.main_model.DesignVariables.createReal(name = "bistop_bound_upr", 
                                value = 0.52, units="angle", delta_type="relative", range=[0,0], use_allowed_values = "no")
                        
                        self.main_model.DesignVariables.createReal(name = "bistop_bound_lwr", 
                                value = -0.52, units="angle", delta_type="relative", range=[0,0], use_allowed_values = "no")
                        
                        self.main_model.DesignVariables.createReal(name = "joint_mu_dyn", 
                                value = 0.5, units="no_units", delta_type="relative", range=[0,0], use_allowed_values = "no")
                        
                        bodies_inner_links = []
                        markers_links_in_ctr = []
                        markers_links_in_end = []

                        bodies_outer_links = []
                        markers_links_out_ctr = []
                        markers_links_out_end = []

                        def roller_circle(body_link_in, marker_link_ctr, marker_link_end, i):
                                mar_roller_circle_ctr = bodies_inner_links[-1].Markers.create(name="MAR_ROLLER_CTR_"+ str(i),
                                        location = rel_loc_custom(marker_link_ctr, [0,0,0]),
                                        orientation=rel_orient_custom(marker_link_ctr, [-90, 90, 90]),
                                        size_of_icons=config.soi_arcs_small)

                                circle_ctr = body_link_in.Geometries.createCircle(name="CIRCLE_CTR"+ str(i), 
                                        center_marker=mar_roller_circle_ctr, radius=config.roll_radius) 
                                
                                mar_roller_cyl_side1 = bodies_inner_links[-1].Markers.create(name="MAR_ROLLER_CYL_SIDE1_"+ str(i),
                                        location=rel_loc_custom(mar_roller_circle_ctr, [0,0,-1*(config.roll_cyl_len/2)]),
                                        orientation=rel_orient_custom(mar_roller_circle_ctr, [0,0,0]))
                                
                                roller_cylinder1 = bodies_inner_links[-1].Geometries.createCylinder(name="ROLLER_CYL1_"+ str(i),
                                        center_marker = mar_roller_cyl_side1, length=config.roll_cyl_len, radius=config.roll_radius)

                                mar_roller_circle_end = bodies_inner_links[-1].Markers.create(name="MAR_ROLLER_END_"+ str(i),
                                        location = rel_loc_custom(marker_link_end, [0,0,0]),
                                        orientation=rel_orient_custom(marker_link_end, [-90, 90, 90]),
                                        size_of_icons=config.soi_arcs_small)
                                
                                mar_roller_cyl_side2 = bodies_inner_links[-1].Markers.create(name="MAR_ROLLER_CYL_SIDE2_"+ str(i),
                                        location=rel_loc_custom(mar_roller_circle_end, [0,0,-1*(config.roll_cyl_len/2)]),
                                        orientation=rel_orient_custom(mar_roller_circle_end, [0,0,0]))
                                
                                roller_cylinder2 = bodies_inner_links[-1].Geometries.createCylinder(name="ROLLER_CYL2_"+ str(i),
                                        center_marker = mar_roller_cyl_side2, length=config.roll_cyl_len, radius=config.roll_radius)

                                circle_end = body_link_in.Geometries.createCircle(name="CIRCLE_END"+ str(i), 
                                        center_marker=mar_roller_circle_end, radius=3.965) 
                                
                        def links_cm(body_link, marker_link_ctr):
                                marker_link_cm = body_link.Markers.create(name="MAR_LINK_1_CM",
                                        location = rel_loc_custom(marker_link_ctr, [0, config.rollers_dist/2, 0]),
                                        orientation=rel_orient_custom(marker_link_ctr, [0, 0, 0]),
                                        size_of_icons=config.soi_arcs_small)

                                body_link.cm = marker_link_cm
                                body_link.mass = 0.005
                                body_link.inertia_values = [0.09, 0.142, 0.08, 0,0,0]

                        def tabs_import(body_link, marker_on_link, loc_x, ori_z, i, color):
                                self.counter_solids += 1
                                parasolids_path_tab = 'src/parasolids/chain_tab.x_t'
                                Adams.read_parasolid_file(file_name=parasolids_path_tab, part_name=body_link.name)
                                
                                tab_solid = body_link.Geometries.__getitem__("SOLID"+ str(self.counter_solids))
                                tab_solid.name = "SOLID_LINK_" + str(self.counter_solids)
                                tab_solid.color_name = color

                                psmar = body_link.Markers.__getitem__("PSMAR")
                                psmar.name = "PS" + marker_on_link.name
                                psmar.location = rel_loc_custom(marker_on_link, [loc_x, 0, 0])
                                psmar.orientation = rel_orient_custom(marker_on_link, [ori_z, 0, 0])
                                # if '_in' in body_link.name:
                                #         for s in self.solids_spr_upr:
                                #                 contact2sprocket=self.main_model.Contacts.createSolidToSolid(
                                #                         name=str(tab_solid.name)+str(self.solids_spr_upr.index(s))+str(i),
                                #                         i_geometry = s, j_geometry = tab_solid,
                                #                         stiffness=100)

                        def vtorque(i_marker_link, j_part, j_marker, i):
                                floatingmarker = j_part.FloatingMarkers.create(name="MAR_FLOAT"+str(i)) 
                                torque_vector = self.main_model.Forces.createTorqueVector(name="TV_"+str(i),
                                        i_marker=i_marker_link, j_floating_marker=floatingmarker, ref_marker=j_marker,
                                        x_torque_function="0",
                                        y_torque_function="BISTOP(AY("+i_marker_link.name+","+j_marker.name+"),"+
                                                "WY("+i_marker_link.name+","+j_marker.name+","+j_marker.name+"),"+
                                                "(bistop_bound_lwr) , (bistop_bound_upr) , (bistop_stiffness_k) , (bistop_force_deform_exp_e) ,"+
                                                 "(bistop_max_damping_cmax) , (bistop_penetration_d))",
                                        z_torque_function="BISTOP(AZ("+i_marker_link.name+","+j_marker.name+"),"+
                                                "WZ("+i_marker_link.name+","+j_marker.name+","+j_marker.name+"),"+
                                                "(bistop_bound_lwr) , (bistop_bound_upr) , (bistop_stiffness_k) , (bistop_force_deform_exp_e) ,"+
                                                 "(bistop_max_damping_cmax) , (bistop_penetration_d))") 

                        # FIRST LINK
                        body_link_1_in = self.main_model.Parts.createRigidBody(name="body_link_1_in")


                        mar_chain_ctr = body_link_1_in.Markers.create(name="MAR_CHAIN_CTR",
                                location =[0, 0, 0], orientation=[0, -90, 0])
                        
                        mar_chain_rad = body_link_1_in.Markers.create(name="MAR_CHAIN_RAD",
                                location =rel_loc_custom(mar_chain_ctr, [0, 0, 0]), 
                                orientation=rel_orient_custom(mar_chain_ctr, [0, 0, 0]),
                                size_of_icons=config.soi_arcs_small)
                        
                        mar_link_1_ctr = body_link_1_in.Markers.create(name="MAR_LINK_1_CTR",
                                location =rel_loc_custom(mar_chain_rad, [0, 0, 0]), 
                                orientation=rel_orient_custom(mar_chain_rad, [0, 0, 0]),
                                size_of_icons=config.soi_arcs_small)
                        
                        mar_link_1_end = body_link_1_in.Markers.create(name="MAR_LINK_1_END",
                                        location = rel_loc_custom(mar_link_1_ctr, [0, config.rollers_dist, 0]),
                                        orientation=rel_orient_custom(mar_link_1_ctr, [0, 0, 0]),
                                        size_of_icons=config.soi_arcs_small)

                        bodies_inner_links.append(body_link_1_in)
                        markers_links_in_end.append(mar_link_1_end)

                        roller_circle(body_link_1_in, mar_link_1_ctr, mar_link_1_end, 1)
                        links_cm(body_link_1_in, mar_link_1_ctr)
                        tabs_import(body_link_1_in, mar_link_1_ctr, config.tab_inr_ctr_loc_x, 0, 0, config.color_link_inr)
                        tabs_import(body_link_1_in, mar_link_1_end, config.tab_inr_end_loc_x, config.tab_inr_end_ori_z, 0, config.color_link_inr)
                        
                        joint_link_fixed = self.main_model.Constraints.createFixed(name="LINK_FIX", 
                                i_part=body_link_1_in, j_part=self.ground,
                                location=[0,0,0]) 
 
                        # for i in range(2, 3):

                        #         if i % 2 != 0: 
                                
                        #                 body_link_in = self.main_model.Parts.createRigidBody(name="body_link_"+ str(i) + "_in")
                        #                 bodies_inner_links.append(body_link_in)

                        #                 marker_link_ctr = bodies_inner_links[-1].Markers.create(name="MAR_LINK_IN_CTR_"+ str(i),
                        #                         location = rel_loc_custom(markers_links_out_end[-1], [0, 0, 0]),
                        #                         orientation= [0, 0, 0],
                        #                         size_of_icons=config.soi_arcs_small)
                        #                 markers_links_in_ctr.append(marker_link_ctr)

                        #                 vtorque(marker_link_ctr, bodies_outer_links[-1], markers_links_out_end[-1], i)
                                        
                        #                 marker_link_end = bodies_inner_links[-1].Markers.create(name="MAR_LINK_IN_END_"+ str(i),
                        #                         location = rel_loc_custom(marker_link_ctr, [0, config.rollers_dist, 0]),
                        #                         orientation=rel_orient_custom(marker_link_ctr, [0, 0, 0]),
                        #                         size_of_icons=config.soi_arcs_small)
                        #                 markers_links_in_end.append(marker_link_end)

                        #                 roller_circle(body_link_in, marker_link_ctr, marker_link_end, i)
                        #                 links_cm(body_link_in, marker_link_ctr)
                        #                 tabs_import(body_link_in, marker_link_ctr, config.tab_inr_ctr_loc_x, 0, i, config.color_link_inr)
                        #                 tabs_import(body_link_in, marker_link_end, config.tab_inr_end_loc_x, config.tab_inr_end_ori_z, i, config.color_link_inr)

                        #                 joint_sph = self.main_model.Constraints.createSpherical(name="JOINT_SPH_"+ str(i-1) + str(i),
                        #                         i_part=body_link_in, j_part=bodies_outer_links[-1], location=rel_loc_custom(marker_link_ctr, [0, 0, 0])) 
                                        
                        #                 joint_friction = self.main_model.Forces.createFriction(name="FR_"+joint_sph.name, joint=joint_sph, 
                        #                         formulation="original", mu_dynamic=config.sph_mu_dyn, mu_static=config.sph_mu_st, 
                        #                         stiction_transition_velocity=config.sph_mu_stv, transition_velocity_coefficient=config.sph_mu_tvc,
                        #                         max_stiction_deformation=config.sph_mu_max_st_def, ball_radius=config.sph_mu_ball_r, effect="all",
                        #                         friction_torque_preload=config.sph_mu_trq_pr)

                        #         else:
                        #                 body_link = self.main_model.Parts.createRigidBody(name="body_link_"+ str(i) + "_out")
                        #                 bodies_outer_links.append(body_link)

                        #                 marker_link_ctr = bodies_outer_links[-1].Markers.create(name="MAR_LINK_OUT_CTR_"+ str(i),
                        #                         location = rel_loc_custom(markers_links_in_end[-1], [0, 0, 0]),
                        #                         orientation=[0, 0, 0],
                        #                         size_of_icons=config.soi_arcs_small)
                        #                 markers_links_out_ctr.append(marker_link_ctr)

                        #                 vtorque(marker_link_ctr, bodies_inner_links[-1], markers_links_in_end[-1], i)

                        #                 marker_link_end = bodies_outer_links[-1].Markers.create(name="MAR_LINK_IN_END_"+ str(i),
                        #                         location = rel_loc_custom(marker_link_ctr, [0, config.rollers_dist, 0]),
                        #                         orientation=rel_orient_custom(marker_link_ctr, [0, 0, 0]),
                        #                         size_of_icons=config.soi_arcs_small)
                        #                 markers_links_out_end.append(marker_link_end)

                        #                 links_cm(body_link, marker_link_ctr)
                        #                 tabs_import(body_link, marker_link_ctr, config.tab_out_ctr_loc_x, 0, i, config.color_link_out)
                        #                 tabs_import(body_link, marker_link_end, config.tab_out_end_loc_x, config.tab_out_end_ori_z, i, config.color_link_out)

                        #                 joint_sph = self.main_model.Constraints.createSpherical(name="JOINT_SPH_"+ str(i-1) + str(i),
                        #                         i_part=body_link, j_part=bodies_inner_links[-1], location=rel_loc_custom(marker_link_ctr, [0, 0, 0])) 

                        #                 joint_friction = self.main_model.Forces.createFriction(name="FR_"+joint_sph.name, joint=joint_sph, 
                        #                         formulation="original", mu_dynamic=config.sph_mu_dyn, mu_static=config.sph_mu_st, 
                        #                         stiction_transition_velocity=config.sph_mu_stv, transition_velocity_coefficient=config.sph_mu_tvc,
                        #                         max_stiction_deformation=config.sph_mu_max_st_def, ball_radius=config.sph_mu_ball_r, effect="all",
                        #                         friction_torque_preload=config.sph_mu_trq_pr) 

                        for i in range(2, 16):

                                if i % 2 != 0: 
                                
                                        body_link_in = self.main_model.Parts.createRigidBody(name="body_link_"+ str(i) + "_in")
                                        bodies_inner_links.append(body_link_in)

                                        marker_link_ctr = bodies_inner_links[-1].Markers.create(name="MAR_LINK_IN_CTR_"+ str(i),
                                                location = rel_loc_custom(markers_links_out_end[-1], [0, 0, 0]),
                                                orientation= [0, -90, 0],
                                                size_of_icons=config.soi_arcs_small)
                                        markers_links_in_ctr.append(marker_link_ctr)

                                        vtorque(marker_link_ctr, bodies_outer_links[-1], markers_links_out_end[-1], i)
                                        
                                        marker_link_end = bodies_inner_links[-1].Markers.create(name="MAR_LINK_IN_END_"+ str(i),
                                                location = rel_loc_custom(marker_link_ctr, [0, config.rollers_dist, 0]),
                                                orientation=rel_orient_custom(marker_link_ctr, [0, 0, 0]),
                                                size_of_icons=config.soi_arcs_small)
                                        markers_links_in_end.append(marker_link_end)

                                        roller_circle(body_link_in, marker_link_ctr, marker_link_end, i)
                                        links_cm(body_link_in, marker_link_ctr)
                                        tabs_import(body_link_in, marker_link_ctr, config.tab_inr_ctr_loc_x, 0, i, config.color_link_inr)
                                        tabs_import(body_link_in, marker_link_end, config.tab_inr_end_loc_x, config.tab_inr_end_ori_z, i, config.color_link_inr)

                                        joint_sph = self.main_model.Constraints.createSpherical(name="JOINT_SPH_"+ str(i-1) + str(i),
                                                i_part=body_link_in, j_part=bodies_outer_links[-1], location=rel_loc_custom(marker_link_ctr, [0, 0, 0])) 
                                        
                                        joint_friction = self.main_model.Forces.createFriction(name="FR_"+joint_sph.name, joint=joint_sph, 
                                                formulation="original", mu_dynamic=config.sph_mu_dyn, mu_static=config.sph_mu_st, 
                                                stiction_transition_velocity=config.sph_mu_stv, transition_velocity_coefficient=config.sph_mu_tvc,
                                                max_stiction_deformation=config.sph_mu_max_st_def, ball_radius=config.sph_mu_ball_r, effect="all",
                                                friction_torque_preload=config.sph_mu_trq_pr)

                                else:
                                        body_link = self.main_model.Parts.createRigidBody(name="body_link_"+ str(i) + "_out")
                                        bodies_outer_links.append(body_link)

                                        marker_link_ctr = bodies_outer_links[-1].Markers.create(name="MAR_LINK_OUT_CTR_"+ str(i),
                                                location = rel_loc_custom(markers_links_in_end[-1], [0, 0, 0]),
                                                orientation=[0, -90, 0],
                                                size_of_icons=config.soi_arcs_small)
                                        markers_links_out_ctr.append(marker_link_ctr)

                                        vtorque(marker_link_ctr, bodies_inner_links[-1], markers_links_in_end[-1], i)

                                        marker_link_end = bodies_outer_links[-1].Markers.create(name="MAR_LINK_IN_END_"+ str(i),
                                                location = rel_loc_custom(marker_link_ctr, [0, config.rollers_dist, 0]),
                                                orientation=rel_orient_custom(marker_link_ctr, [0, 0, 0]),
                                                size_of_icons=config.soi_arcs_small)
                                        markers_links_out_end.append(marker_link_end)

                                        links_cm(body_link, marker_link_ctr)
                                        tabs_import(body_link, marker_link_ctr, config.tab_out_ctr_loc_x, 0, i, config.color_link_out)
                                        tabs_import(body_link, marker_link_end, config.tab_out_end_loc_x, config.tab_out_end_ori_z, i, config.color_link_out)

                                        joint_sph = self.main_model.Constraints.createSpherical(name="JOINT_SPH_"+ str(i-1) + str(i),
                                                i_part=body_link, j_part=bodies_inner_links[-1], location=rel_loc_custom(marker_link_ctr, [0, 0, 0])) 

                                        joint_friction = self.main_model.Forces.createFriction(name="FR_"+joint_sph.name, joint=joint_sph, 
                                                formulation="original", mu_dynamic=config.sph_mu_dyn, mu_static=config.sph_mu_st, 
                                                stiction_transition_velocity=config.sph_mu_stv, transition_velocity_coefficient=config.sph_mu_tvc,
                                                max_stiction_deformation=config.sph_mu_max_st_def, ball_radius=config.sph_mu_ball_r, effect="all",
                                                friction_torque_preload=config.sph_mu_trq_pr)                       
                        
                        # # 45 deg
                        # for i in range(10, 20):

                        #         if i % 2 != 0: 
                        #                 body_link_in = self.main_model.Parts.createRigidBody(name="body_link_"+ str(i) + "_in")
                        #                 bodies_inner_links.append(body_link_in)

                        #                 marker_link_ctr = bodies_inner_links[-1].Markers.create(name="MAR_LINK_IN_CTR_"+ str(i),
                        #                         location = rel_loc_custom(markers_links_out_end[-1], [0, 0, 0]),
                        #                         orientation= [40, 0, 0],
                        #                         size_of_icons=config.soi_arcs_small)
                        #                 markers_links_in_ctr.append(marker_link_ctr)

                        #                 vtorque(marker_link_ctr, bodies_outer_links[-1], markers_links_out_end[-1], i)
                                        
                        #                 marker_link_end = bodies_inner_links[-1].Markers.create(name="MAR_LINK_IN_END_"+ str(i),
                        #                         location = rel_loc_custom(marker_link_ctr, [0, config.rollers_dist, 0]),
                        #                         orientation=rel_orient_custom(marker_link_ctr, [0, 0, 0]),
                        #                         size_of_icons=config.soi_arcs_small)
                        #                 markers_links_in_end.append(marker_link_end)

                        #                 roller_circle(body_link_in, marker_link_ctr, marker_link_end, i)
                        #                 links_cm(body_link_in, marker_link_ctr)
                        #                 tabs_import(body_link_in, marker_link_ctr, config.tab_inr_ctr_loc_x, 0, i, config.color_link_inr)
                        #                 tabs_import(body_link_in, marker_link_end, config.tab_inr_end_loc_x, config.tab_inr_end_ori_z, i, config.color_link_inr)

                        #                 joint_sph = self.main_model.Constraints.createSpherical(name="JOINT_SPH_"+ str(i-1) + str(i),
                        #                         i_part=body_link_in, j_part=bodies_outer_links[-1], location=rel_loc_custom(marker_link_ctr, [0, 0, 0])) 
                                        
                        #                 joint_friction = self.main_model.Forces.createFriction(name="FR_"+joint_sph.name, joint=joint_sph, 
                        #                         formulation="original", mu_dynamic=config.sph_mu_dyn, mu_static=config.sph_mu_st, 
                        #                         stiction_transition_velocity=config.sph_mu_stv, transition_velocity_coefficient=config.sph_mu_tvc,
                        #                         max_stiction_deformation=config.sph_mu_max_st_def, ball_radius=config.sph_mu_ball_r, effect="all",
                        #                         friction_torque_preload=config.sph_mu_trq_pr)

                        #         else:
                        #                 body_link = self.main_model.Parts.createRigidBody(name="body_link_"+ str(i) + "_out")
                        #                 bodies_outer_links.append(body_link)

                        #                 marker_link_ctr = bodies_outer_links[-1].Markers.create(name="MAR_LINK_OUT_CTR_"+ str(i),
                        #                         location = rel_loc_custom(markers_links_in_end[-1], [0, 0, 0]),
                        #                         orientation=[40, 0, 0],
                        #                         size_of_icons=config.soi_arcs_small)
                        #                 markers_links_out_ctr.append(marker_link_ctr)

                        #                 vtorque(marker_link_ctr, bodies_inner_links[-1], markers_links_in_end[-1], i)

                        #                 marker_link_end = bodies_outer_links[-1].Markers.create(name="MAR_LINK_IN_END_"+ str(i),
                        #                         location = rel_loc_custom(marker_link_ctr, [0, config.rollers_dist, 0]),
                        #                         orientation=rel_orient_custom(marker_link_ctr, [0, 0, 0]),
                        #                         size_of_icons=config.soi_arcs_small)
                        #                 markers_links_out_end.append(marker_link_end)

                        #                 links_cm(body_link, marker_link_ctr)
                        #                 tabs_import(body_link, marker_link_ctr, config.tab_out_ctr_loc_x, 0, i, config.color_link_out)
                        #                 tabs_import(body_link, marker_link_end, config.tab_out_end_loc_x, config.tab_out_end_ori_z, i, config.color_link_out)

                        #                 joint_sph = self.main_model.Constraints.createSpherical(name="JOINT_SPH_"+ str(i-1) + str(i),
                        #                         i_part=body_link, j_part=bodies_inner_links[-1], location=rel_loc_custom(marker_link_ctr, [0, 0, 0])) 

                        #                 joint_friction = self.main_model.Forces.createFriction(name="FR_"+joint_sph.name, joint=joint_sph, 
                        #                         formulation="original", mu_dynamic=config.sph_mu_dyn, mu_static=config.sph_mu_st, 
                        #                         stiction_transition_velocity=config.sph_mu_stv, transition_velocity_coefficient=config.sph_mu_tvc,
                        #                         max_stiction_deformation=config.sph_mu_max_st_def, ball_radius=config.sph_mu_ball_r, effect="all",
                        #                         friction_torque_preload=config.sph_mu_trq_pr)                        
                                               
                        # last_link = 11
                        # for i in range(10, last_link):

                        #         if i % 2 != 0: 
                        #                 body_link_in = self.main_model.Parts.createRigidBody(name="body_link_"+ str(i) + "_in")
                        #                 bodies_inner_links.append(body_link_in)

                        #                 marker_link_ctr = bodies_inner_links[-1].Markers.create(name="MAR_LINK_IN_CTR_"+ str(i),
                        #                         location = rel_loc_custom(markers_links_out_end[-1], [0, 0, 0]),
                        #                         orientation= [0, 0, 0],
                        #                         size_of_icons=config.soi_arcs_small)
                        #                 markers_links_in_ctr.append(marker_link_ctr)

                        #                 vtorque(marker_link_ctr, bodies_outer_links[-1], markers_links_out_end[-1], i)
                                        
                        #                 marker_link_end = bodies_inner_links[-1].Markers.create(name="MAR_LINK_IN_END_"+ str(i),
                        #                         location = rel_loc_custom(marker_link_ctr, [0, config.rollers_dist, 0]),
                        #                         orientation=rel_orient_custom(marker_link_ctr, [0, 0, 0]),
                        #                         size_of_icons=config.soi_arcs_small)
                        #                 markers_links_in_end.append(marker_link_end)

                        #                 roller_circle(body_link_in, marker_link_ctr, marker_link_end, i)
                        #                 links_cm(body_link_in, marker_link_ctr)
                        #                 tabs_import(body_link_in, marker_link_ctr, config.tab_inr_ctr_loc_x, 0, i, config.color_link_inr)
                        #                 tabs_import(body_link_in, marker_link_end, config.tab_inr_end_loc_x, config.tab_inr_end_ori_z, i, config.color_link_inr)

                        #                 joint_sph = self.main_model.Constraints.createSpherical(name="JOINT_SPH_"+ str(i-1) + str(i),
                        #                         i_part=body_link_in, j_part=bodies_outer_links[-1], location=rel_loc_custom(marker_link_ctr, [0, 0, 0])) 
                                        
                        #                 joint_friction = self.main_model.Forces.createFriction(name="FR_"+joint_sph.name, joint=joint_sph, 
                        #                         formulation="original", mu_dynamic=config.sph_mu_dyn, mu_static=config.sph_mu_st, 
                        #                         stiction_transition_velocity=config.sph_mu_stv, transition_velocity_coefficient=config.sph_mu_tvc,
                        #                         max_stiction_deformation=config.sph_mu_max_st_def, ball_radius=config.sph_mu_ball_r, effect="all",
                        #                         friction_torque_preload=config.sph_mu_trq_pr)

                        #         else:
                        #                 body_link = self.main_model.Parts.createRigidBody(name="body_link_"+ str(i) + "_out")
                        #                 bodies_outer_links.append(body_link)

                        #                 marker_link_ctr = bodies_outer_links[-1].Markers.create(name="MAR_LINK_OUT_CTR_"+ str(i),
                        #                         location = rel_loc_custom(markers_links_in_end[-1], [0, 0, 0]),
                        #                         orientation=[0, 0, 0],
                        #                         size_of_icons=config.soi_arcs_small)
                        #                 markers_links_out_ctr.append(marker_link_ctr)

                        #                 vtorque(marker_link_ctr, bodies_inner_links[-1], markers_links_in_end[-1], i)

                        #                 marker_link_end = bodies_outer_links[-1].Markers.create(name="MAR_LINK_IN_END_"+ str(i),
                        #                         location = rel_loc_custom(marker_link_ctr, [0, config.rollers_dist, 0]),
                        #                         orientation=rel_orient_custom(marker_link_ctr, [0, 0, 0]),
                        #                         size_of_icons=config.soi_arcs_small)
                        #                 markers_links_out_end.append(marker_link_end)

                        #                 links_cm(body_link, marker_link_ctr)
                        #                 tabs_import(body_link, marker_link_ctr, config.tab_out_ctr_loc_x, 0, i, config.color_link_out)
                        #                 tabs_import(body_link, marker_link_end, config.tab_out_end_loc_x, config.tab_out_end_ori_z, i, config.color_link_out)

                        #                 joint_sph = self.main_model.Constraints.createSpherical(name="JOINT_SPH_"+ str(i-1) + str(i),
                        #                         i_part=body_link, j_part=bodies_inner_links[-1], location=rel_loc_custom(marker_link_ctr, [0, 0, 0])) 

                        #                 joint_friction = self.main_model.Forces.createFriction(name="FR_"+joint_sph.name, joint=joint_sph, 
                        #                         formulation="original", mu_dynamic=config.sph_mu_dyn, mu_static=config.sph_mu_st, 
                        #                         stiction_transition_velocity=config.sph_mu_stv, transition_velocity_coefficient=config.sph_mu_tvc,
                        #                         max_stiction_deformation=config.sph_mu_max_st_def, ball_radius=config.sph_mu_ball_r, effect="all",
                        #                         friction_torque_preload=config.sph_mu_trq_pr)  
                                
                        #         if i==last_link-1:
                        #                 joint_sph_grnd = self.main_model.Constraints.createTranslational(name="LINK_FIX_LAST", 
                        #                          i_part=body_link, j_part=self.ground,
                        #                          location=[0,0,0], relative_to=marker_link_end)  
                        #                 jprim = self.main_model.Constraints.createOrientation(name="JPRIM_END", i_marker=marker_link_end, j_marker=self.mar_ground)
                        #                 jprim_x = self.main_model.Constraints.createMotionT(name="JPRIM_END_X", joint=jprim, time_derivative="displacement", function="1*time", type_of_freedom="translational") 
                              
                        #                 pointmotion_x = self.main_model.Constraints.createPointMotion(name="PM_X", i_marker=marker_link_end, j_marker=self.mar_ground, axis="x", function="0*time")
                        #                 pointmotion_y = self.main_model.Constraints.createPointMotion(name="PM_Y", i_marker=marker_link_end, j_marker=self.mar_ground, axis="y", function="1*time")                    
                        #                 pointmotion_z = self.main_model.Constraints.createPointMotion(name="PM_Z", i_marker=marker_link_end, j_marker=self.mar_ground, axis="z", function="0*time")
                        #                 pointmotion_ax = self.main_model.Constraints.createPointMotion(name="PM_AX", i_marker=marker_link_end, j_marker=self.mar_ground, axis="b1", function="0*time")
                        #                 pointmotion_ay = self.main_model.Constraints.createPointMotion(name="PM_AY", i_marker=marker_link_end, j_marker=self.mar_ground, axis="b2", function="0*time")
                        #                 pointmotion_az = self.main_model.Constraints.createPointMotion(name="PM_AZ", i_marker=marker_link_end, j_marker=self.mar_ground, axis="b3", function="0*time")
                        # statevariable = self.main_model.DataElements.createStateVariable(name="VARIABLE_1", function="DY(MAR_LINK_OUT_CTR_10,MAR_Y_TARGET,MAR_Y_TARGET)-1")
                        # diffequation = self.main_model.SystemElements.createDifferentialEquation(name ="DIFF_1", function="-1*(DIF(.CHAIN_SIMPLE.DIFF_1)-1)", static_hold="off")
                        # general motion function for Y
                        # -100*( DY(MAR_LINK_OUT_CTR_10,MAR_Y_TARGET,MAR_Y_TARGET)-(0-DIF(.CHAIN_SIMPLE.DIFF_1))) * time 
        
        def measurements(self):
                measure_marker = self.main_model.Measures.createFunction(name= "MAR_DISP", function="DZ(MAR_GROUND, MAR_LINK_IN_END_15, MAR_GROUND)", units="mm", create_measure_display="no")
                measure_marker = self.main_model.Measures.createFunction(name= "MAR_ANG", function="AZ(MAR_LINK_1_CTR, MAR_LINK_IN_END_15)", units="degree", create_measure_display="no")
                  
def prep_data():
        data = np.loadtxt("src/data/tracked_data.txt", skiprows=1)  # Skip first row (header)
        y_positions = data[:, 2]  # Y position in pixels
        y_positions = y_positions[-55:]#[-70:] 
        y_positions = np.multiply(y_positions, -0.145)
        y_positions = np.add(y_positions, 64)
        kernel_size = 2
        kernel_z = np.ones(kernel_size) / kernel_size
        y_positions_f = np.convolve(y_positions, kernel_z, mode='valid') 
        return y_positions_f

def visualize():
        from plotly.subplots import make_subplots
        import plotly.graph_objects as go
        set = load_plot_settings()
        model =  Adams.stoo(".CHAIN_SIMPLE")
        analysis = model.Analyses["Last_Run"]
        # val = analysis.results["CONTACT_data"]["theNormalForce"].values
        # val2 = analysis.results["CONTACT_data"]["theCPortion"].values
        cpu_mea = analysis.results["CPU_MEA"]["Q"].values
        mar_disp_y = analysis.results["MAR_DISP"]["Q"].values
        mar_ang_z = analysis.results["MAR_ANG"]["Q"].values
        res_time = analysis.results["TIME"].values

        mar_disp_y = mar_disp_y[:69]
        mar_disp_y = np.multiply(mar_disp_y, -1)

        # Read the data file
        # df = pd.read_csv("src/data/tracked_data.txt", sep="\t")
        data = np.loadtxt("src/data/tracked_data.txt", skiprows=1)  # Skip first row (header)

        # Extract columns into separate lists
        time = data[:, 0]  # Time in seconds
        time = np.multiply(time, 0.129)#00.129)
        x_positions = data[:, 1]  # X position in pixels
        y_positions = data[:, 2]  # Y position in pixels
        y_positions = y_positions[-70:] 
        y_positions = np.multiply(y_positions, -0.145)
        y_positions = np.add(y_positions, 64)
        # # Convert columns to lists
        # tracked_times = df["Time(s)"].tolist()
        # y_positions = df["Y(pixels)"].tolist()

        kernel_size = 2
        kernel_z = np.ones(kernel_size) / kernel_size
        y_positions_f = np.convolve(y_positions, kernel_z, mode='valid')
        
        x = res_time
        fig1 = make_subplots(rows=1, cols=1, shared_xaxes=True)
        fig1.add_trace(go.Scatter(x=x, y=mar_disp_y,
                marker=dict(color=set.TUMOrange), name='<span style="font: Arial; font-size: 11pt"> Modell',
                legendgroup='1'), row=1, col=1)
        fig1.add_trace(go.Scatter(x=time, y=y_positions_f,
                marker=dict(color=set.TUMBlue), name='<span style="font: Arial; font-size: 11pt"> Versuch', legendgroup='1'), row=1, col=1)

        #'> <i>F</i><sub>YT,f</sub> <'
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
        fig1.write_image("fig1.jpg", width=642, height=300, scale=5)


def visualize_t1():
        from plotly.subplots import make_subplots
        import plotly.graph_objects as go
        set = load_plot_settings()
        model =  Adams.stoo(".CHAIN_SIMPLE")
        analysis = model.Analyses["Last_Run"]
        cpu_mea = analysis.results["CPU_MEA"]["Q"].values
        mar_disp_y = analysis.results["FUNCTION_MEA_1"]["Q"].values
        res_time = analysis.results["TIME"].values

        mar_disp_y = mar_disp_y
        mar_disp_y = np.multiply(mar_disp_y, -1)

        # Read the data file
        # df = pd.read_csv("src/data/tracked_data.txt", sep="\t")
        data = np.loadtxt("src/data/tracked_data.txt", skiprows=1)  # Skip first row (header)

        x = res_time
        fig1 = make_subplots(rows=1, cols=1, shared_xaxes=True)
        fig1.add_trace(go.Scatter(x=x, y=mar_disp_y,
                marker=dict(color=set.TUMOrange), name='<span style="font: Arial; font-size: 11pt"> Text',
                legendgroup='1'), row=1, col=1)

        fig1.add_vline(x=11.3, line_width=1, line_dash="dash", line_color="black", annotation_text='<span style="font-family: Cambria; font-size: 10pt" > 11.3 ', annotation_position="bottom")
        fig1.add_hline(y=-45, line_width=1, line_dash="dash", line_color="black", annotation_text='<span style="font-family: Cambria; font-size: 10pt" > -45 ', annotation_position="left")

        #'> <i>F</i><sub>YT,f</sub> <'
        fig1.update_layout(
                yaxis_title=dict(text='<span style="font-family: Arial; font-size: 11pt"'
                                '> Winkel  <i>AX</i> <' # type here
                                '/span>'
                                '<span style="font-family: Arial; font-size: 11pt"'
                                '> in <' # type here
                                '/span>'
                                '<span style="font-family: Sitka Heading; font-size: 13pt"'
                                '> ° <' # type here
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
                legend=dict(x=1.2, y=1, xanchor='right', yanchor='top', font=dict(family="Arial", size=29),itemsizing='constant'),
                legend_bordercolor='#000000',
                legend_borderwidth=0,
                font_color="black",
                legend_tracegroupgap=360
        )
        fig1.update_layout(autosize=False, width=450, height=400)
        fig1.show()
        fig1.write_image("fig1.jpg", width=450, height=400, scale=5)

def tsbo_cost(data):
        tsbo_input_file = 'src/matlab/TSBO/input.txt'
        model =  Adams.stoo(".CHAIN_SIMPLE") 
        analysis = model.Analyses["Last_Run"]
        mar_disp_y = analysis.results["MAR_DISP"]["Q"].values
        mar_disp_y = mar_disp_y[15:69]
        mar_disp_y = np.multiply(mar_disp_y, -1)
        y_positions_f = data

        def RMSE(y, y_pred):
                y_arr = np.array(y)
                y_pred_arr = np.array(y_pred)
                mse = ((y_arr - y_pred_arr) ** 2).mean()
                return np.sqrt(mse)
        rmse = RMSE(mar_disp_y, y_positions_f)
        emax = max(abs(np.subtract(mar_disp_y, y_positions_f)))
        cost = rmse #+ emax
        print(rmse)
        print(emax)
        with open(tsbo_input_file, 'a') as file:
                file.write('\n')
                file.write(str(cost))

def simulate():
        import time
        model =  Adams.stoo(".CHAIN_SIMPLE") 
        simulation = model.Simulations.create(name="SIM1", duration=0.4, number_of_steps=90, type="dynamic")

        Adams.read_command_file("src/modeling/cmd_files/verify.cmd")
        # simulation.simulate()
        data = prep_data()
        # tsbo_cost(data)

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

                        bound_upr =  Adams.stoo(".CHAIN_SIMPLE.bistop_bound_upr") 
                        stiffness_k =  Adams.stoo(".CHAIN_SIMPLE.bistop_stiffness_k")
                        damping_c =  Adams.stoo(".CHAIN_SIMPLE.bistop_max_damping_cmax")
                        fexp_e = Adams.stoo(".CHAIN_SIMPLE.bistop_force_deform_exp_e")

                        output_file = 'src/matlab/TSBO/output.txt'
                        with open(output_file, "r") as file:
                                lines = file.readlines()
                                actual_vals = lines[-1]
                                bound_upr_new=float(actual_vals.split()[0])
                                stiffness_k_new=float(actual_vals.split()[1])
                                damping_c_new=float(actual_vals.split()[2])
                                fexp_e_new=float(actual_vals.split()[3])

                        bound_upr.value = bound_upr_new    
                        stiffness_k.value = stiffness_k_new
                        damping_c.value = damping_c_new
                        fexp_e.value = fexp_e_new

                        simulation.simulate()
                        tsbo_cost(data)
                          # Update the previous line count
                        previous_num_lines = current_num_lines

        # Pause for a short interval to avoid excessive CPU usage
        time.sleep(1)

# def simulate():            
#         model =  Adams.stoo(".CHAIN_SIMPLE")                            
#         simulation = model.Simulations.create(name="SIM1", duration=1, number_of_steps=500)
#         simulation.simulate()
#         # self.visualize()   
    


model = MODEL()
model.chain()
# model.measurements()

Adams.read_command_file("src/modeling/cmd_files/verify.cmd")

# simulate()

# visualize()

# bound_upr =  Adams.stoo(".CHAIN_SIMPLE.bistop_bound_upr") 
# print(bound_upr.value)
        
# data = prep_data()        
# tsbo_cost(data)
