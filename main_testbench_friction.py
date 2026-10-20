# import imp
import importlib
import sys
import os
import Adams
import math as m
import numpy as np
import pandas as pd
import scipy as sci
# from scipy.integrate import cumulative_trapezoid
import toml

abs_work_dir_path = os.path.abspath(os.curdir)
sys.path.append(abs_work_dir_path)  

from src.utils.datamanager import load_config, load_plot_settings
from src.utils.custom_tools import rel_loc_custom, rel_orient_custom, loc_mirror_zx, ori_along_axis, ori_one_axis
import src.modeling.frame as frame_module
import src.modeling.chain_2d as chain_2d_module
#
class MODEL():

    def model_base(self):
        self.config = load_config()
        # MODEL BASE
        self.model_name = 'TEST_FRICTION_0KG'
        Adams.defaults.units.setUnits(length='mm',mass='kg',time = 'second',angle='degrees',force='newton', frequency='hz')
        Adams.execute_cmd("default unit orient=body123") # changes the rotation sequence from default 313 to 123
        self.main_model = Adams.Models.create(name=self.model_name)
        self.main_model.size_of_icons = self.config.soi_model
        self.ground = self.main_model.ground_part
        grav_angle_deg = 90
        g = 9.81
        grav_yz = [g*10e2*m.cos(m.radians(grav_angle_deg)), -g*10e2*m.sin(m.radians(grav_angle_deg))]
        grav = self.main_model.Forces.createGravity(name='gravity', xyz_component_gravity = [0, round(grav_yz[0],0), round(grav_yz[1],0)])
        print('GRAVITY_Y=' + str(grav_yz[0]))
        print('GRAVITY_Z=' + str(grav_yz[1]))
        # gravity = self.main_model.Forces.createGravity(name='gravity', xyz_component_gravity = [0, 0, -9807])
        # gravity = self.main_model.Forces.createGravity(name='gravity', xyz_component_gravity = [0, 0, 0])
        self.mar_ground = self.ground.Markers.create(name="MAR_GROUND")

        measure_cpu = self.main_model.Measures.createFunction(name= "CPU_MEA", function="CPU", units="no_units", create_measure_display="no")
        # Adams.read_command_file("src/modeling/cmd_files/callback.cmd")
        ### STARTER PACK ###
        self.counter_solids = 0
        self.solids = []
        self.contacts_2d = []
        ### SOLVER SETTINGS ###
        s=Adams.defaults.model.settings.solver
        i=Adams.defaults.model.settings.integrator
        s.threads=4
        i.type = "gstiff"
        i.formulation = "si2"
        i.error = 0.01
        Adams.defaults.model.settings.contact.faceting_tolerance=600 
        self.parasolids_path = 'src/parasolids/'
        ###
        ###
        ### PENDULUM PARAMETERS FOR 0KG, 1KG and 2KG:
        self.pendulum_length = 308.9 # in mm, from CAD
        self.dist_tracking_point = 69.823 # distance from pendulum end (from Video)
        #
        ### 0 kg
        self.start_angle_deg = 11.33 # match the first amplitude in video
        self.pendulum_mass = 0.238
        self.pendulum_cog_location = self.pendulum_length - 190.2 # CAD pendulum length - CAD distance revolute joint to cog
        self.pendulum_inertia = [2.0392e+3, 2.0278e+3, 68.38, 0,0,0] # inertia at COG from CAD; inertia frame in CAD has different orientation
        #
        ### 1 kg
        # self.start_angle_deg = 11.33 # match the first amplitude in video
        # self.pendulum_mass = 1.245
        # self.pendulum_cog_location = self.pendulum_length - 286.3 # CAD pendulum length - CAD distance revolute joint to cog
        # self.pendulum_inertia = [6.1138e+3, 5.4943e+3, 8.0992e+02, 0,0,0] # 1 kg
        #
        # ### 2 kg
        # self.start_angle_deg = 11.05 # match the first amplitude in video
        # self.pendulum_mass = 2.287
        # self.pendulum_cog_location = self.pendulum_length - 248.9 # CAD pendulum length - CAD distance revolute joint to cog
        # self.pendulum_inertia = [1.13368e+4, 1.0093e+4, 1.5583e+03, 0,0,0] # inertia at COG from CAD; inertia frame in CAD has different orientation
        #



# PREPARE TESTBENCH FRICTION
import src.modeling.testbench_friction as testbench_friction_module
importlib.reload(testbench_friction_module)

if __name__ == "__main__": 
    tb = testbench_friction_module.TESTBENCH_FR()
    tb.build_chain()
    #
    ### AFTERWORK
    Adams.read_command_file("src/modeling/cmd_files/verify.cmd") # VERIFY MODEL
    Adams.read_command_file("src/modeling/cmd_files/set_view.cmd") # SET VIEW
    # Adams.read_command_file("src/modeling/cmd_files/rotate_view.cmd") # SET VIEW

    ### UNCOMMENT SECTION TO TRANSLATE AND ROTATE THE WHOLE MODEL
    # importlib.reload(place_model_module)
    # place_model_module.MODEL_R()
    # place_model_module.MODEL_R.merge_and_clean(model2rotate=model.main_model, translation=[0,500,500], rotation=[-42,0,0])
    ###

    print('___!MODEL IS READY!___')
