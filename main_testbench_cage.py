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
from src.utils.prep_input_data import prep_data
import src.utils.place_model as place_model_module
import src.modeling.frame as frame_module
import src.modeling.chain_2d as chain_2d_module
import src.modeling.suspension as suspension_module
import src.modeling.sprockets as sprockets_module
import src.modeling.derailleur as derailleur_module
import src.modeling.testbench_cage as testbench_cage_module

config = load_config()
#
class MODEL():
    def __init__(self):
        ### STARTER PACK ###
        self.counter_solids = 0
        self.solids = []
        self.contacts_2d = []
        # MODEL BASE
        self.model_name = 'TEST_CAGE'
        Adams.defaults.units.setUnits(length='mm',mass='kg',time = 'second',angle='degrees',force='newton', frequency='hz')
        Adams.execute_cmd("default unit orient=body123") # changes the rotation sequence from default 313 to 123
        self.main_model = Adams.Models.create(name=self.model_name)
        self.main_model.size_of_icons = config.soi_model
        self.ground = self.main_model.ground_part
        grav_angle_deg = 42#52
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

        ### SOLVER SETTINGS ###
        s=Adams.defaults.model.settings.solver
        i=Adams.defaults.model.settings.integrator
        s.threads=4
        i.type = "gstiff"
        i.formulation = "si2"
        i.error = 0.01
        Adams.defaults.model.settings.contact.faceting_tolerance=600 
        self.parasolids_path = 'src/parasolids/'
        
        ### PREPARED INPUT DATA ###
        # self.data_vtrack, self.data_logger, self.vtrack_scaling, self.logger_scaling = prep_data()


model = MODEL()
# SET GEAR
importlib.reload(sprockets_module)
sprockets_module.set_gear(model, config)
# PREPARE TESTBENCH CAGE
importlib.reload(testbench_cage_module)
testbench_cage_module.prep_testbench_cage(model, config) # shifts cage knuckle to origin
testbench_cage_module.build_testbench_cage(model, config) # builds an isolated derailleur
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
