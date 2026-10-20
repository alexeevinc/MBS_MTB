# import imp
import importlib
from pathlib import Path
import sys
import os
import Adams
import math as m
import numpy as np
import pandas as pd
import scipy as sci

# from scipy.integrate import cumulative_trapezoid
# abspath = 'C:/1_A/HSPF/repo_1'
# abspath = 'D:/HSPF/repo_1'
# abs_work_dir_path = os.path.abspath(os.curdir)
# sys.path.append(abs_work_dir_path)  

PROJECT_ROOT = Path.cwd()
os.chdir(PROJECT_ROOT)
sys.path.insert(0, str(PROJECT_ROOT))

from src.utils.datamanager import load_config
from src.utils.custom_tools import add_gui_cabi
import src.utils.prep_input_data as prep_input_module
import src.utils.place_model as place_model_module 
import src.modeling.frame as frame_module
import src.modeling.chain_2d as chain_2d_module
import src.modeling.suspension as suspension_module
import src.modeling.sprockets as sprockets_module
import src.modeling.derailleur as derailleur_module

config = load_config()

#
class MODEL():
    def __init__(self):
        ### STARTER PACK ###
        self.counter_solids = 0
        self.solids = []
        self.contacts_2d = []
        # model base
        self.model_name = 'CHAIN_DRIVE_G' + str(config.cassette_gear) 
        Adams.defaults.units.setUnits(length='mm',mass='kg',time = 'second', angle='degrees',force='newton', frequency='hz')
        Adams.execute_cmd("default unit orient=body123") # changes the rotation sequence from default 313 to 123
        self.main_model = Adams.Models.create(name=self.model_name)
        self.main_model.size_of_icons = config.soi_model
        self.ground = self.main_model.ground_part
        grav_angle_deg = 37.5#52
        g = 9.81
        grav_yz = [g*10e2*m.cos(m.radians(grav_angle_deg)), -g*10e2*m.sin(m.radians(grav_angle_deg))]
        grav = self.main_model.Forces.createGravity(name='gravity', xyz_component_gravity = [0, round(grav_yz[0],0), round(grav_yz[1],0)])
        print('GRAVITY_Y=' + str(grav_yz[0]))
        print('GRAVITY_Z=' + str(grav_yz[1]))
        # gravity = self.main_model.Forces.createGravity(name='gravity', xyz_component_gravity = [0, 0, -9807])
        # gravity = self.main_model.Forces.createGravity(name='gravity', xyz_component_gravity = [0, 0, 0])
        self.mar_ground = self.ground.Markers.create(name="MAR_GROUND")

        measure_cpu = self.main_model.Measures.createFunction(name="CPU_MEA", function="CPU", units="no_units", create_measure_display="no")
        # Adams.read_command_file("src/modeling/cmd_files/callback.cmd")

        ### SOLVER SETTINGS ###
        s=Adams.defaults.model.settings.solver
        i=Adams.defaults.model.settings.integrator
        i.type = "gstiff"
        i.formulation = "si2"
        i.error = 0.01
        ### SIMULATION SETTINGS ###
        adm=Adams.defaults.model
        adm.Simulations.mo = 2
        adm.Simulations.end_time = 2
        adm.Simulations.step_size = 0.001

        ### OTHER SETTINGS ###
        Adams.defaults.model.settings.contact.faceting_tolerance=600 
        self.parasolids_path = 'src/parasolids/'

        ### ITS EASIER TO SWITCH TO CMD FOR THERE SETTINGS ###
        Adams.execute_cmd('.gui.sim_int_panel.c_interactive.number_of_steps.preload_strings = "2000"')
        Adams.execute_cmd('.gui.sim_int_panel.c_interactive.end_time.preload_strings = "2"')

        ### PREPARED INPUT DATA ###
        importlib.reload(prep_input_module)
        self.data_vtrack, self.data_logger, self.vtrack_scaling, self.logger_scaling = prep_input_module.prep_data()


model = MODEL()
#
# BUILD FRAME
importlib.reload(frame_module)
frame_module.build_frame(model, config)
#
# BUILD SUSPENSION
importlib.reload(suspension_module)
suspension_module.build_rocker(model, config)
suspension_module.build_sus_link_lwr(model, config)
suspension_module.build_sus_link_upr(model, config)
suspension_module.build_damper_cylinder(model, config)
suspension_module.build_damper_piston(model, config)
#
# BUILD SPROCKETS
importlib.reload(sprockets_module)
sprockets_module.set_gear(model, config)
sprockets_module.build_sprocket_main(model, config)
sprockets_module.build_sprocket_upr(model, config)
sprockets_module.build_cassette(model, config)
#
# BUILD DERAILLEUR
importlib.reload(derailleur_module)
derailleur_module.build_derailleur_stat(model, config)
derailleur_module.build_knuckle(model, config)
derailleur_module.build_der_cage(model, config)
#
# BUILD JOCKEYS
sprockets_module.build_jockey_upr(model, config)
sprockets_module.build_jockey_lwr(model, config)
#
# BUILD CHAIN
importlib.reload(chain_2d_module)
chain_2d_module.build_chain(model, config)
#
### AFTERWORK
Adams.read_command_file("src/modeling/cmd_files/verify.cmd") # VERIFY MODEL
Adams.read_command_file("src/modeling/cmd_files/set_view.cmd") # SET VIEW
# Adams.read_command_file("src/modeling/cmd_files/rotate_view.cmd") # SET VIEW

### UNCOMMENT SECTION TO TRANSLATE AND ROTATE THE WHOLE MODEL
importlib.reload(place_model_module)
place_model_module.MODEL_R()
place_model_module.MODEL_R.merge_and_clean(model2rotate=model.main_model, translation=[0,500,500], rotation=[-37.5,0,0])
###
###t
### ADD CALC BIKE TAB
# add_gui_cabi(PROJECT_ROOT)
#
Adams.execute_cmd(f"executive set numerical model = .{model.model_name} corrector = modified")
Adams.execute_cmd(f"executive set numerical model = .{model.model_name} hmax = 0.01")
Adams.execute_cmd("executive_control set preferences thread_count = 14")
Adams.execute_cmd("simulation set save_files = yes")
Adams.execute_cmd(f"output set output model = .{model.model_name} grsave = off")
Adams.execute_cmd(f"output set output model = .{model.model_name} reqsave = off")
Adams.execute_cmd(f"output set output model = .{model.model_name} print = off")
Adams.execute_cmd(f"output set output model = .{model.model_name} separator = off")
Adams.execute_cmd('simulation set file_prefix = "G7_MG_test"')

# Adams.execute_cmd('marker modify marker_name = .CHAIN_DRIVE_G7.body_jockey_upr.MAR_JOCKEY_UPR_CENTER_LCL orientation = (ORI_RELATIVE_TO({0, -7.7827914421-0.2, 0}, MAR_JOCKEY_UPR_CENTER_GL))')
