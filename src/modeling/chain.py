import Adams 
import numpy as np 
from src.utils.datamanager import load_config
from src.utils.custom_tools import rel_loc_custom, rel_orient_custom, loc_mirror_zx

config = load_config()

def chain_small(model, body1, marker_center):

    body_roll = model.Parts.createRigidBody(name="body_roll")
    
    # mar_chain_ctr = body_roll.Markers.create(name="MAR_CHAIN_CENTER", 
    #                     location=[0,0,0], orientation=[0,0,0])
    mar_roll_cm = body_roll.Markers.create(name="MAR_ROLL_CM",
                location =[50, 0, 0], orientation=[-90, 90, 90])
    body_roll.cm = mar_roll_cm
    body_roll.mass = 0.005


    body_roll_new = model.Parts.createRigidBody(name="body_roll_new")
    
    mar_roll_cm_new = body_roll_new.Markers.create(name="MAR_ROLL_CM_new", 
                location = rel_loc_custom(mar_roll_cm, [0, 10, 0]),
                orientation=[0,0,0])
    body_roll_new.cm = mar_roll_cm_new
    body_roll_new.mass = 0.005
    

    # mar_curve_ctr = body_sprocket.Markers.create(name="MAR_CURVE_DATA",
    #     location = rel_loc_custom(mar_sprocket_ctr, [0, 0, 0]),
    #     orientation = rel_orient_custom(mar_sprocket_ctr, [-90, 90, 90]),
    #     size_of_icons = config.soi_arcs_small)

    # mar_chain_0 = body_roll.Markers.create(name="MAR_CHAIN_0")
    # mar_circ.location = rel_loc_custom(mar_chain_ctr, [0,-32.6,0])
    # mar_circ.orientation = rel_orient_custom(mar_chain_ctr,[-90, 90, 90])
    # mar_circ.size_of_icons = config.soi_arcs_small

    # circle_object = body_roll.Geometries.createCircle(name="ROLL1", 
    #         center_marker=mar_circ, radius=3.965) 
    