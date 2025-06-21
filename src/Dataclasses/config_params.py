from dataclasses import dataclass
from typing import List

@dataclass
class config():
    # Settings
    #
    ### APPEARENCE ###
    #
    # MARKERS #
    soi_model: float
    soi_arcs_small: float
    #
    #
    ### FITTING ###
    sprocket_upr_ori_x: float
    sprocket_upr_roll_r: float
    links_init_angle: float

    # CASSETTE
    cassette_ori: List[float]
    cassette_loc: List[float]
    cassette_rad: float
    cassette_pitch_deg: float

    # SPROCKET MAIN
    sprocket_main_ori_x: float
    sprocket_main_ori_z: float
    sprocket_main_loc: List[float]
    sprocket_main_rad: float
    sprocket_main_pitch_deg: float

    # SPROCKET UPPER
    sprocket_upr_loc: List[float]
    sprocket_upr_ori: List[float]
    sprocket_upr_pitch_deg: float

    # DERAILLEUR STATIC PART
    # parameters for MAR_DER_STAT_CENTER:
    der_stat_ori: List[float]
    der_stat_loc: List[float]
    der_stat_cm: List[float]
    der_stat_inertia: List[float]

    # LINK INNER
    # parameters for MAR_LINK_INR_CENTER:
    link_inr_ori: List[float]
    link_inr_loc: List[float]
    link_inr_cm: List[float]
    link_inr_inertia: List[float]

    # FOUR-BAR LINKAGE PARAMETERS
    fbar_input_init: float
    fbar_output_init: float
    fbar_input: float
    fbar_AB: float
    fbar_BC: float
    fbar_CD: float
    fbar_AD: float

    # LINK OUTER
    # parameters for MAR_LINK_OUT_CENTER:
    link_out_ori: List[float]
    link_out_loc: List[float]
    link_out_cm: List[float]
    link_out_inertia: List[float]

    # KNUCKLE
    # parameters for MAR_KNUCKLE_CENTER:
    knuckle_ori: List[float]
    knuckle_loc: List[float]
    knuckle_cm: List[float]
    knuckle_inertia: List[float]

    # DERAILLEUR CAGE
    # parameters for MAR_DER_CAGE_CENTER:
    der_cage_ori: List[float]
    der_cage_loc: List[float]
    der_cage_cm: List[float]
    der_cage_inertia: List[float]

    # JOCKEY UPPER
    jockey_upr_ori_lcl: List[float]
    jockey_upr_loc_lcl: List[float]
    jockey_upr_ori_gl: List[float]
    jockey_upr_loc_gl: List[float]
    jockey_upr_cm: List[float]
    jockey_upr_inertia: List[float]
    jockey_upr_rad: float
    jockey_upr_pitch_deg: float
    jockey_upr_2_kn_len: float
    jockey_upr_2_kn_deg: float

    # JOCKEY LOWER
    # parameters for MAR_JOCKEY_LWR_CENTER:
    jockey_lwr_ori_lcl: List[float]
    jockey_lwr_loc_lcl: List[float]
    jockey_lwr_ori_gl: List[float]
    jockey_lwr_loc_gl: List[float]
    jockey_lwr_cm: List[float]
    jockey_lwr_inertia: List[float]
    jockey_lwr_rad: float
    jockey_lwr_pitch_deg: float

    # SUSPENSION 
    rocker_rear_loc: List[float]
    rocker_mid_loc: List[float]
    rocker_front_loc: List[float]
    damper_lwr_loc: List[float]
    link_lwr_front_loc: List[float]
    link_lwr_rear_loc: List[float]
    front_axle_loc: List[float]
    logger_loc: List[float]
    logger_ori: List[float]

    # CHAIN 
    # spherical joints friction parameters 
    sph_mu_dyn: float
    sph_mu_st: float
    sph_mu_stv: float
    sph_mu_tvc: float
    sph_mu_max_st_def: float
    sph_mu_ball_r: float
    sph_mu_trq_pr: float
    # parameters and fitting for chain
    roll_cyl_len: float
    roll_radius: float
    # angles
    links_ori_x_2: float
    links_ori_x_3: float
    links_ori_4: float
    links_ori_5: float
    links_ori_6: float
    links_ori_7: float
    links_ori_8: float
    links_ori_9: float
    links_ori_10: float


    #
    ### KEY PARAMETERS ###
    rollers_dist: float
    tab_inr_ctr_loc_x: float
    tab_inr_end_loc_x: float
    tab_inr_end_ori_z: float

    tab_out_ctr_loc_x: float
    tab_out_end_loc_x: float
    tab_out_end_ori_z: float

    
    color_link_inr: str
    color_link_out: str

@dataclass
class plot_settings():
    TUMBlue: str
    TUMOrange: str
    TUMGreen: str
    TUMGray2: str



