import Adams 
import numpy as np 
from src.utils.datamanager import load_config
from src.utils.custom_tools import rel_loc_custom, rel_orient_custom, loc_mirror_zx
config = load_config()

def curve_tooth_small(body1, marker_center):

    mar_arc1_ctr = body1.Markers.create(name="MAR_ARC1_CTR")
    mar_arc1_ctr.location = rel_loc_custom(mar_curve_ctr, [0,0, 0])
    mar_arc1_ctr.orientation = rel_orient_custom(mar_curve_ctr, [10.927,0,0])
    mar_arc1_ctr.size_of_icons = config.soi_arcs_small

    arc1 = body1.Geometries.createArc(name= "A1", center_marker=mar_arc1_ctr) 
    arc1.radius = 28.5
    arc1.angle_extent = 0.65
    
    mar_arc12_ctr = body1.Markers.create(name="MAR_ARC12_CTR")
    mar_arc12_ctr.location = rel_loc_custom(mar_arc1_ctr, [0, 0, 0])
    mar_arc12_ctr.orientation = rel_orient_custom(mar_arc1_ctr, [0.65,0,0])
    mar_arc12_ctr.size_of_icons = config.soi_arcs_small

    mar_arc2_ctr = body1.Markers.create(name="MAR_ARC2_CTR")
    mar_arc2_ctr.location = rel_loc_custom(mar_arc12_ctr, [32.25, 0, 0])
    mar_arc2_ctr.orientation = rel_orient_custom(mar_arc12_ctr, [180,0,0])
    mar_arc2_ctr.size_of_icons = config.soi_arcs_small

    arc2 = body1.Geometries.createArc(name= "A2", center_marker=mar_arc2_ctr) 
    arc2.radius = 3.75
    arc2.angle_extent = -49.1

    mar_arc23_ctr = body1.Markers.create(name="MAR_ARC23_CTR")
    mar_arc23_ctr.location = rel_loc_custom(mar_arc2_ctr, [0, 0, 0])
    mar_arc23_ctr.orientation = rel_orient_custom(mar_arc2_ctr, [-49.1,0,0])
    mar_arc23_ctr.size_of_icons = config.soi_arcs_small

    mar_arc3_ctr = body1.Markers.create(name="MAR_ARC3_CTR")
    mar_arc3_ctr.location = rel_loc_custom(mar_arc23_ctr, [4.75, 0, 0])
    mar_arc3_ctr.orientation = rel_orient_custom(mar_arc23_ctr, [180,0,0])
    mar_arc3_ctr.size_of_icons = config.soi_arcs_small

    arc3 = body1.Geometries.createArc(name= "A3", center_marker=mar_arc3_ctr) 
    arc3.radius = 1
    arc3.angle_extent = 20.06

    mar_arc34_ctr = body1.Markers.create(name="MAR_ARC34_CTR")
    mar_arc34_ctr.location = rel_loc_custom(mar_arc3_ctr, [0, 0, 0])
    mar_arc34_ctr.orientation = rel_orient_custom(mar_arc3_ctr, [20.06,0,0])
    mar_arc34_ctr.size_of_icons = config.soi_arcs_small

    mar_arc4_ctr = body1.Markers.create(name="MAR_ARC4_CTR")
    mar_arc4_ctr.location = rel_loc_custom(mar_arc34_ctr, [4.11, 0, 0])
    mar_arc4_ctr.orientation = rel_orient_custom(mar_arc34_ctr, [180,0,0])
    mar_arc4_ctr.size_of_icons = config.soi_arcs_small

    arc4 = body1.Geometries.createArc(name= "A4", center_marker=mar_arc4_ctr) 
    arc4.radius = 3.11
    arc4.angle_extent = -49.41

    mar_arc45_ctr = body1.Markers.create(name="MAR_ARC45_CTR")
    mar_arc45_ctr.location = rel_loc_custom(mar_arc4_ctr, [0, 0, 0])
    mar_arc45_ctr.orientation = rel_orient_custom(mar_arc4_ctr, [-49.41,0,0])
    mar_arc45_ctr.size_of_icons = config.soi_arcs_small

    mar_arc5_ctr = body1.Markers.create(name="MAR_ARC5_CTR")
    mar_arc5_ctr.location = rel_loc_custom(mar_arc45_ctr, [7.03, 0, 0])
    mar_arc5_ctr.orientation = rel_orient_custom(mar_arc45_ctr, [180,0,0])
    mar_arc5_ctr.size_of_icons = config.soi_arcs_small

    arc5 = body1.Geometries.createArc(name= "A5", center_marker=mar_arc5_ctr) 
    arc5.radius = 3.92
    arc5.angle_extent = 53.73

    mar_arc6_ctr = body1.Markers.create(name="MAR_ARC6_CTR")
    mar_arc6_ctr.location = rel_loc_custom(mar_arc1_ctr, [32.25,0, 0])
    mar_arc6_ctr.orientation = rel_orient_custom(mar_arc1_ctr, [180,0,0])
    mar_arc6_ctr.size_of_icons = config.soi_arcs_small

    arc6 = body1.Geometries.createArc(name= "A6", center_marker=mar_arc6_ctr) 
    arc6.radius = 3.75
    arc6.angle_extent = 61.4

    mar_arc67_ctr = body1.Markers.create(name="MAR_ARC67_CTR")
    mar_arc67_ctr.location = rel_loc_custom(mar_arc6_ctr, [0, 0, 0])
    mar_arc67_ctr.orientation = rel_orient_custom(mar_arc6_ctr, [61.4,0,0])
    mar_arc67_ctr.size_of_icons = config.soi_arcs_small

    mar_arc7_ctr = body1.Markers.create(name="MAR_ARC7_CTR")
    mar_arc7_ctr.location = rel_loc_custom(mar_arc67_ctr, [-5.5, 0, 0])
    mar_arc7_ctr.orientation = rel_orient_custom(mar_arc67_ctr, [0,0,0])
    mar_arc7_ctr.size_of_icons = config.soi_arcs_small

    arc7 = body1.Geometries.createArc(name= "A7", center_marker=mar_arc7_ctr) 
    arc7.radius = 9.25
    arc7.angle_extent = 13.58

    mar_arc78_ctr = body1.Markers.create(name="MAR_ARC78_CTR")
    mar_arc78_ctr.location = rel_loc_custom(mar_arc7_ctr, [0, 0, 0])
    mar_arc78_ctr.orientation = rel_orient_custom(mar_arc7_ctr, [13.58,0,0])
    mar_arc78_ctr.size_of_icons = config.soi_arcs_small

    mar_arc8_ctr = body1.Markers.create(name="MAR_ARC8_CTR")
    mar_arc8_ctr.location = rel_loc_custom(mar_arc78_ctr, [17.25, 0, 0])
    mar_arc8_ctr.orientation = rel_orient_custom(mar_arc78_ctr, [180,0,0])
    mar_arc8_ctr.size_of_icons = config.soi_arcs_small

    arc8 = body1.Geometries.createArc(name= "A8", center_marker=mar_arc8_ctr) 
    arc8.radius = 8
    arc8.angle_extent = -19.62

    mar_arc89_ctr = body1.Markers.create(name="MAR_ARC89_CTR")
    mar_arc89_ctr.location = rel_loc_custom(mar_arc8_ctr, [0, 0, 0])
    mar_arc89_ctr.orientation = rel_orient_custom(mar_arc8_ctr, [-19.62,0,0])
    mar_arc89_ctr.size_of_icons = config.soi_arcs_small

    mar_arc9_ctr = body1.Markers.create(name="MAR_ARC9_CTR")
    mar_arc9_ctr.location = rel_loc_custom(mar_arc89_ctr, [7.65, 0, 0])
    mar_arc9_ctr.orientation = rel_orient_custom(mar_arc89_ctr, [0,0,0])
    mar_arc9_ctr.size_of_icons = config.soi_arcs_small

    arc9 = body1.Geometries.createArc(name= "A9", center_marker=mar_arc9_ctr) 
    arc9.radius = 0.35
    arc9.angle_extent = -64.41

    ### ARCS ARRAY ####

    mar_poly_small1 = body1.Markers.create(name="MAR_POLY_SMALL1")
    mar_poly_small1.orientation = rel_orient_custom(mar_arc5_ctr, [0,0,53.73])
    mar_poly_small1.location = rel_loc_custom(mar_arc5_ctr, [0, 0, 0])
    mar_poly_small1.size_of_icons = config.soi_arcs_small


    mar_poly_small2 = body1.Markers.create(name="MAR_POLY_SMALL2")
    mar_poly_small2.orientation = rel_orient_custom(mar_poly_small1, [0,0,0])
    mar_poly_small2.location = rel_loc_custom(mar_poly_small1, [3.92, 0, 0])
    mar_poly_small2.size_of_icons = config.soi_arcs_small


    mar_poly_small3 = body1.Markers.create(name="MAR_POLY_SMALL3")
    mar_poly_small3.orientation = rel_orient_custom(mar_poly_small2, [-53.56,0,0])
    mar_poly_small3.location = rel_loc_custom(mar_poly_small2, [0, 0, 0])
    mar_poly_small3.size_of_icons = config.soi_arcs_small


    mar_poly_small21 = body1.Markers.create(name="MAR_POLY_SMALL21")
    mar_poly_small21.orientation = rel_orient_custom(mar_arc9_ctr, [-64.41,0,0])
    mar_poly_small21.location = rel_loc_custom(mar_arc9_ctr, [0, 0, 0])
    mar_poly_small21.size_of_icons = config.soi_arcs_small


    mar_poly_small22 = body1.Markers.create(name="MAR_POLY_SMALL22")
    mar_poly_small22.location = rel_loc_custom(mar_poly_small21, [0.35, 0, 0])
    mar_poly_small22.size_of_icons = config.soi_arcs_small


    loc1=mar_curve_ctr.location
    loc2=mar_poly_small22.location

    mar_curve_mid1 = body1.Markers.create(name="MAR_CURVE_MID1")
    mar_curve_mid1.location = rel_loc_custom(mar_curve_ctr, [0, 0, 0])
    mar_curve_mid1.along_axis_orientation = [loc1[0],loc1[1], loc1[2], loc2[0],loc2[1], loc2[2]]

    mar_curve_mid2 = body1.Markers.create(name="MAR_CURVE_MID2")
    mar_curve_mid2.orientation = rel_orient_custom(mar_curve_mid1, [0,-11.25,0])
    mar_curve_mid2.location = rel_loc_custom(mar_curve_mid1, [0, 0, 0])
    mar_curve_mid2.size_of_icons = config.soi_arcs_small


    mar_curve_mid3 = body1.Markers.create(name="MAR_CURVE_MID3")
    mar_curve_mid3.location = loc_mirror_zx(mar_poly_small22.location, mar_curve_mid2, 'xz')
    mar_curve_mid3.size_of_icons = config.soi_arcs_small


    loc3=mar_curve_mid3.location
    loc4=mar_poly_small2.location

    line_small = body1.Geometries.createPolyline(name="POL_SMALL", close="no", 
                                    location=[loc3[0],loc3[1], loc3[2], loc4[0],loc4[1], loc4[2]])

    curve_list=[arc1, arc2, arc3, arc4, arc5, arc6, arc7, arc8, arc9, line_small]

    chain_object = body1.Geometries.createChain(name= "CHAIN1", 
                               objects_in_chain=curve_list) 
    polyline_original = body1.Geometries.createPolyline(name= "POL1", close="no",
                                path_curve=chain_object)
    
    mar_curve_ctr = body1.Markers.create(name="MAR_CURVE_CTR",
    location = rel_loc_custom(mar_curve_ctr, [0, 0, 0]),
    orientation = rel_orient_custom(mar_curve_ctr, [0,0,0]))
    #orientation = rel_orient_custom(mar_sprocket_ctr, [-90, 90, 90]))

    mar_mult_poly_ctr = body1.Markers.create(name="MAR_MULT_POLY_CTR",
    location = rel_loc_custom(mar_curve_ctr, [0, 0, 0]),
    orientation = rel_orient_custom(mar_curve_ctr, [0,0,0]),
    size_of_icons = config.soi_arcs_small)

    tooth_loc_arr = np.loadtxt('tooth_small_xy.txt', skiprows=2)
    tooth_loc_ready = tooth_loc_arr.flatten().tolist()

    mar_polcopy_list = []
    polylines_list = []

    for i in range(1,17):
        marker = body1.Markers.create(name="MAR_POLCOPY_"+str(i))
        if i == 1:
            marker.location = rel_loc_custom(mar_mult_poly_ctr, [0, 0, 0])
            marker.orientation = rel_orient_custom(mar_mult_poly_ctr, [0, 0, 0])
        else:
            marker.location = rel_loc_custom(mar_polcopy_list[i-2], [0, 0, 0])
            marker.orientation = rel_orient_custom(mar_polcopy_list[i-2], [22.5, 0, 0])

        polyline_copy = body1.Geometries.createPolyline(name= "POL"+str(i), 
                        location=tooth_loc_ready, close="no", relative_to=marker)

        mar_polcopy_list.append(marker)
        polylines_list.append(polyline_copy)

    chain_teeth = body1.Geometries.createChain(name="CHAIN1", objects_in_chain=polylines_list, center_marker =mar_curve_ctr) 

    polyline_object_all = body1.Geometries.createPolyline(name="POL_ALL", 
                    path_curve=chain_teeth, close="no")
    