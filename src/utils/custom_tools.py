import Adams 
import numpy as np

def rel_loc_custom(marker, coordinates): # parametrize relative location to a marker
    internal_expression = "(LOC_RELATIVE_TO({"+str(coordinates[0])+", "+str(coordinates[1])+", "+str(coordinates[2])+"}," +str(marker.name)+"))"
    return Adams.expression(str(internal_expression)) 

def rel_orient_custom(marker, angles): # parametrize relative orientation to a marker
    internal_expression = "(ORI_RELATIVE_TO({"+str(angles[0])+", "+str(angles[1])+", "+str(angles[2])+"}," +str(marker.name)+"))"
    return Adams.expression(str(internal_expression)) 

def loc_mirror_zx(coordinates, ref_marker):
    internal_expression = '(LOC_MIRROR({'+str(coordinates[0])+', '+str(coordinates[1])+', '+str(coordinates[2])+'}, '+str(ref_marker.name)+', '+'"zx"'+'))'
    return Adams.eval(str(internal_expression))

def loc_mirror_xy(coordinates, ref_marker):
    internal_expression = '(LOC_MIRROR({'+str(coordinates[0])+', '+str(coordinates[1])+', '+str(coordinates[2])+'}, '+str(ref_marker.name)+', '+'"xy"'+'))'
    return Adams.eval(str(internal_expression))

def loc_mirror_zy(coordinates, ref_marker):
    internal_expression = '(LOC_MIRROR({'+str(coordinates[0])+', '+str(coordinates[1])+', '+str(coordinates[2])+'}, '+str(ref_marker.name)+', '+'"zy"'+'))'
    return Adams.eval(str(internal_expression))

def ori_along_axis(marker_1, marker_2, axis):
    internal_expression='ORI_ALONG_AXIS('+str(marker_1.name)+','+str(marker_2.name)+',"'+axis+'")'
    return Adams.eval(str(internal_expression))

def ori_one_axis(p1, p2, axis):
    p1_str = ','.join(map(str, p1)) 
    p2_str = ','.join(map(str, p2)) 
    internal_expression="ORI_ONE_AXIS("+"{"+"{"+p1_str+"},{"+p2_str+"}"+"}"+',"'+axis+'")'
    return Adams.eval(str(internal_expression))