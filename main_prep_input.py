###
###
###
from src.utils.prep_input_data import prep_data, plot_data

data_vtrack, data_logger, vtrack_scaling, logger_scaling = prep_data()
#
# !!! DATA IS PLOTTED IN SAME UNITS AS IT APPEARS AFTER CONVERSION IN ADAMS !!! -> check units settings
plot_data(data_vtrack, data_logger, vtrack_scaling, logger_scaling, beacons=[0.4, 1.4])  # set time beacons to inspect data values !step 0.004 s!


