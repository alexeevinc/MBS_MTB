import numpy as np
import pandas as pd
import math as m
import science as sci
from scipy.signal import detrend
from scipy.integrate import cumulative_trapezoid
from plotly.subplots import make_subplots
import plotly.graph_objects as go
from src.utils.datamanager import load_config, load_plot_settings
from src.utils.prep_input_data import insert_spline

dfpkl = pd.read_pickle('src/data/versuch06n2.pkl')
 
gyro_y = dfpkl['GYRO_Y']
gyro_y = gyro_y*-1 + 0.22
gyro_y = gyro_y[19630:21130]
time_df = dfpkl['Time']
itime = len(gyro_y)
time_df = time_df[:itime]
### GYRO POSTPROC
# def extrapolate_lin_delpta(signal, steps):
#     signal = np.array(signal)
#     delta = signal[-1] - signal[-10]        
#     return signal[-1] + delta * np.arange(0, steps, 0.1)

# gyro_extra = extrapolate_lin_delpta(gyro_y[520:530],  1.3)
# gyro_y_cut = np.concatenate((gyro_y[:529], gyro_extra), axis=0)
# gyro_y_cut = np.concatenate((gyro_y_cut, gyro_y[612:]), axis=0)


data1 = 'src/data/tracked_data_spr_upr.txt'
data2 = 'src/data/tracked_data_spr_main.txt'
data3 = 'src/data/tracked_data_pedal.txt'

time, pos_X_su, pos_Y_su, pos_X_s_main, pos_Y_s_main, pos_X_pedal, pos_Y_pedal = [], [], [], [], [], [], []

with open(data1, "r") as file:
    next(file) 
    for line in file:
        values = line.strip().split() 
        pos_X_su.append(float(values[1]))
        pos_Y_su.append(float(values[2]))

with open(data2, "r") as file:
    next(file) 
    for line in file:
        values = line.strip().split() 
        time.append(float(values[0]))
        pos_X_s_main.append(float(values[1]))
        pos_Y_s_main.append(float(values[2]))

with open(data3, "r") as file:
    next(file) 
    for line in file:
        values = line.strip().split() 
        pos_X_pedal.append(float(values[1]))
        pos_Y_pedal.append(float(values[2]))

pos_X_su = np.array(pos_X_su)              
pos_Y_su =  np.array(pos_Y_su)
pos_X_s_main =  np.array(pos_X_s_main)
pos_Y_s_main =  np.array(pos_Y_s_main)
pos_X_pedal =  np.array(pos_X_pedal)
pos_Y_pedal =  np.array(pos_Y_pedal)
time = np.array(time)

pos_X_s_main *= 1.94
pos_Y_s_main *= 1.94
pos_X_pedal *= 1.94
pos_Y_pedal *= 1.94

# delta_x = pos_X_s_main - pos_X_su
# delta_y = pos_Y_s_main - pos_Y_su
# angles_deg = np.degrees(np.arctan2(delta_x, delta_y)) 
# gyro_y = np.gradient(angles_deg, time)

# dx_su = np.gradient(pos_X_su, time)
# dy_su = np.gradient(pos_Y_su, time)
dx_s_main = np.gradient(pos_X_s_main, time*0.06)
dy_s_main = np.gradient(pos_Y_s_main, time*0.06)


def polyfit(time_meas, signal, order):
    coeffs = np.polyfit(time_meas, signal, order)
    poly = np.poly1d(coeffs)
    time_smooth = np.linspace(time_meas.min(), time_meas.max(), len(time_meas))
    signal_smooth = poly(time_smooth)
    return signal_smooth, time_smooth

def smooth_regions(time, signal, order0, order1, order2, order3, order4):
    fit0, time0 = polyfit(time_meas=time[:125], signal=signal[:125], order=order0)
    fit1, time1 = polyfit(time_meas=time[125:170], signal=signal[125:170], order=order1)
    fit2, time2 = polyfit(time_meas=time[170:200], signal=signal[170:200], order=order2)
    fit3, time3 = polyfit(time_meas=time[200:210], signal=signal[200:210], order=order3)
    fit4, time4 = polyfit(time_meas=time[210:], signal=signal[210:], order=order4)

    overall_fit = np.concatenate((fit0, fit1, fit2, fit3, fit4))
    time_smooth = np.concatenate((time0, time1, time2, time3, time4))

    return overall_fit, time_smooth

def smooth_regions2(time, signal, order, order0, order1, order2, order3, order4):
    fit, timef = polyfit(time_meas=time[:20], signal=signal[:20], order=order)
    fit0, time0 = polyfit(time_meas=time[20:125], signal=signal[20:125], order=order0)
    fit1, time1 = polyfit(time_meas=time[125:170], signal=signal[125:170], order=order1)
    fit2, time2 = polyfit(time_meas=time[170:200], signal=signal[170:200], order=order2)
    fit3, time3 = polyfit(time_meas=time[200:210], signal=signal[200:210], order=order3)
    fit4, time4 = polyfit(time_meas=time[210:], signal=signal[210:], order=order4)

    overall_fit = np.concatenate((fit, fit0, fit1, fit2, fit3, fit4))
    time_smooth = np.concatenate((timef, time0, time1, time2, time3, time4))

    return overall_fit, time_smooth


# dx_s_main_fit, dx_s_main_time_smooth = smooth_regions(time, dx_s_main, 10, 50, 50, 50, 50)
# dy_s_main_fit, dy_s_main_time_smooth = smooth_regions(time, dy_s_main, 10, 50, 50, 50, 50)
# dx_s_main_fit, dx_su_time_smooth = smooth_regions(time, dx_s_main, 10, 100, 200, 200, 200)

pos_X_su_fit, pos_X_su_time_smooth = smooth_regions(time, pos_X_su, 10, 210, 10, 50, 100)
pos_Y_su_fit, pos_Y_su_time_smooth = smooth_regions(time, pos_Y_su, 10, 10, 10, 50, 50)
#
pos_X_s_main_fit, pos_X_s_main_time_smooth = smooth_regions(time, pos_X_s_main, 10, 50, 10, 10, 10)
pos_Y_s_main_fit, pos_Y_s_main_time_smooth = smooth_regions(time, pos_Y_s_main, 20, 30, 10, 10, 10)
pos_X_s_main_fit[0] = pos_X_s_main_fit[1]      
pos_Y_s_main_fit[0]=10.63
pos_Y_s_main_fit[1]=10.63 + 0.05
#

pos_X_pedal_fit, pos_X_pedal_time_smooth = smooth_regions2(time, pos_X_pedal, 10, 10, 30, 7, 5, 10)
pos_Y_pedal_fit, pos_Y_pedal_time_smooth = smooth_regions(time, pos_Y_pedal, 7, 30, 10, 5, 10)
pos_X_pedal_fit[169] = 776.86
pos_Y_pedal_fit[0]= pos_Y_pedal_fit[1]
pos_Y_pedal_fit[168]= 723.9

# VELOCITIES FITTING
time_koef = 0.06
dx_s_main_fit = np.gradient(pos_X_s_main_fit, pos_X_s_main_time_smooth*time_koef)
dy_s_main_fit = np.gradient(pos_Y_s_main_fit, pos_Y_s_main_time_smooth*time_koef)
dx_s_main_fit[0:2]=0
dy_s_main_fit[0]=0
#
# dx_pedal_fit = np.gradient(pos_X_pedal_fit, pos_X_pedal_time_smooth*time_koef)
# dy_pedal_fit = np.gradient(pos_Y_pedal_fit, pos_Y_pedal_time_smooth*time_koef)


delta_x_smooth = pos_X_s_main_fit - pos_X_pedal_fit
delta_y_smooth = pos_Y_s_main_fit - pos_Y_pedal_fit
angles_deg_smooth = np.degrees(np.arctan2(delta_x_smooth, delta_y_smooth))-124.5
# angles_deg_smooth_fit, angles_deg_smooth_fit_time = smooth_regions(time, angles_deg_smooth, 5, 50, 50, 50, 50)
angles_deg_smooth_fit, angles_deg_smooth_fit_time = smooth_regions2(time, angles_deg_smooth, 3, 10, 50, 10, 10, 50)
gyro_y_smooth = np.gradient(angles_deg_smooth_fit, time*time_koef)
gyro_y_smooth[171] = -10.8
gyro_y_smooth[:40] = np.linspace(0, gyro_y_smooth[40], 40)
gyro_y_smooth[:70], timegfit = polyfit(time_meas=time[:70], signal=gyro_y_smooth[:70], order=4)

# FIX DISTURBANCE 
gyro_y_smooth_fix = np.copy(gyro_y_smooth)
gyro_y_smooth_fix = insert_spline(
        pos_X_pedal_time_smooth, gyro_y_smooth_fix,
        t_start = 13, t_end = 14.6
)

gyro_y_smooth_fix = insert_spline(
        pos_X_pedal_time_smooth, gyro_y_smooth_fix,
        t_start = 17, t_end = 20,
        start_value=None, end_value=0.0,
        before_value=None, after_value=0.0,
)

# FIX CONCATENATE ORIGINAL GYRO Y
gyro_y_smooth_plus_imu = np.copy(gyro_y_smooth)
time_imu_align = time_df/time_koef-0.4
imu_replace_start = 12
imu_replace_end = 24
replace_mask = (
        (pos_X_pedal_time_smooth >= imu_replace_start)
        & (pos_X_pedal_time_smooth <= imu_replace_end)
)
gyro_y_smooth_plus_imu[replace_mask] = np.interp(
        pos_X_pedal_time_smooth[replace_mask],
        time_imu_align,
        gyro_y,
)

gyro_y_smooth_plus_imu = insert_spline(
        pos_X_pedal_time_smooth, gyro_y_smooth_plus_imu,
        t_start = 11.85, t_end = 12.3
)


### OUTPUT ###
# data_out1 = 'src/data/tracked_data_spr_main_dt.txt'
# with open(data_out1, "w") as f:
#     f.write("Time(s)\tdX(mmps)\tdY(mmps)\tWY(degps)\n")
#     for t, s1, s2, s3 in zip(pos_X_s_main_time_smooth*time_koef, dx_s_main_fit, dy_s_main_fit, gyro_y_smooth_fix):
#         f.write(f"{t:.3f}\t{s1:.2f}\t{s2:.2f}\t{s3:.2f}\n")

### VIZUALIZATION ###
set = load_plot_settings()
cfg = load_config()

fig1 = make_subplots(rows=2, cols=1, shared_xaxes=True)
fig2 = make_subplots(rows=1, cols=1, shared_xaxes=True)

fig2.add_trace(go.Scatter(x=time_imu_align, y=gyro_y,
        marker=dict(color=set.TUMGray2), name='<span style="font: Arial; font-size: 11pt">gyro_y IMU ',
        legendgroup='1'), row=1, col=1)

### ANGULAR ###
# fig2.add_trace(go.Scatter(x=pos_X_pedal_time_smooth, y=angles_deg_smooth*5,
#         marker=dict(color=set.TUMGray2), name='<span style="font: Arial; font-size: 11pt">angles deg smooth ',
#         legendgroup='1'), row=1, col=1)

# fig2.add_trace(go.Scatter(x=angles_deg_smooth_fit_time, y=angles_deg_smooth_fit*5,
#         marker=dict(color=set.TUMBlue), name='<span style="font: Arial;   font-size: 11pt">angles deg smooth fit ',
#         legendgroup='1'), row=1, col=1)

# fig2.add_trace(go.Scatter(x=pos_X_pedal_time_smooth, y=gyro_y_smooth,
#         marker=dict(color=set.TUMBlue), name='<span style="font: Arial; font-size: 11pt">gyro_y VID smooth ',
#         legendgroup='1'), row=1, col=1)

fig2.add_trace(go.Scatter(x=pos_X_pedal_time_smooth, y=gyro_y_smooth_fix,
        marker=dict(color=set.TUMOrange), name='<span style="font: Arial; font-size: 11pt">gyro_y VID smooth FIX',
        legendgroup='1'), row=1, col=1)

fig2.add_trace(go.Scatter(x=pos_X_pedal_time_smooth, y=gyro_y_smooth_plus_imu,
        marker=dict(color=set.TUMGreen), name='<span style="font: Arial; font-size: 11pt">gyro_y VID + IMU',
        legendgroup='1'), row=1, col=1)

### SPROCKET MAIN ###
fig1.add_trace(go.Scatter(x=time, y=pos_X_s_main,
        marker=dict(color=set.TUMBlue), name='<span style="font: Arial; font-size: 11pt"> pos X spr main',
        legendgroup='1'), row=1, col=1)

fig1.add_trace(go.Scatter(x=pos_X_s_main_time_smooth, y=pos_X_s_main_fit,
        marker=dict(color=set.TUMOrange), name='<span style="font: Arial; font-size: 11pt"> pos X spr main fit',
        legendgroup='1'), row=1, col=1)

fig1.add_trace(go.Scatter(x=time, y=pos_Y_s_main,
        marker=dict(color=set.TUMBlue), name='<span style="font: Arial; font-size: 11pt"> pos Y spr main',
        legendgroup='1'), row=1, col=1)

fig1.add_trace(go.Scatter(x=pos_Y_s_main_time_smooth, y=pos_Y_s_main_fit,
        marker=dict(color=set.TUMOrange), name='<span style="font: Arial; font-size: 11pt"> pos Y spr main fit',
        legendgroup='1'), row=1, col=1)

### PEDAL ###
fig1.add_trace(go.Scatter(x=time, y=pos_X_pedal,
        marker=dict(color=set.TUMBlue), name='<span style="font: Arial; font-size: 11pt"> pos X pedal',
        legendgroup='1'), row=1, col=1)

fig1.add_trace(go.Scatter(x=pos_X_pedal_time_smooth, y=pos_X_pedal_fit,
        marker=dict(color=set.TUMOrange), name='<span style="font: Arial; font-size: 11pt"> pos X pedal fit',
        legendgroup='1'), row=1, col=1)

fig1.add_trace(go.Scatter(x=time, y=pos_Y_pedal,
        marker=dict(color=set.TUMBlue), name='<span style="font: Arial; font-size: 11pt"> pos Y pedal',
        legendgroup='1'), row=1, col=1)

fig1.add_trace(go.Scatter(x=pos_Y_pedal_time_smooth, y=pos_Y_pedal_fit,
        marker=dict(color=set.TUMOrange), name='<span style="font: Arial; font-size: 11pt"> pos Y pedal fit',
        legendgroup='1'), row=1, col=1)

### VELOCITY PEDAL ###
# fig1.add_trace(go.Scatter(x=pos_X_pedal_time_smooth, y=dx_pedal_fit,
#         marker=dict(color=set.TUMBlue), name='<span style="font: Arial; font-size: 11pt"> vel X pedal fit',
#         legendgroup='1'), row=1, col=1)

# fig1.add_trace(go.Scatter(x=pos_Y_pedal_time_smooth, y=dy_pedal_fit,
#         marker=dict(color=set.TUMOrange), name='<span style="font: Arial; font-size: 11pt"> vel Y pedal fit',
#         legendgroup='1'), row=1, col=1)


### VELOCITY MAIN ###
# fig1.add_trace(go.Scatter(x=time, y=dx_s_main,
#         marker=dict(color=set.TUMOrange), name='<span style="font: Arial; font-size: 11pt"> vel X spr main',
#         legendgroup='1'), row=1, col=1)

fig1.add_trace(go.Scatter(x=pos_X_s_main_time_smooth, y=dx_s_main_fit,
        marker=dict(color=set.TUMBlue), name='<span style="font: Arial; font-size: 11pt"> vel X spr main fit',
        legendgroup='1'), row=2, col=1)

# fig1.add_trace(go.Scatter(x=time*0.06, y=dy_s_main,
#         marker=dict(color=set.TUMOrange), name='<span style="font: Arial; font-size: 11pt"> vel Y spr main',
#         legendgroup='1'), row=1, col=1)





fig1.add_trace(go.Scatter(x=pos_X_s_main_time_smooth, y=-dy_s_main_fit,
        marker=dict(color=set.TUMOrange), name='<span style="font: Arial; font-size: 11pt"> vel Y spr main fit',
        legendgroup='1'), row=2, col=1)

# fig1.add_trace(go.Scatter(x=time_df, y=gyro_y_smooth*8,
#         marker=dict(color=set.TUMOrange), name='<span style="font: Arial; font-size: 11pt"> GYRO Y CUT',
#         legendgroup='1'), row=1, col=1)


# fig1.add_trace(go.Scatter(x=time_df, y=gyro_y*8,
#         marker=dict(color=set.TUMGreen), name='<span style="font: Arial; font-size: 11pt"> GYRO Y',
#         legendgroup='1'), row=1, col=1)


### SPROCKET UPR ###
# fig1.add_trace(go.Scatter(x=time, y=pos_X_su,
#         marker=dict(color=set.TUMBlue), name='<span style="font: Arial; font-size: 11pt"> pos X spr upr',
#         legendgroup='1'), row=1, col=1)

# fig1.add_trace(go.Scatter(x=pos_X_su_time_smooth, y=pos_X_su_fit,
#         marker=dict(color=set.TUMOrange), name='<span style="font: Arial; font-size: 11pt"> pos X spr upr fit',
#         legendgroup='1'), row=1, col=1)

# fig1.add_trace(go.Scatter(x=time, y=pos_Y_su,
#         marker=dict(color=set.TUMBlue), name='<span style="font: Arial; font-size: 11pt"> pos Y spr upr',
#         legendgroup='1'), row=1, col=1)

# fig1.add_trace(go.Scatter(x=pos_Y_su_time_smooth, y=pos_Y_su_fit,
#         marker=dict(color=set.TUMOrange), name='<span style="font: Arial; font-size: 11pt"> pos Y spr upr fit',
#         legendgroup='1'), row=1, col=1)


### VELOCITY ###
# fig1.add_trace(go.Scatter(x=time, y=dx_su,
#         marker=dict(color=set.TUMBlue), name='<span style="font: Arial; font-size: 11pt"> vel X',
#         legendgroup='1'), row=1, col=1)

# fig1.add_trace(go.Scatter(x=time, y=dy_su,
#         marker=dict(color=set.TUMGreen), name='<span style="font: Arial; font-size: 11pt"> vel Y ',
#         legendgroup='1'), row=1, col=1)

# fig1.add_trace(go.Scatter(x=time, y=pos_Y_su,
#         marker=dict(color=set.TUMBlue), name='<span style="font: Arial; font-size: 11pt"> pos Y spr upr',
#         legendgroup='1'), row=1, col=1)

# fig1.add_trace(go.Scatter(x=x, y=acc_z,
#         marker=dict(color=set.TUMBlue), name='<span style="font: Arial; font-size: 11pt"> ACC Z',
#         legendgroup='1'), row=1, col=1)

fig1.update_layout(
        yaxis_title=dict(text='<span style="font-family: Arial; font-size: 11pt"'
                        '> Weg   <' # type here
                        '/span>'
                        '<span style="font-family: Arial; font-size: 11pt"'
                        '> in <' # type here
                        '/span>'
                        '<span style="font-family: Sitka Heading; font-size: 13pt"'
                        '> mm <' # type here
                        '/span>'),
        xaxis_title=dict(text='<span style="font-family: Arial; font-size: 11pt"'
                        '>Zeit in der Aufnshme in <' # type here
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
        dtick=1
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
        legend_tracegroupgap=320,
        hovermode='x',
)

# fig1.update_layout(hovermode='x unified',
#                   hoverlabel=dict(namelength=-1)) # Disable default hovertemplate

fig1.update_xaxes(showspikes=True, spikemode='across', spikesnap='cursor', spikecolor="black", spikethickness=1)
# fig1.update_yaxes(showspikes=True, spikemode='across', spikesnap='cursor', spikecolor="red", spikethickness=2)

# fig1.update_layout(title='<span style="font-family: Arial; font-size: 11pt"'
#                     '> Weg des letzten Kettenglieds in Z-Richtung<' # type here
#                     '/span>')

fig1.update_layout(
title={
        'y':0.75,
        'x':0.5,
        'xanchor': 'center',
        'yanchor': 'top'})

# fig1.update_layout(autosize=False, width=642, height=300)
fig1.update_layout(autosize=False, width=1200, height=800)
fig1.show()

### FIGURE 2 ###
fig2.update_layout(
        yaxis_title=dict(text='<span style="font-family: Arial; font-size: 11pt"'
                        '> Weg   <' # type here
                        '/span>'
                        '<span style="font-family: Arial; font-size: 11pt"'
                        '> in <' # type here
                        '/span>'
                        '<span style="font-family: Sitka Heading; font-size: 13pt"'
                        '> mm <' # type here
                        '/span>'),
        xaxis_title=dict(text='<span style="font-family: Arial; font-size: 11pt"'
                        '>Zeit in der Aufnshme in <' # type here
                        '/span> <span style="font-family: Sitka Heading; font-size: 13pt"'
                        '>s<' # type here
                        '/span>'),

        xaxis1=dict(tickfont=dict(family="Cambria", size=13)),
        yaxis1=dict(tickfont=dict(family="Cambria", size=13)),
        plot_bgcolor='white',
)

fig2.update_layout(plot_bgcolor='white', legend_tracegroupgap=60)
fig2.update_xaxes(
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
        dtick=1
)
fig2.update_yaxes(
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

fig2.update_layout(
        legend=dict(x=0.9, y=0.95, xanchor='right', yanchor='top', font=dict(family="Arial", size=29),itemsizing='constant'),
        legend_bordercolor='#000000',
        legend_borderwidth=1,
        font_color="black",
        legend_tracegroupgap=320,
        hovermode='x',
)

# fig1.update_layout(hovermode='x unified',
#                   hoverlabel=dict(namelength=-1)) # Disable default hovertemplate

fig2.update_xaxes(showspikes=True, spikemode='across', spikesnap='cursor', spikecolor="black", spikethickness=1)
# fig1.update_yaxes(showspikes=True, spikemode='across', spikesnap='cursor', spikecolor="red", spikethickness=2)

# fig1.update_layout(title='<span style="font-family: Arial; font-size: 11pt"'
#                     '> Weg des letzten Kettenglieds in Z-Richtung<' # type here
#                     '/span>')

fig2.update_layout(
title={
        'y':0.75,
        'x':0.5,
        'xanchor': 'center',
        'yanchor': 'top'})

# fig1.update_layout(autosize=False, width=642, height=300)
fig2.update_layout(autosize=False, width=1200, height=800)
fig2.show()