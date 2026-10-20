import numpy as np
import pandas as pd
import math as m
import science as sci
from scipy.signal import detrend
from scipy.integrate import cumulative_trapezoid
from plotly.subplots import make_subplots
import plotly.graph_objects as go
from src.utils.datamanager import load_config, load_plot_settings

# df = pd.read_excel('src/data/versuch25.xlsx', skiprows=[1])

# pkl_path = "src/data/versuch25.pkl"
# # list11 = df['SUS_rear']
# # print(list11)
# df.to_pickle(pkl_path)

# gcalc1 = m.sqrt(0.46**2+9.87**2)
# gcalc2 = m.sqrt(6.53**2+7.12**2)
# gcalc3 = m.sqrt(9.59**2+0.1**2)
# print(gcalc1)
# print(gcalc2)
# print(gcalc3) 

# DM = m.sqrt((0-863.26)**2+(-2-20.65)**2)
# D_main_uml = m.sqrt((-20-863.26)**2+(114-20.65)**2)
# print(DM)

DM_vid = m.sqrt((496-478)**2+(75-18)**2)
print(DM_vid)

dfpkl = pd.read_pickle('src/data/versuch06n2.pkl')
 
sus_rear = dfpkl['SUS_rear']
# rd_cage = dfpkl['Angle_RD']
gyro_y = dfpkl['GYRO_Y']
gyro_y = gyro_y*-1 + 0.22
crank_trq = dfpkl['crank_torque']
# gyro_y = gyro_y[19630:]

acc_x = dfpkl['ACC_X']
acc_z = dfpkl['ACC_Z']
# acc_z_ext = dfpkl['ACC_Z_EXTERN']
time = dfpkl['Time']

def calc_theta(gyro, theta0_deg):
    inc = 0.0000008           
    n = len(gyro)
    drift = np.arange(n) * inc
    gyro_corrected = gyro.to_numpy() - drift
    dt = 0.001 
    def cumulative_trapezoid(y, x=None, initial=0):
        trapz_areas = 0.5 * (y[:-1] + y[1:]) * 0.001
        cum_int = np.cumsum(trapz_areas)
        if initial is not None:
                cum_int = np.insert(cum_int, 0, initial)

        return cum_int
    theta = cumulative_trapezoid(gyro_corrected)
    return theta


def subtract_gravity(ax, az, theta):
    theta = theta*m.radians(1)
    gw = np.array([0, 9.81])
    ax_linear = []
    az_linear = []
    for i in range(len(ax)):
        R = np.array([
            [np.cos(theta[i]), np.sin(theta[i])],
            [-np.sin(theta[i]), np.cos(theta[i])]
        ])
        g_body = R @ gw
        ax_linear.append(ax[i] - g_body[0])
        az_linear.append(az[i] - g_body[1])
    return ax_linear, az_linear

theta_est = calc_theta(gyro_y, 0) + 63#60.5#63.5
ax_linear, az_linear = subtract_gravity(acc_x, acc_z, theta_est)

t1 = 9191
# gyro_y=gyro_y[t1:t1+3*1000]
# gyro_y.iloc[0]=0

# acc_x=acc_x[t1:t1+3*1000]
# acc_x.iloc[0]=0

# sus_rear=sus_rear[t1:t1+3*1000]
# sus_rear += abs(sus_rear.iloc[772])
# sus_rear[0:772]=0
# zeros = pd.Series([0] * 500)
# sus_rear = pd.concat([zeros, sus_rear], ignore_index=True)
# sus_rear /= -1000

# sus_rear_dt = np.concatenate((np.diff(sus_rear), [0]))
# sus_rear_dt_conv = np.convolve(sus_rear_dt, np.ones(3) / 3, mode='same')

# sus_rear_dt_conv[:9000]=0
# np.append(sus_rear_conv_dt, 0) 
# # sus_rear_conv_dt[-10:]=0
# sus_rear_conv_dt[1803:-1]=0
# # SHIFT THE SUS IN TIME
# sus_rear_conv_dt_shift=np.concatenate((sus_rear_conv_dt[:20], sus_rear_conv_dt[:-20])) 

# DIFF SUS RAW
# gyro_y = gyro_y-0.21#*0.017 # deg to rad VERSUCH 25
gyro_y_cutout1 =pd.concat([gyro_y[:10464], gyro_y[10495:]]) # old->10469
gyro_y_cutout = gyro_y_cutout1[10446:10473:3]

gyro_y_cutout_new = pd.concat([gyro_y_cutout1[:10446], gyro_y_cutout])
gyro_y_cutout_new2 = pd.concat([gyro_y_cutout_new, gyro_y_cutout1[10473:]])
    
# gyro_y_cutout =pd.concat([gyro_y_cutout[:10432], gyro_y_cutout[10446:]])
# gyro_y_cutout[10460:10475] = np.convolve(gyro_y_cutout[10460:10475], np.ones(3) / 3, mode='same')
# gyro_y_cutout[10460] = gyro_y[10460]
# gyro_y_cutout = np.concatenate(gyro_y[:10469],gyro_y[10495:]) # VERSUCH 31  cut out the 0.025s long shake in the middle
# # acc_x = self.df['ACC_X']
# # acc_z = self.df['ACC_Z']
# t1 = 9191
# t2 = 3*1000
# gyro_y=gyro_y[t1:t1+t2]
# gyro_y.iloc[0:3]=0
# zeros = pd.Series([0] * 500)
# gyro_y = pd.concat([zeros, gyro_y], ignore_index=True)
# gyro_y.iloc[2388:-1]=0
# time=time[:t2+500]

x = time
set = load_plot_settings()
cfg = load_config()

x_new = time[:len(gyro_y_cutout_new2)]

fig1 = make_subplots(rows=1, cols=1, shared_xaxes=True)
# fig1.add_trace(go.Scatter(x=x, y=sus_rear_conv_dt*100000,
#         marker=dict(color=set.TUMBlue), name='<span style="font: Arial; font-size: 11pt"> DSUS orig',
#         legendgroup='1'), row=1, col=1)
# fig1.add_trace(go.Scatter(x=x, y=sus_rear_conv_dt_shift*100000,
#         marker=dict(color=set.TUMOrange), name='<span style="font: Arial; font-size: 11pt"> DSUS w_shift',
#         legendgroup='1'), row=1, col=1)
# fig1.add_trace(go.Scatter(x=x, y=acc_z_ext,
#         marker=dict(color=set.TUMGreen), name='<span style="font: Arial; font-size: 11pt"> Z extern',
#         legendgroup='1'), row=1, col=1)

# GYRO Y
fig1.add_trace(go.Scatter(x=x, y=gyro_y,
        marker=dict(color=set.TUMGray2), name='<span style="font: Arial; font-size: 11pt"> GYRO_Y',
        legendgroup='1'), row=1, col=1)
# CRANK TORQUE
# fig1.add_trace(go.Scatter(x=x, y=crank_trq+4,
#         marker=dict(color=set.TUMBlue), name='<span style="font: Arial; font-size: 11pt"> CRANK TORQUE',
#         legendgroup='1'), row=1, col=1)

# fig1.add_trace(go.Scatter(x=x_new, y=gyro_y_cutout_new2,
#         marker=dict(color=set.TUMOrange), name='<span style="font: Arial; font-size: 11pt"> GYRO_Y CUT OUT',
#         legendgroup='1'), row=1, col=1)
# fig1.add_trace(go.Scatter(x=x, y=gyro_y_cutout1,
#         marker=dict(color=set.TUMOrange), name='<span style="font: Arial; font-size: 11pt"> GYRO_Y CUT OUT',
#         legendgroup='1'), row=1, col=1)
# fig1.add_trace(go.Scatter(x=x, y=sus_rear_dt,
#         marker=dict(color=set.TUMBlue), name='<span style="font: Arial; font-size: 11pt"> raw dt',
#         legendgroup='1'), row=1, col=1)
# fig1.add_trace(go.Scatter(x=x, y=sus_rear_dt_conv,
#         marker=dict(color=set.TUMOrange), name='<span style="font: Arial; font-size: 11pt"> conv dt',
#         legendgroup='1'), row=1, col=1)

# fig1.add_trace(go.Scatter(x=x, y=sus_rear,
#         marker=dict(color=set.TUMOrange), name='<span style="font: Arial; font-size: 11pt"> SUS REAR',
#         legendgroup='1'), row=1, col=1)

# fig1.add_trace(go.Scatter(x=x, y=rd_cage,
#         marker=dict(color=set.TUMOrange), name='<span style="font: Arial; font-size: 11pt">RD Angle',
#         legendgroup='1'), row=2, col=1)

# ACC
# fig1.add_trace(go.Scatter(x=x, y=acc_x,
#         marker=dict(color=set.TUMOrange), name='<span style="font: Arial; font-size: 11pt"> ACC X',
#         legendgroup='1'), row=1, col=1)

# fig1.add_trace(go.Scatter(x=x, y=ax_linear,
#         marker=dict(color=set.TUMBlue), name='<span style="font: Arial; font-size: 11pt"> AX LIN',
#         legendgroup='1'), row=1, col=1)

# fig1.add_trace(go.Scatter(x=x, y=az_linear,
#         marker=dict(color=set.TUMGreen), name='<span style="font: Arial; font-size: 11pt"> AZ LIN',
#         legendgroup='1'), row=1, col=1)

# fig1.add_trace(go.Scatter(x=x, y=theta_est,
#         marker=dict(color=set.TUMGray2), name='<span style="font: Arial; font-size: 11pt"> THETA EST',
#         legendgroup='1'), row=1, col=1)

# fig1.add_trace(go.Scatter(x=x, y=acc_z,
#         marker=dict(color=set.TUMBlue), name='<span style="font: Arial; font-size: 11pt"> ACC Z',
#         legendgroup='1'), row=1, col=1)

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
        dtick=0.1
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
