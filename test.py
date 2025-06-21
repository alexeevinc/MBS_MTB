

import numpy as np
import pandas as pd
from plotly.subplots import make_subplots
import plotly.graph_objects as go
from src.utils.datamanager import load_config, load_plot_settings

# df = pd.read_excel('src/data/versuch25.xlsx', skiprows=[1])
# pkl_path = "src/data/versuch25.pkl"
# list11 = df['SUS_rear']
# print(list11)
# df.to_pickle(pkl_path)

dfpkl = pd.read_pickle('src/data/versuch25.pkl')
sus_rear = dfpkl['SUS_rear']
gyro_y = dfpkl['GYRO_Y']
time = dfpkl['Time']
t1 = 9191
gyro_y=gyro_y[t1:t1+3*1000]
gyro_y.iloc[0]=0

sus_rear=sus_rear[t1:t1+3*1000]
sus_rear += abs(sus_rear.iloc[772])
sus_rear[0:772]=0
zeros = pd.Series([0] * 500)
sus_rear = pd.concat([zeros, sus_rear], ignore_index=True)
sus_rear /= -1000 
sus_rear_conv = np.convolve(sus_rear, np.ones(3) / 3, mode='same')
sus_rear_conv_dt = np.gradient(sus_rear_conv)
np.append(sus_rear_conv_dt, 0)

x = time
set = load_plot_settings()


fig1 = make_subplots(rows=1, cols=1, shared_xaxes=True)
fig1.add_trace(go.Scatter(x=x, y=sus_rear_conv_dt,
        marker=dict(color=set.TUMBlue), name='<span style="font: Arial; font-size: 11pt"> Modell',
        legendgroup='1'), row=1, col=1)
fig1.add_trace(go.Scatter(x=x, y=sus_rear_conv,
        marker=dict(color=set.TUMOrange), name='<span style="font: Arial; font-size: 11pt"> Modell',
        legendgroup='1'), row=1, col=1)

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
        dtick=0.5
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
        legend_tracegroupgap=320
)

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
