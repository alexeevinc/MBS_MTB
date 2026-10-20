#
import math as m
import pandas as pd
from matplotlib.pyplot import plot
import plotly.graph_objects as go
from plotly.subplots import make_subplots
from src.utils.datamanager import load_config, load_plot_settings
from src.utils.res_reader import AdamsXRFReader

set = load_plot_settings()

reader = AdamsXRFReader("G7_MG_494.res")
reader2 = AdamsXRFReader("G7_MG_495.res")
reader3 = AdamsXRFReader("G7_MG_496.res")
reader4 = AdamsXRFReader("G7_MG_497.res")


def read_optional_reader(reader_name, columns):
    """Read a comparison result only when its reader is defined above."""
    optional_reader = globals().get(reader_name)
    if optional_reader is None:
        return None

    return optional_reader.read_dataframe(
        columns,
        step_types=("dynamic", "contactIncident"),
        include_context=True,
    )


df = read_optional_reader(
    "reader",
    [
        "time.TIME",
        "BUSHING_SPR_MAIN_2_FRAME.TX",
        "DAMPER_DISP.Q",
        "body_link_33_out_XFORM.Z",
    ],
)
df2 = read_optional_reader("reader2", ["time.TIME", "body_link_33_out_XFORM.Z", "BUSHING_SPR_MAIN_2_FRAME.TX"])
df3 = read_optional_reader("reader3", ["time.TIME", "body_link_33_out_XFORM.Z", "BUSHING_SPR_MAIN_2_FRAME.TX"])
df4 = read_optional_reader("reader4", ["time.TIME", "body_link_33_out_XFORM.Z", "BUSHING_SPR_MAIN_2_FRAME.TX"])

sim_time = None if df is None else df["time.TIME"]
sim_time2 = None if df2 is None else df2["time.TIME"]
sim_time3 = None if df3 is None else df3["time.TIME"]
sim_time4 = None if df4 is None else df4["time.TIME"]
body_link_33_out_Z = None if df is None else df["body_link_33_out_XFORM.Z"]
body_link_33_out_Z2 = None if df2 is None else df2["body_link_33_out_XFORM.Z"]
body_link_33_out_Z3 = None if df3 is None else df3["body_link_33_out_XFORM.Z"]
body_link_33_out_Z4 = None if df4 is None else df4["body_link_33_out_XFORM.Z"]
crank_trq_sim = None if df is None else -df["BUSHING_SPR_MAIN_2_FRAME.TX"] * 1e-3
crank_trq_sim2 = None if df2 is None else -df2["BUSHING_SPR_MAIN_2_FRAME.TX"] * 1e-3
crank_trq_sim3 = None if df3 is None else -df3["BUSHING_SPR_MAIN_2_FRAME.TX"] * 1e-3
crank_trq_sim4 = None if df4 is None else -df4["BUSHING_SPR_MAIN_2_FRAME.TX"] * 1e-3

if df is not None:
    x = sim_time
    damper_disp_sim = df["DAMPER_DISP.Q"] - 225

    # LOGGER DATA
    dfpkl = pd.read_pickle('src/data/versuch06n2.pkl')
    meas_time = pd.Series(dfpkl['Time'])
    crank_trq_meas = 4.4 + pd.Series(dfpkl['crank_torque'])
    damper_disp_meas = -pd.Series(dfpkl['SUS_rear']) + 6.97
    measurement_time_shift_s = -19.5 + 0.353 + 0.015 - 0.007
    meas_time_shifted = meas_time + measurement_time_shift_s

    # Trim the shifted measurement trace to the visible simulation window.
    mask = (meas_time_shifted >= x.min()) & (meas_time_shifted <= x.max())
    meas_time_shifted = meas_time_shifted[mask]
    crank_trq_meas = crank_trq_meas[mask]
    damper_disp_meas = damper_disp_meas[mask]

fig1 = make_subplots(rows=2, cols=1, shared_xaxes=True)

# CRANK TORQUE
if crank_trq_sim is not None:
    fig1.add_trace(go.Scatter(x=sim_time, y=crank_trq_sim,
            line=dict(color=set.TUMBlue, width=2),
            marker=dict(color=set.TUMBlue), name='<span style="font: Arial; font-size: 11pt"> MG 494 torque',
            legendgroup='1'), row=1, col=1)

if crank_trq_sim2 is not None:
    fig1.add_trace(go.Scatter(x=sim_time2, y=crank_trq_sim2,
            line=dict(color=set.TUMOrange, width=2),
            marker=dict(color=set.TUMOrange), name='<span style="font: Arial; font-size: 11pt"> MG 495 torque',
            legendgroup='1'), row=1, col=1)

if crank_trq_sim3 is not None:
    fig1.add_trace(go.Scatter(x=sim_time3, y=crank_trq_sim3,
            line=dict(color=set.TUMGreen, width=2),
            marker=dict(color=set.TUMGreen), name='<span style="font: Arial; font-size: 11pt"> MG 496 torque',
            legendgroup='1'), row=1, col=1)

if crank_trq_sim4 is not None:
    fig1.add_trace(go.Scatter(x=sim_time4, y=crank_trq_sim4,
            line=dict(color=set.TUMGray2, width=2),
            marker=dict(color=set.TUMGray2), name='<span style="font: Arial; font-size: 11pt"> MG 497 torque',
            legendgroup='1'), row=1, col=1)

if df is not None:
    fig1.add_trace(go.Scatter(x=meas_time_shifted, y=crank_trq_meas,
            line=dict(color="black", width=2),
            marker=dict(color="black"), name='<span style="font: Arial; font-size: 11pt"> CRANK TRQ MEAS',
            legendgroup='1'), row=1, col=1)


# fig1.add_trace(go.Scatter(x=x, y=damper_disp_sim,
#         line=dict(color="red", width=3),
#         marker=dict(color="red"), name='<span style="font: Arial; font-size: 11pt"> DAMPER DISP SIM',
#         legendgroup='1'), row=2, col=1)

# fig1.add_trace(go.Scatter(x=meas_time_shifted, y=damper_disp_meas,
#         line=dict(color=set.TUMBlue, width=3),
#         marker=dict(color=set.TUMBlue), name='<span style="font: Arial; font-size: 11pt"> DAMPER DISP MEAS',
#         legendgroup='1'), row=2, col=1)

if body_link_33_out_Z is not None:
    fig1.add_trace(go.Scatter(x=sim_time, y=body_link_33_out_Z,
            line=dict(color=set.TUMBlue, width=3),
            marker=dict(color=set.TUMBlue), name='<span style="font: Arial; font-size: 11pt"> MG 494',
            legendgroup='1'), row=2, col=1)

if body_link_33_out_Z2 is not None:
    fig1.add_trace(go.Scatter(x=sim_time2, y=body_link_33_out_Z2,
            line=dict(color=set.TUMOrange, width=3),
            marker=dict(color=set.TUMOrange), name='<span style="font: Arial; font-size: 11pt"> MG 495',
            legendgroup='1'), row=2, col=1)

if body_link_33_out_Z3 is not None:
    fig1.add_trace(go.Scatter(x=sim_time3, y=body_link_33_out_Z3,
            line=dict(color=set.TUMGreen, width=3),
            marker=dict(color=set.TUMGreen), name='<span style="font: Arial; font-size: 11pt"> MG 496',
            legendgroup='1'), row=2, col=1)

if body_link_33_out_Z4 is not None:
    fig1.add_trace(go.Scatter(x=sim_time4, y=body_link_33_out_Z4,
            line=dict(color=set.TUMGray2, width=3),
            marker=dict(color=set.TUMGray2), name='<span style="font: Arial; font-size: 11pt"> MG 497',
            legendgroup='1'), row=2, col=1)

fig1.update_layout(
        yaxis_title=dict(text='<span style="font-family: Arial; font-size: 11pt"'
                        '> Torque<' # type here
                        '/span>'
                        '<span style="font-family: Arial; font-size: 11pt"'
                        '> in <' # type here
                        '/span>'
                        '<span style="font-family: Sitka Heading; font-size: 13pt"'
                        '> Nm <' # type here
                        '/span>'),
        yaxis2_title=dict(text='<span style="font-family: Arial; font-size: 11pt"'
                         '> Pin 33 Z-Displacement Z in <'
                         '/span>'
                         '<span style="font-family: Sitka Heading; font-size: 13pt"'
                         '> mm <'
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
        # dtick=0.1
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

# # fig1.update_layout(hovermode='x unified',
# #                   hoverlabel=dict(namelength=-1)) # Disable default hovertemplate


# # fig1.update_layout(title='<span style="font-family: Arial; font-size: 11pt"'
# #                     '> Weg des letzten Kettenglieds in Z-Richtung<' # type here
# #                     '/span>')

# fig1.update_layout(
# title={
#         'y':0.75,
#         'x':0.5,
#         'xanchor': 'center',
#         'yanchor': 'top'})


fig1.update_layout(autosize=True, width=1200, height=800)
fig1.show()
