import numpy as np
import pandas as pd
import plotly.graph_objects as go
from scipy.signal import find_peaks


def analyze_decay(df, name, poly_degree=2):
    t = df.iloc[:, 0].to_numpy()
    x = df.iloc[:, 1].to_numpy()

    x0 = np.mean(x)
    y = x - x0

    abs_y = np.abs(y)
    prom = 0.05 * (np.max(abs_y) - np.min(abs_y))

    peak_idx, _ = find_peaks(
        abs_y,
        prominence=prom,
        distance=int(0.4 / np.mean(np.diff(t)))
    )

    peak_times = t[peak_idx]
    amps = abs_y[peak_idx]

    delta_values = np.log(amps[:-2] / amps[2:])
    delta = np.mean(delta_values)

    zeta = delta / np.sqrt(4 * np.pi**2 + delta**2)

    coeffs = np.polyfit(peak_times, amps, poly_degree)
    poly_env = np.poly1d(coeffs)

    t_fit = np.linspace(peak_times.min(), peak_times.max(), 500)
    upper_env = poly_env(t_fit)
    lower_env = -upper_env

    print(f"\n{name}")
    print(f"Equilibrium X0: {x0:.4f} mm")
    print(f"Logarithmic decrement delta: {delta:.6f}")
    print(f"Damping ratio zeta: {zeta:.6f}")

    return {
        "name": name,
        "t": t,
        "y": y,
        "t_fit": t_fit,
        "upper_env": upper_env,
        "lower_env": lower_env,
        "delta": delta,
        "zeta": zeta,
    }

def vtrack_vs_sim(vtrack_data, sim_data):

    vtrack = analyze_decay(vtrack_data, "Video tracking")
    sim = analyze_decay(sim_data, "Simulation")

    # Detect negative peaks (invert signal)
    v_peak_idx, _ = find_peaks(
        -vtrack["y"],
        prominence=0.2 * np.max(np.abs(vtrack["y"]))
    )

    s_peak_idx, _ = find_peaks(
        -sim["y"],
        prominence=0.2 * np.max(np.abs(sim["y"]))
    )

    # First negative peak times
    v_first_peak_time = vtrack["t"][v_peak_idx[0]]
    s_first_peak_time = sim["t"][s_peak_idx[0]]

    # Compute shift
    time_shift = v_first_peak_time - s_first_peak_time

    # Apply shift
    sim["t"] += time_shift
    sim["t_fit"] += time_shift

    print(f"Simulation time shift applied: {time_shift:.4f} s")
    print(f"Video first NEG peak: {v_first_peak_time:.4f} s")
    print(f"Sim first NEG peak before shift: {s_first_peak_time:.4f} s")

    fig = go.Figure()

    # Video tracking
    fig.add_trace(go.Scatter(
        x=vtrack["t"],
        y=vtrack["y"],
        mode="lines",
        name="Video tracking",
        line=dict(width=2, color="blue")
    ))

    fig.add_trace(go.Scatter(
        x=vtrack["t_fit"],
        y=vtrack["upper_env"],
        mode="lines",
        name="Video upper envelope",
        line=dict(width=0.8, color="red")
    ))

    fig.add_trace(go.Scatter(
        x=vtrack["t_fit"],
        y=vtrack["lower_env"],
        mode="lines",
        name="Video lower envelope",
        line=dict(width=0.8, color="red")
    ))

    # Simulation
    fig.add_trace(go.Scatter(
        x=sim["t"],
        y=sim["y"],
        mode="lines",
        name="Simulation",
        line=dict(width=2, color="green")
    ))

    fig.add_trace(go.Scatter(
        x=sim["t_fit"],
        y=sim["upper_env"],
        mode="lines",
        name="Simulation upper envelope",
        line=dict(width=0.8, color="orange", dash="dash")
    ))

    fig.add_trace(go.Scatter(
        x=sim["t_fit"],
        y=sim["lower_env"],
        mode="lines",
        name="Simulation lower envelope",
        line=dict(width=0.8, color="orange", dash="dash")
    ))

    fig.add_hline(y=0, line_dash="dash")

    fig.update_yaxes(
        mirror=True,
        ticks="inside",
        ticklen=10,
        showline=True,
        zeroline=True,
        zerolinewidth=1,
        zerolinecolor="lightgrey",
        linecolor="black",
        gridcolor="lightgrey",
        gridwidth=1,
        exponentformat="none",
        separatethousands=False,
        title_standoff=20,
    )

    fig.update_xaxes(
        showspikes=True,
        spikemode="across",
        spikesnap="cursor",
        spikecolor="black",
        spikethickness=1,
    )

    fig.update_xaxes(
        mirror=True,
        ticks="inside",
        ticklen=10,
        showline=True,
        linecolor="black",
        gridcolor="lightgrey",
        gridwidth=1,
        showgrid=True,
    )

    fig.update_layout(
        title=(
            "Pendulum Decay: "
            f"Logarithmic Decrement δ, Damping Ratio ζ<br>"
            f"VID: δ={vtrack['delta']:.6f}, ζ={vtrack['zeta']:.6f} <br>"
            f"SIM: δ={sim['delta']:.6f}, ζ={sim['zeta']:.6f}"
        ),
        xaxis_title="Time (s)",
        yaxis_title="Tracking point horizontal projection (mm)",
        template="plotly_white",
        hovermode="x unified",
        plot_bgcolor="white",
        paper_bgcolor="white",
        autosize=False,
        width=1500,
        height=700,
    )

    fig.show()