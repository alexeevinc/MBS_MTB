###
### RUNNING SIMULATIONS
###
import sys
import os
import Adams
import numpy as np
import pandas as pd
import math as m
import plotly.graph_objects as go
from plotly.subplots import make_subplots
from src.utils.datamanager import load_config, load_plot_settings


abs_work_dir_path = os.path.abspath(os.curdir)
sys.path.append(abs_work_dir_path)  

from src.utils.datamanager import load_config, load_plot_settings
from src.utils import sim_notification

config = load_config()


class SIM_MODELING():
    def __init__(self, model_name: str):
        self.main_model = Adams.stoo("."+ model_name)

    def set_simulation(self, name: str, duration: float, number_of_steps: float):
            simulation = self.main_model.Simulations.create(name=name, duration=duration, number_of_steps=number_of_steps)
            print("Simulation '" + name + "' created with duration " + str(duration) + "s and " + str(number_of_steps) + " steps.")
            return simulation
    
    def run_simulation(self, simulation):
            print("Simulation " + simulation.name + " started.")
            simulation.simulate()

    def get_results(self, new_name_4_results:str):
            analysis = self.main_model.Analyses['Last_Run']
            # analysis.name = new_name_4_results
        #     cpu_mea_seconds = round(self.analysis.results["CPU_MEA"]["Q"].values[-1], 0)
        #     cpu_minutes = int(cpu_mea_seconds // 60)
        #     cpu_seconds = round(cpu_mea_seconds % 60, 0)
        #     cpu_mea_convert = [cpu_minutes, cpu_seconds]

            return analysis
          

modeling = SIM_MODELING(model_name="CHAIN_DRIVE_G7") # !!! CHANGE MODEL NAME HERE

# # ### SIMULATION 1 START
# sim1 = modeling.set_simulation(name="SIM_1", duration=0.15, number_of_steps=150) # CREATE NEW SIMULATION SCRIPT 
# # sim_notification.send_via_telegram("Simulation session started...")  
# modeling.run_simulation(simulation=sim1)
# # ### SIMULATION 1 FINISH

analysis = modeling.get_results(new_name_4_results='SIM_1_RESULTS') # RENAMES RESULTS AND GETS REQUESTS
sim_time = analysis.results["TIME"].values
# 
### CHANGE OUTPUT FILE NAME:
# res_name = "sim_pendulum_0kg"
# res_name = "sim_pendulum_1kg"
res_name = "sim_pendulum_2kg"

#
# with open(f"src/data/sim_out_pendulum/{res_name}.txt", "w") as f:
#     # header
#     f.write("Time(s)\tDY(mm)\n")
#     # data
#     for t, x in zip(sim_time, dy_tracking):
#         f.write(f"{t:.4f}\t{x:.2f}\n")

####
### OVERWRITE DESIGN VARIABLES HERE
# pre_KT = Adams.stoo("pre_KT")
# pre_KT.value = 5000 # new value 

# chain_mu_st = Adams.stoo("chain_mu_st")
# chain_mu_st.value = 0.1 # new value 

# chain_mu_dyn = Adams.stoo("chain_mu_dyn")
# chain_mu_dyn.value = 0.4 # new value 
###

### SIMULATION 2 START
# sim2 = modeling.set_simulation(name="SIM_2", duration=0.2, number_of_steps=200) # CREATE NEW SIMULATION SCRIPT 
# modeling.run_simulation(simulation=sim2) 
# cpu_mea = modeling.get_results(new_name_4_results='SIM_2_RESULTS') # RENAMES RESULTS AND GETS REQUESTS
### SIMULATION 1 FINISH



# sim_notification.send_via_telegram("✅ Simulation"+sim2.name+" is finished!\n CPU Time: "+ str(cpu_mea[0])+" min "+ str(cpu_mea[1])+" s")  #uncomment to send notifications via Telegram

# sim_notification.send_via_telegram("...simulation session finished")  



from src.utils.res_reader import AdamsXRFReader


reader = AdamsXRFReader("resss.res")

df = reader.read_dataframe(
    [
        "BUSHING_SPR_MAIN_2_FRAME.Element_Torque.X_component"
    ],
    step_types=("dynamic", "contactIncident"),
    include_context=True,
)

print(df.head())


# dfpkl = pd.read_pickle('src/data/versuch06n2.pkl')
 
# sus_rear = dfpkl['SUS_rear']
# # rd_cage = dfpkl['Angle_RD']
# gyro_y = dfpkl['GYRO_Y']
# gyro_y = gyro_y*-1 + 0.22
# crank_trq = dfpkl['crank_torque']
# # gyro_y = gyro_y[19630:]
# time = dfpkl['Time']

# x=sim_time
# set = load_plot_settings()
# cfg = load_config()


# fig1 = make_subplots(rows=2, cols=1, shared_xaxes=True)

# # CRANK TORQUE
# fig1.add_trace(go.Scatter(x=x, y=crank_trq+4,
#         marker=dict(color=set.TUMBlue), name='<span style="font: Arial; font-size: 11pt"> CRANK TORQUE',
#         legendgroup='1'), row=1, col=1)

# fig1.show()


