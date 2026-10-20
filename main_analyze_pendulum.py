#
#
#
# PROCESSING PENDULUM OSCILATIONS
# Compare video tracking data with simulation results
# 
#
from src.res_proc.proc_oscilations import vtrack_vs_sim

### 0 KG
# vtrack_data = "src/data/vtrack_pendulum/tracked_data_pendulum_0kg_22s.txt"
# sim_data = "src/data/sim_out_pendulum/sim_pendulum_0kg.txt"

### 1 KG
# vtrack_data = "src/data/vtrack_pendulum/tracked_data_pendulum_1kg_62s.txt"
# sim_data = "src/data/sim_out_pendulum/sim_pendulum_1kg.txt"

### 2 KG
vtrack_data = "src/data/vtrack_pendulum/tracked_data_pendulum_2kg_56s.txt"
sim_data = "src/data/sim_out_pendulum/sim_pendulum_2kg.txt"



vtrack_vs_sim(vtrack_data, sim_data)