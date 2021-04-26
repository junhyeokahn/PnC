import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import os

## -----------------------------------------------------------------------------
## Read Data
## -----------------------------------------------------------------------------
file_path = os.getcwd() + "/../../../ExperimentData/"

t = np.genfromtxt(file_path+'running_time.txt', delimiter='\n', dtype=(float))

st_idx = 5
end_idx = len(t) - 10
t = t[st_idx:end_idx]

mpc_rxn_forces = np.genfromtxt(file_path+'mpc_rxn_forces.txt', delimiter=None, dtype=(float))[st_idx:end_idx]

## UNCOMMENT WHEN WBIC DATA PROVIDED
wbic_rxn_forces = np.genfromtxt(file_path+'wbc_rxn_forces.txt', delimiter=None, dtype=(float))[st_idx:end_idx]

y_label = ["Fx", "Fy", "Fz"]


data_phse = np.genfromtxt(file_path+'phase.txt', delimiter=None, dtype=(float))[st_idx:end_idx]

phseChange = []
for i in range(0,len(t)-1):
        if data_phse[i] != data_phse[i+1]:
            phseChange.append(i)
        else:
            pass

## -----------------------------------------------------------------------------
## Plot Cmd
## -----------------------------------------------------------------------------
def plot_phase(ax):
    for j in phseChange:
        ax.axvline(x=t[j],color='indigo',linestyle='-')
        ax.text(t[j],ax.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')

fig, axes = plt.subplots(3, 4)
for i in range(4):
    for j in range(3):
        axes[j,i].plot(t, mpc_rxn_forces[:,3*i+j], color='r', linestyle='dashed', linewidth=2)
        
        #FOR TESTING (use the line above when the length of t matches mpc_rxn_forces.shape[0])
        #axes[j,i].plot(t[:mpc_rxn_forces.shape[0]], mpc_rxn_forces[:,3*i+j], 'r--', linewidth=2)
        
        axes[j,i].grid(True)
        axes[j,i].set_ylabel(y_label[j])
        plot_phase(axes[j,i])

        #FOR FUTURE USE (when we have wbic data)
        axes[j,i].plot(t[:wbic_rxn_forces.shape[0]], wbic_rxn_forces[:,3*i+j], color='b', linewidth=3)

        #title
        title = y_label[j]+ " Rxn Force Profile for leg " + str(i) #maybe change to be shorter
        axes[j,i].set_title(title) 


plt.show()


