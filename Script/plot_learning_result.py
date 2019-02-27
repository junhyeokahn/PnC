import matplotlib
matplotlib.use('tkagg')
from baselines.common import plot_util as pu
import matplotlib.pyplot as plt
import numpy as np

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--log_path", type=str)
    args = parser.parse_args()

    result = pu.load_results(args.log_path)

    # ==========================================================================
    # total_numstep vs eprewmean
    # ==========================================================================
    res = result[0]
    f1, ax1 = plt.subplots()
    ax1.plot(res.progress.total_timesteps, res.progress.eprewmean)
    ax1.grid(True)
    ax1.set_xlabel('total_timestep')
    ax1.set_ylabel('eprewmean')

    # ==========================================================================
    # nupdates vs eprewmean
    # ==========================================================================
    f2, ax3 = plt.subplots()
    ax3.plot(res.progress.nupdates, res.progress.eprewmean)
    ax3.grid(True)
    ax3.set_xlabel('nupdates')
    ax3.set_ylabel('eprewmean')

    # ==========================================================================
    # nupdates vs total_num_dones
    # ==========================================================================
    # f3, ax3 = plt.subplots()
    # ax3.plot(res.progress.nupdates, res.progress.total_num_dones)
    # ax3.grid(True)
    # ax3.set_xlabel('nupdates')
    # ax3.set_ylabel('total_num_dones')


    # f4, ax4 = plt.subplots()
    # ax4.plot(np.cumsum(res.monitor.l), pu.smooth(res.monitor.r, radius=10))
    # ax4.grid(True)

    # ==========================================================================
    # nupdates vs total_num_dones
    # ==========================================================================
    f5, ax5 = plt.subplots()
    ax5.plot(res.progress.total_timesteps, res.progress.eplenmean)
    ax5.grid(True)
    ax5.set_xlabel('total_timestep')
    ax5.set_ylabel('eplenmean')

    plt.show()
