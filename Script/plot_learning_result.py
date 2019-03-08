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
    # idx = 140
    f3, ax3 = plt.subplots()
    # ax3.plot(res.progress.nupdates[:idx], res.progress.total_num_dones[:idx])
    ax3.plot(res.progress.nupdates, res.progress.total_num_dones)
    ax3.grid(True)
    ax3.set_xlabel('nupdates')
    ax3.set_ylabel('total_num_dones')

    # ==========================================================================
    # total_numstep vs total reward
    # ==========================================================================
    f4, ax4 = plt.subplots()
    ax4.plot(res.progress.total_timesteps, res.progress.dataset_rew)
    ax4.set_xlabel('total_timestep')
    ax4.set_ylabel('dataset_rew')
    ax4.grid(True)

    # ==========================================================================
    # nupdates vs totla reward
    # ==========================================================================
    f6, ax6 = plt.subplots()
    ax6.plot(res.progress.nupdates, res.progress.dataset_rew)
    ax6.set_xlabel('nupdates')
    ax6.set_ylabel('dataset_rew')
    ax6.grid(True)

    plt.show()
