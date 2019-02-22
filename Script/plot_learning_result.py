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
    res = result[0]
    f, ax = plt.subplots()
    ax.plot(res.progress.total_timesteps, res.progress.eprewmean)
    ax.grid(True)
    ax.set_xlabel('time_step')
    ax.set_ylabel('eprewmean')
    plt.plot(res.progress.total_timesteps, res.progress.eprewmean)
    plt.show()

