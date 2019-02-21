from baselines.common import plot_util as pu
import matplotlib
# matplotlib.use('tkagg')
import matplotlib.pyplot as plt
import numpy as np

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--log_path", type=str)
    args = parser.parse_args()

    result = pu.load_results(args.log_path)
    r = result[0]
    plt.plot(r.progress.total_timesteps, r.progress.eprewmean)
    # __import__('ipdb').set_trace()
    plt.show()

