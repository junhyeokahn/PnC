import numpy as np
import matplotlib
matplotlib.use('tkagg')
import matplotlib.pyplot as plt
import yaml

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--data_path", type=str)
    parser.add_argument("--config_path", type=str)
    args = parser.parse_args()

    # =========================================================================
    # Read Config File
    # =========================================================================
    with open(args.config_path) as f:
        config = yaml.safe_load(f)
        action_scale = np.array(config['test_configuration']['action_scale'])
        action_lower_bound = np.array(config['test_configuration']['action_lower_bound'])
        action_upper_bound = np.array(config['test_configuration']['action_upper_bound'])
        obs_lower_bound = np.array(config['test_configuration']['terminate_obs_lower_bound'])
        obs_upper_bound = np.array(config['test_configuration']['terminate_obs_upper_bound'])
    action_lower_bound = action_lower_bound * action_scale
    action_upper_bound = action_upper_bound * action_scale

    # =========================================================================
    # Read Data
    # =========================================================================
    data = np.load(args.data_path)
    obs = data['obs']
    ret = data['ret']
    rew = data['rew']
    mask = data['mask']
    action = data['action']
    action *= action_scale
    value = data['value']
    nsteps = data['n_steps'][0]
    done_idx = np.nonzero(mask)[0]
    additional_done = np.array([0])

    n_data = obs.shape[0]
    n_obs = obs.shape[1]
    n_action = action.shape[1]

    while(True):
        additional_done[0] += nsteps
        if done_idx.size == 0:
            done_idx = np.hstack([done_idx, additional_done])
        if additional_done[0] < done_idx[-1]:
            done_idx = np.hstack([done_idx, additional_done])
        elif additional_done[0] < n_data:
            done_idx = np.hstack([done_idx, additional_done])
        else:
            break

    # =========================================================================
    # Plot
    # =========================================================================
    plt.close('all')
    f_obs, ax_obs = plt.subplots(n_obs, sharex=True)
    for i in range(n_obs):
        ax_obs[i].plot(obs[:, i], '--.')
        ax_obs[i].axhline(y=obs_upper_bound[i], color='red', linestyle='-')
        ax_obs[i].axhline(y=obs_lower_bound[i], color='red', linestyle='-')
        ax_obs[i].grid()
        for j in done_idx:
            ax_obs[i].axvline(x=(j+1), color='indigo', linestyle='-')
    f_obs.suptitle("obs")

    f_ret, ax_ret = plt.subplots()
    ax_ret.plot(ret[:], '--.')
    ax_ret.grid()
    for j in done_idx:
        ax_ret.axvline(x=(j+1), color='indigo', linestyle='-')
    f_ret.suptitle("ret")

    if n_action == 1:
        f_act, ax_act= plt.subplots()
        ax_act.plot(action[:], '--.')
        ax_act.axhline(y=action_upper_bound, color='red', linestyle='-')
        ax_act.axhline(y=action_lower_bound, color='red', linestyle='-')
        ax_act.grid()
        for j in done_idx:
            ax_act.axvline(x=(j+1), color='indigo', linestyle='-')
    else:
        f_act, ax_act= plt.subplots(n_action, sharex=True)
        for i in range(n_action):
            ax_act[i].plot(action[:, i], '--.')
            ax_act[i].axhline(y=action_upper_bound[i], color='red', linestyle='-')
            ax_act[i].axhline(y=action_lower_bound[i], color='red', linestyle='-')
            ax_act[i].grid()
            for j in done_idx:
                ax_act[i].axvline(x=(j+1), color='indigo', linestyle='-')
    f_act.suptitle("act")

    f_val, ax_val = plt.subplots()
    ax_val.plot(value[:], '--.')
    ax_val.grid()
    for j in done_idx:
        ax_val.axvline(x=(j+1), color='indigo', linestyle='-')
    f_val.suptitle("val")

    f_rew, ax_rew = plt.subplots()
    ax_rew.plot(rew[:], '--.')
    ax_rew.grid()
    for j in done_idx:
        ax_rew.axvline(x=(j+1), color='indigo', linestyle='-')
    f_rew.suptitle("rew")

    plt.show()
