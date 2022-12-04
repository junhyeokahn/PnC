import numpy as np
import matplotlib.pyplot as plt
import yaml, math, matplotlib
import pickle

matplotlib.use('TkAgg')
import matplotlib.patches as patches
import matplotlib.gridspec as gridspec
from mpl_toolkits.mplot3d import Axes3D


def set_axes_equal(ax):
    '''Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc..  This is one possible solution to Matplotlib's
    ax.set_aspect('equal') and ax.axis('equal') not working for 3D.
    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    '''

    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5 * max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])


'Conversion from quaternion to rotation matrix'


def quat2mat(quat):
    quat = np.squeeze(np.asarray(quat))
    w, x, y, z = quat
    return np.matrix([[
        1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w
    ], [
        2 * x * y + 2 * z * w, 1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * x * w
    ], [
        2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x * x - 2 * y * y
    ]])


def plot_foot(ax, pos, ori, color, text):
    foot_half_len = 0.11
    foot_half_wid = 0.04
    foot_half_len_ctrl = 0.08
    foot_half_wid_ctrl = 0.03
    if text:
        ax.text(pos[0], pos[1] + 0.03, pos[2] + 0.05, text, color=color)
    rmat = quat2mat(ori)
    normal = np.array([rmat[0, 2], rmat[1, 2], rmat[2, 2]])
    d = -pos.dot(normal)
    xx, yy = np.meshgrid(np.linspace(-foot_half_len, foot_half_len, 2),
                         np.linspace(-foot_half_wid, foot_half_wid, 2))
    xx_ctrl, yy_ctrl = np.meshgrid(
        np.linspace(-foot_half_len_ctrl, foot_half_len_ctrl, 2),
        np.linspace(-foot_half_wid_ctrl, foot_half_wid_ctrl, 2))
    xx, yy = np.einsum('ji, mni->jmn', rmat[0:2, 0:2], np.dstack([xx, yy]))
    xx += pos[0]
    yy += pos[1]
    xx_ctrl, yy_ctrl = np.einsum('ji, mni->jmn', rmat[0:2, 0:2],
                                 np.dstack([xx_ctrl, yy_ctrl]))
    xx_ctrl += pos[0]
    yy_ctrl += pos[1]
    z = (-normal[0] * xx - normal[1] * yy - d) * 1. / normal[2]
    if text:
        ax.plot_wireframe(xx, yy, z, color=color, linewidth=1.5)
        ax.plot_surface(xx, yy, z, edgecolors=color, color=color, alpha=0.5)
        ax.scatter(xs=pos[0],
                   ys=pos[1],
                   zs=pos[2],
                   zdir='z',
                   s=50,
                   c=color,
                   depthshade=True)
    else:
        ax.plot(xx, yy, color=color, linewidth=1.5)
        ax.plot(np.transpose(xx), np.transpose(yy), color=color, linewidth=1.5)
        ax.plot(xx_ctrl, yy_ctrl, color=color, linewidth=1.5, alpha=0.2)
        ax.plot(np.transpose(xx_ctrl),
                np.transpose(yy_ctrl),
                color=color,
                linewidth=1.5,
                alpha=0.2)


def main(args):
    file = args.file
    b_plot_2D = args.b_plot_2D

    with open(file, 'r') as stream:
        try:
            cfg = yaml.load(stream, Loader=yaml.FullLoader)
            initial_time = cfg["temporal_parameters"]["initial_time"]
            final_time = cfg["temporal_parameters"]["final_time"]
            time_step = cfg["temporal_parameters"]["time_step"]
            t_ds = cfg["temporal_parameters"]["t_ds"]
            t_ss = cfg["temporal_parameters"]["t_ss"]
            t_transfer = cfg["temporal_parameters"]["t_transfer"]

            curr_rfoot_contact_pos = np.array(
                cfg["contact"]["curr_right_foot"]["pos"])
            curr_rfoot_contact_ori = np.array(
                cfg["contact"]["curr_right_foot"]["ori"])
            curr_lfoot_contact_pos = np.array(
                cfg["contact"]["curr_left_foot"]["pos"])
            curr_lfoot_contact_ori = np.array(
                cfg["contact"]["curr_left_foot"]["ori"])
            rfoot_contact_pos = np.array(cfg["contact"]["right_foot"]["pos"])
            rfoot_contact_ori = np.array(cfg["contact"]["right_foot"]["ori"])
            assert rfoot_contact_pos.shape[0] == rfoot_contact_ori.shape[0]
            lfoot_contact_pos = np.array(cfg["contact"]["left_foot"]["pos"])
            lfoot_contact_ori = np.array(cfg["contact"]["left_foot"]["ori"])
            assert lfoot_contact_pos.shape[0] == lfoot_contact_ori.shape[0]

            t = np.array(cfg["reference"]["time"])
            dcm_pos_ref = np.array(cfg["reference"]["dcm_pos"])
            dcm_vel_ref = np.array(cfg["reference"]["dcm_vel"])
            com_pos_ref = np.array(cfg["reference"]["com_pos"])
            com_vel_ref = np.array(cfg["reference"]["com_vel"])
            vrp_ref = np.array(cfg["reference"]["vrp"])

        except yaml.YAMLError as exc:
            print(exc)

    # ==========================================================================
    # Get actual data
    # ==========================================================================
    time = []
    com_pos_act = []
    icp_act = []
    icp_des = []
    cmp_des = []
    lfoot_pos_act = []
    rfoot_pos_act = []
    with open('experiment_data/pnc.pkl', 'rb') as file:
        while True:
            try:
                d = pickle.load(file)
                time.append(d['time'])
                com_pos_act.append(d['task_com_pos'])
                icp_act.append(d['icp'])
                icp_des.append(d['icp_des'])
                lfoot_pos_act.append(d['task_lfoot_lin_pos'])
                rfoot_pos_act.append(d['task_rfoot_lin_pos'])
                cmp_des.append(d['des_cmp'])
            except EOFError:
                break

    com_pos_act = np.stack(com_pos_act, axis=0)
    icp_act = np.stack(icp_act, axis=0)
    icp_des = np.stack(icp_des, axis=0)
    cmp_des = np.stack(cmp_des, axis=0)
    lfoot_pos_act = np.stack(lfoot_pos_act, axis=0)
    rfoot_pos_act = np.stack(rfoot_pos_act, axis=0)

    ## get times of inteterest
    initial_step_idx = np.where(
        np.abs(np.array(time) -
               initial_time) == np.min(np.abs(np.array(time) -
                                              initial_time)))[0][0]

    time_at_swing = initial_time + t_transfer + 1.5 * t_ds
    time_at_touch_down = time_at_swing + t_ss

    first_step_idx = np.where(
        np.abs(np.array(time) - time_at_touch_down) == np.min(
            np.abs(np.array(time) - time_at_touch_down)))[0][0]
    first_liftoff_idx = np.where(
        np.abs(np.array(time) -
               time_at_swing) == np.min(np.abs(np.array(time) -
                                               time_at_swing)))[0][0]

    landing_rfoot_pos = rfoot_pos_act[first_step_idx]
    landing_lfoot_pos = lfoot_pos_act[first_step_idx]
    liftoff_icp_pos_act = icp_act[first_liftoff_idx]
    liftoff_icp_pos_des = icp_des[first_liftoff_idx]

    line_styles = {0: '--', 1: '-', 2: '--', 3: '-'}
    colors = {0: 'red', 1: 'magenta', 2: 'blue', 3: 'cyan'}
    line_colors = {
        0: 'cornflowerblue',
        1: 'sandybrown',
        2: 'seagreen',
        3: 'gold'
    }
    # ==========================================================================
    # Plot Motion
    # ==========================================================================
    offset = 0.05
    axis_tick_size = 10
    axis_label_size = 14
    axis_tick_color = '#434440'
    axis_label_color = '#373834'
    comref_linewidth = 2
    comref_linecolor = 'darkorange'
    dcmref_linewidth = 4
    dcmref_linecolor = 'cornflowerblue'

    fig1 = plt.figure()
    if b_plot_2D:
        plt.plot(com_pos_ref[:, 0],
                 com_pos_ref[:, 1],
                 linewidth=comref_linewidth,
                 color=comref_linecolor)
        plt.plot(dcm_pos_ref[:, 0],
                 dcm_pos_ref[:, 1],
                 linewidth=dcmref_linewidth,
                 linestyle=line_styles[0],
                 color=dcmref_linecolor)
        plt.grid()

        # plot ICP during stepping time
        plt.plot(icp_act[0, 0], icp_act[0, 1], 'ms', label='ICP start')
        plt.plot(icp_act[first_step_idx, 0],
                 icp_act[first_step_idx, 1],
                 'co',
                 label='ICP end')
        plt.plot(icp_act[initial_step_idx:first_step_idx, 0],
                 icp_act[initial_step_idx:first_step_idx, 1],
                 color='c')

        plt.plot(liftoff_icp_pos_act[0],
                 liftoff_icp_pos_act[1],
                 'c*',
                 label='ICP liftoff(act)')
        plt.plot(liftoff_icp_pos_des[0],
                 liftoff_icp_pos_des[1],
                 'k*',
                 label='ICP liftoff(des)')

        # plot cmp during single support
        plt.plot(cmp_des[first_liftoff_idx:first_step_idx, 0],
                 cmp_des[first_liftoff_idx:first_step_idx, 1],
                 'pink',
                 label='CMP des (ss)')

        # plt.plot(com_pos_act[0, 0], com_pos_act[0, 1], 'ms', label='CoM start')
        # plt.plot(com_pos_act[-1, 0], com_pos_act[-1, 1], 'co', label='CoM end')
        # plt.plot(com_pos_act[:, 0], com_pos_act[:, 1], color='k')

        # plot feet position
        plt.plot(lfoot_pos_act[initial_step_idx, 0],
                 lfoot_pos_act[initial_step_idx, 1],
                 'rs',
                 label='LF act',
                 alpha=0.2)
        plt.plot(rfoot_pos_act[initial_step_idx, 0],
                 rfoot_pos_act[initial_step_idx, 1],
                 'bs',
                 label='RF act',
                 alpha=0.2)

        # plot footstep plan
        plt.plot(curr_lfoot_contact_pos[0, 0],
                 curr_lfoot_contact_pos[0, 1],
                 'rs',
                 label='initLF')
        plt.plot(curr_rfoot_contact_pos[0, 0],
                 curr_rfoot_contact_pos[0, 1],
                 'bs',
                 label='initRF')

        # plot planned and actual feet position at landing
        if rfoot_contact_pos.shape[0] == 0:
            plt.plot(landing_lfoot_pos[0],
                     landing_lfoot_pos[1],
                     'ro',
                     label='LF act step',
                     alpha=0.2)
            plt.plot(lfoot_contact_pos[0, 0],
                     lfoot_contact_pos[0, 1],
                     'darkorange',
                     marker='s',
                     label='2ndLF(planned)')
        else:
            plt.plot(landing_rfoot_pos[0],
                     landing_rfoot_pos[1],
                     'bo',
                     label='RF act step',
                     alpha=0.2)
            plt.plot(rfoot_contact_pos[0, 0],
                     rfoot_contact_pos[0, 1],
                     'cs',
                     label='2ndRF(planned)')

        # plot feet
        ax = plt.gca()
        plot_foot(ax, np.squeeze(curr_lfoot_contact_pos),
                  np.squeeze(curr_lfoot_contact_ori), colors[0], False)
        plot_foot(ax, np.squeeze(curr_rfoot_contact_pos),
                  np.squeeze(curr_rfoot_contact_ori), colors[1], False)

        plt.xlabel('x (m)')
        plt.ylabel('y (m)')
        plt.legend(bbox_to_anchor=(1.3, 1), loc='upper right')
        # ax = plt.gca()
        ax.set_aspect('equal')
    else:
        com_motion = Axes3D(fig1)

        # plot com
        com_motion.plot(xs=com_pos_ref[:, 0],
                        ys=com_pos_ref[:, 1],
                        zs=com_pos_ref[:, 2],
                        linewidth=comref_linewidth,
                        color=comref_linecolor)
        com_motion.plot(xs=dcm_pos_ref[:, 0],
                        ys=dcm_pos_ref[:, 1],
                        zs=dcm_pos_ref[:, 2],
                        linewidth=dcmref_linewidth,
                        linestyle=line_styles[0],
                        color=dcmref_linecolor)

        # plot foot
        plot_foot(com_motion, np.squeeze(curr_rfoot_contact_pos),
                  np.squeeze(curr_rfoot_contact_ori), colors[0], "InitRF")
        plot_foot(com_motion, np.squeeze(curr_lfoot_contact_pos),
                  np.squeeze(curr_lfoot_contact_ori), colors[1], "InitLF")
        for i, (pos,
                ori) in enumerate(zip(rfoot_contact_pos, rfoot_contact_ori)):
            plot_foot(com_motion, pos, ori, colors[0], "RF" + str(i))
        for i, (pos,
                ori) in enumerate(zip(lfoot_contact_pos, lfoot_contact_ori)):
            plot_foot(com_motion, pos, ori, colors[1], "LF" + str(i))

        com_motion.tick_params(labelsize=axis_tick_size,
                               colors=axis_tick_color)
        com_motion.set_xlabel("x",
                              fontsize=axis_label_size,
                              color=axis_label_color)
        com_motion.set_ylabel("y",
                              fontsize=axis_label_size,
                              color=axis_label_color)
        com_motion.set_zlabel("z",
                              fontsize=axis_label_size,
                              color=axis_label_color)
        set_axes_equal(com_motion)

    # ==========================================================================
    # Plot Trajectory
    # ==========================================================================

    plt.show()


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--file", type=str)
    parser.add_argument("--b_plot_2D", default=False, action='store_true')
    args = parser.parse_args()
    main(args)
