'''
 Copyright [2017] Max Planck Society. All rights reserved.
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
'''

import numpy as np
from scipy import signal
import yaml, math, matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.gridspec as gridspec
from mpl_toolkits.mplot3d import Axes3D

'Conversion from quaternion to rotation matrix'
def quat2mat(quat):
    quat = np.squeeze(np.asarray(quat))
    w, x, y, z = quat
    return np.matrix([
            [1 - 2*y*y-2*z*z, 2*x*y - 2*z*w, 2*x*z+2*y*w],
            [2*x*y+2*z*w, 1-2*x*x-2*z*z, 2*y*z-2*x*w],
            [2*x*z-2*y*w, 2*y*z+2*x*w, 1-2*x*x-2*y*y]])

'Base class to define a contact state'
class CntState:
    def __init__(self, pars):
        self.tini = pars[0]
        self.tend = pars[1]
        self.pos  = np.matrix(pars[2:5]).transpose()
        self.ori = quat2mat(pars[5:9])
        
    def display(self):
        print "time", self.tini, "--", self.tend
        print "pos ", self.pos
        print "ori ", self.ori
        print "---------------------------------"

'Base class to show motion'
class Graphics:
    'Initialization'
    def __init__(self):
        self.effs_cnts = list()
        
    'Helper functions'
    def id_to_eff_str_map(self, id):
        if (id == 0):
            return 'rf'
        elif (id == 1):
            return 'lf'
        elif (id == 2):
            return 'rh'
        elif (id == 3):
            return 'lh'
        else:
            print 'Err: id not handled'
            
    def nice_timesteps_plot(self, axis, timesteps1, timesteps2):
        axis.plot(timesteps1, color='royalblue')
        axis.plot(timesteps2, color='lightseagreen')
 
        shading = 0.2
        tick_size = 10
        grid_color = '#ACADA9'
        axis_color = '#8C8E87'
        tick_color = '#434440'
        ylim = [0.05, 0.30]
        xlim = [1,len(timesteps1)-1]
        linecolors = {0: 'cornflowerblue', 1: 'darkorange', 2: 'seagreen', 3: 'gold'}
 
        time_spec = np.squeeze(np.asarray(np.linspace(1, len(timesteps1), len(timesteps1))))
        time_vec  = np.squeeze(np.asarray(np.linspace(self.time_step, self.time_horizon, len(timesteps1))))
        for eef_id in range(0,4):
            trueData = np.zeros(len(timesteps1))
            for cnt_id in range(0,len(self.effs_cnts[eef_id])):
                for id in range(0,len(trueData)):
                    if (time_vec[id] >= self.effs_cnts[eef_id][cnt_id].tini and time_vec[id] < self.effs_cnts[eef_id][cnt_id].tend):
                        trueData[id] = 1
            axis.fill_between(time_spec, ylim[0], ylim[1], where=trueData>0.5, facecolor=linecolors[eef_id], alpha=shading)
        
        axis.grid(True)
        axis.set_xlim(xlim)
        axis.set_ylim(ylim)
        for item in axis.get_xgridlines():
            item.set_color(grid_color)
        for item in axis.get_ygridlines():
            item.set_color(grid_color)

        axis.yaxis.tick_right()
        for t in axis.axes.get_yticklabels():
            t.set_horizontalalignment('right')  
            t.set_x(1.04)
        axis.spines["top"].set_color(axis_color)
        axis.spines["left"].set_color(axis_color)
        axis.spines["right"].set_color(axis_color)
        axis.spines["bottom"].set_color(axis_color)
        axis.tick_params(labelsize=tick_size, colors=tick_color)
        axis.set_ylabel(r'$\Delta t$', fontsize=10, color='#373834')

    def nice_mom_plot(self, axis, datax, datay, line_color, line_width, line_style, grid_color, axis_color, xlim, ylim, xmsg, ymsg, label_size, label_color, show_xlabel, tick_size, tick_color, remove_Xticks, remove_Yticks, linecolors, shading, linelabel, showLabel):
        axis.plot(datax, datay, color=line_color, linewidth=line_width, linestyle=line_style, label=linelabel)
        
        if (showLabel is True):
            leg = axis.legend(bbox_to_anchor=(0., -0.30, 1., .102), loc=3, ncol=2, mode="expand", borderaxespad=0.)
        
        time_vec = np.squeeze(np.asarray(np.linspace(self.time_step, self.time_horizon, len(datax))))
        for eef_id in range(0,4):
            trueData = np.zeros(len(datax))
            for cnt_id in range(0,len(self.effs_cnts[eef_id])):
                for id in range(0,len(trueData)):
                    if (time_vec[id] >= self.effs_cnts[eef_id][cnt_id].tini and time_vec[id] < self.effs_cnts[eef_id][cnt_id].tend):
                        trueData[id] = 1
            axis.fill_between(datax, ylim[0], ylim[1], where=trueData>0.5, facecolor=linecolors[eef_id], alpha=shading)
    
        axis.grid(True)
        axis.set_xlim(xlim)
        axis.set_ylim(ylim)
        for item in axis.get_xgridlines():
            item.set_color(grid_color)
        for item in axis.get_ygridlines():
            item.set_color(grid_color)
        if (remove_Xticks is True):
            axis.axes.xaxis.set_ticklabels([])
        if (remove_Yticks is True):
            axis.axes.yaxis.set_ticklabels([])
        else:
            axis.yaxis.tick_right()
            for t in axis.axes.get_yticklabels():
                t.set_horizontalalignment('right')  
                t.set_x(1.15)

        axis.spines["top"].set_color(axis_color)
        axis.spines["left"].set_color(axis_color)
        axis.spines["right"].set_color(axis_color)
        axis.spines["bottom"].set_color(axis_color)
        axis.tick_params(labelsize=tick_size, colors=tick_color)
        axis.set_ylabel(ymsg, fontsize=label_size, color=label_color)
        if (show_xlabel is True):
            axis.set_xlabel(xmsg, fontsize=label_size, color=label_color)
    
    def nice_frc_plot(self, axis, datax, datay, linecolors, linestyles, linelabels, addlabels, line_width, grid_color, tick_size, tick_color, axis_color, xlim, ylim, xmsg, ymsg, label_size, label_color, legend_textsize, legend_color, legend_fcolor, legend_flinewidth, legend_fbgcolor, shading):
        axis.grid(True)
        axis.set_xlim(xlim)
        axis.set_ylim(ylim)
        for item in axis.get_xgridlines():
            item.set_color(grid_color)
        for item in axis.get_ygridlines():
            item.set_color(grid_color)
        axis.spines["top"].set_color(axis_color)
        axis.spines["left"].set_color(axis_color)
        axis.spines["right"].set_color(axis_color)
        axis.spines["bottom"].set_color(axis_color)
        axis.tick_params(labelsize=tick_size, colors=tick_color)
        axis.set_xlabel(xmsg, fontsize=label_size, color=label_color)
        axis.set_ylabel(ymsg, fontsize=label_size, color=label_color)
        axis.yaxis.tick_right()
        for t in axis.axes.get_yticklabels():
            t.set_horizontalalignment('right')  
            t.set_x(1.15)
        if (addlabels is True):
            axis.axes.xaxis.set_ticklabels([])
        
        time_vec = np.squeeze(np.asarray(np.linspace(self.time_step, self.time_horizon, len(datax))))
        for eef_id in range(0,4):
            trueData = np.zeros(len(datax))
            for cnt_id in range(0,len(self.effs_cnts[eef_id])):
                for id in range(0,len(trueData)):
                    if (time_vec[id] >= self.effs_cnts[eef_id][cnt_id].tini and time_vec[id] < self.effs_cnts[eef_id][cnt_id].tend):
                        trueData[id] = 1
            axis.fill_between(datax, ylim[0], ylim[1], where=trueData>0.5, facecolor=linecolors[eef_id], alpha=shading)

        
        for id in range(0, len(datay)):
            if (addlabels is False):
                axis.plot(datax, datay[id], color=linecolors[id], linewidth=line_width, linestyle=linestyles[id])
            else:        
                axis.plot(datax, datay[id], label=linelabels[id], color=linecolors[id], linewidth=line_width, linestyle=linestyles[id])

        if (addlabels is True):
            leg = axis.legend(bbox_to_anchor=(0., -0.30, 1., .102), loc=3, ncol=4, mode="expand", borderaxespad=0.)

    'Function to show the motion'
    def show_motion(self, cfg_file):
        'Read data from file'
        with open(cfg_file, 'r') as stream:
            try:
                cfg_pars = yaml.load(stream)
                self.time_step = cfg_pars['dynopt_params']['time_step']
                self.robot_mass = cfg_pars['dynopt_params']['robot_mass']
                self.n_act_eefs = cfg_pars['dynopt_params']['n_act_eefs']
                self.time_horizon = cfg_pars['dynopt_params']['time_horizon']
                self.time_vec = np.matrix(cfg_pars['dynopt_params']['time_vec'])
                
                self.com = np.matrix(cfg_pars['dynopt_params']['com_motion'])
                self.com_ref = np.matrix(cfg_pars['dynopt_params']['com_motion_ref'])
                self.lmom = np.matrix(cfg_pars['dynopt_params']['lin_mom'])/self.robot_mass
                self.amom = np.matrix(cfg_pars['dynopt_params']['ang_mom'])/self.robot_mass
                self.lmom_ref = np.matrix(cfg_pars['dynopt_params']['lin_mom_ref'])/self.robot_mass
                self.amom_ref = np.matrix(cfg_pars['dynopt_params']['ang_mom_ref'])/self.robot_mass
                self.eef_frcs = list()
                for eff_id in range(0, self.n_act_eefs):
                    self.eef_frcs.insert(eff_id, np.matrix(cfg_pars['dynopt_params']['eef_frc_'+str(eff_id)]))    

                for eff_id in range(0, 4):
                    eef_cnts = list()
                    for cnt_id in range(0, cfg_pars['cntopt_params']['num_contacts'][eff_id]):
                        eef_cnts.append(CntState(cfg_pars['cntopt_params']['eefcnt_'+self.id_to_eff_str_map(eff_id)]['cnt'+str(cnt_id)]))
                    self.effs_cnts.append(eef_cnts)                    

            except yaml.YAMLError as exc:
                print(exc)

        'Build arrays of data to be displayed' 
        comx = np.squeeze(np.asarray(self.com[0,:]))
        comy = np.squeeze(np.asarray(self.com[1,:]))
        comz = np.squeeze(np.asarray(self.com[2,:]))
        lmomx = np.squeeze(np.asarray(self.lmom[0,:]))
        lmomy = np.squeeze(np.asarray(self.lmom[1,:]))
        lmomz = np.squeeze(np.asarray(self.lmom[2,:]))
        amomx = np.squeeze(np.asarray(self.amom[0,:]))
        amomy = np.squeeze(np.asarray(self.amom[1,:]))
        amomz = np.squeeze(np.asarray(self.amom[2,:]))

        comrefx = np.squeeze(np.asarray(self.com_ref[0,:]))
        comrefy = np.squeeze(np.asarray(self.com_ref[1,:]))
        comrefz = np.squeeze(np.asarray(self.com_ref[2,:]))
        lmomrefx = np.squeeze(np.asarray(self.lmom_ref[0,:]))
        lmomrefy = np.squeeze(np.asarray(self.lmom_ref[1,:]))
        lmomrefz = np.squeeze(np.asarray(self.lmom_ref[2,:]))
        amomrefx = np.squeeze(np.asarray(self.amom_ref[0,:]))
        amomrefy = np.squeeze(np.asarray(self.amom_ref[1,:]))
        amomrefz = np.squeeze(np.asarray(self.amom_ref[2,:]))
        
        time = np.squeeze(np.asarray(self.time_vec[0,:]))
        timesteps1 = np.squeeze(np.asarray( np.zeros((len(time),1)) ))
        timesteps2 = np.squeeze(np.asarray( np.zeros((len(time),1)) ))

        for id in range (0,len(time)):
            timesteps1[id] = self.time_step
            if id is 0:
                timesteps2[id] = time[id]
            else:
                timesteps2[id] = time[id]-time[id-1]

        frcs_x = list()
        frcs_y = list()
        frcs_z = list()
        for eff_id in range(0,self.n_act_eefs):
            frcs_x.insert(eff_id, np.squeeze(np.asarray(self.eef_frcs[eff_id][0,:])))
            frcs_y.insert(eff_id, np.squeeze(np.asarray(self.eef_frcs[eff_id][1,:])))
            frcs_z.insert(eff_id, np.squeeze(np.asarray(self.eef_frcs[eff_id][2,:])))

        'Figure1: Center of Mass motion'
        offset = 0.05
        com_linewidth = 2
        comref_linewidth = 2
        com_axis_tick_size = 10
        com_axis_label_size = 14
        com_axis_tick_color = '#434440'
        com_axis_label_color = '#373834'
        com_linecolor = 'cornflowerblue'
        comref_linecolor = 'darkorange'
        linestyles = {0:'--', 1:'-', 2: '--', 3: '-'}
        endeffectors = {0: 'RF', 1: 'LF', 2: 'RH', 3: 'LH'}
        colors = {0: 'red', 1: 'magenta', 2: 'blue', 3: 'cyan'}
        linecolors = {0: 'cornflowerblue', 1: 'sandybrown', 2: 'seagreen', 3: 'gold'}
        
        'Figure1: Center of mass motion'
        fig1 = plt.figure()
        CoM_motion = Axes3D(fig1)
        CoM_motion.plot(xs=comx, ys=comy, zs=comz, linewidth=com_linewidth, color=com_linecolor)
        CoM_motion.plot(xs=comrefx, ys=comrefy, zs=comrefz, linewidth=comref_linewidth, color=comref_linecolor, linestyle='--')
        for eff_id in range(0, 4):
            for cnt_id in range(0, len(self.effs_cnts[eff_id])):
                cnt_pos = self.effs_cnts[eff_id][cnt_id].pos
                cnt_pos = np.squeeze(np.asarray(cnt_pos))
                cnt_ori = self.effs_cnts[eff_id][cnt_id].ori[:,2]
                CoM_motion.text(cnt_pos[0]+offset, cnt_pos[1]+offset, cnt_pos[2]+offset, endeffectors[eff_id]+str(cnt_id), color=colors[eff_id])
                  
                point  = np.array([cnt_pos[0], cnt_pos[1], cnt_pos[2]])
                normal = np.array([cnt_ori[0,0], cnt_ori[1,0], cnt_ori[2,0]])
                d = -point.dot(normal)
                xx, yy = np.meshgrid(np.linspace(cnt_pos[0]-0.10,cnt_pos[0]+0.10,2), np.linspace(cnt_pos[1]-0.05,cnt_pos[1]+0.05,2))
                z = (-normal[0] * xx - normal[1] * yy - d) * 1. /normal[2]
                CoM_motion.plot_wireframe(xx, yy, z, color=colors[eff_id], linewidth=1.5)
                CoM_motion.plot_surface(xx, yy, z, edgecolors=colors[eff_id], color=colors[eff_id], alpha = 0.5)
                CoM_motion.scatter(xs=cnt_pos[0], ys=cnt_pos[1], zs=cnt_pos[2], zdir='z', s=50, c=colors[eff_id], depthshade=True)
         
        CoM_motion.tick_params(labelsize=com_axis_tick_size, colors=com_axis_tick_color)
        CoM_motion.set_xlabel('Forward direction', fontsize=com_axis_label_size, color=com_axis_label_color)
        CoM_motion.set_ylabel('Lateral direction', fontsize=com_axis_label_size, color=com_axis_label_color)
        CoM_motion.set_zlabel('Vertical direction', fontsize=com_axis_label_size, color=com_axis_label_color)        

        'Figure2: Linear and angular momenta, forces and timesteps'
        mom_line_width = 2
        momref_line_width = 2
        mom_axis_tick_size = 10
        mom_axis_label_size = 10
        
        lmomx_y_size = [-50.0/self.robot_mass, 50.0/self.robot_mass]
        lmomy_y_size = [-25.0/self.robot_mass, 25.0/self.robot_mass]
        lmomz_y_size = [-25.0/self.robot_mass, 25.0/self.robot_mass]
        amom_y_size  = [-10.0/self.robot_mass, 10.0/self.robot_mass]
        mom_x_size = [0., time[len(time)-1]]
        mom_axis_color = '#8C8E87'
        mom_grid_color = '#ACADA9'
        mom_axis_tick_color = '#434440'
        mom_axis_label_color = '#373834'
        mom_line_color = 'cornflowerblue'
        momref_line_color = 'darkorange'
        mom_line_styles = {0:'-', 1:'--'}
        shading = 0.1

        frc_line_width = 2
        frc_axis_tick_size = 10
        frc_legend_textsize = 10
        frc_axis_label_size = 10
        frc_x_ysize = [-0.40, 0.40]
        frc_y_ysize = [-0.20, 0.20]
        frc_z_ysize = [-0.01, 1.02]
        frc_legend_frame_linewidth = 1
        frc_xsize = [0., time[len(time)-1]]
        frc_grid_color = '#ACADA9'
        frc_axis_color = '#8C8E87'
        frc_legend_color = '#8C8E87'
        frc_axis_tick_color = '#434440'
        frc_axis_label_color = '#373834'
        frc_legend_frame_color = '#ACADA9'
        frc_legend_frame_background_color = '#EAECEF'
        
        fig1 = plt.figure(figsize=(10,5))
        gs = gridspec.GridSpec(4, 3)
        
        lmom_x1 = plt.subplot(gs[0,0])
        lmom_y1 = plt.subplot(gs[0,1])
        lmom_z1 = plt.subplot(gs[0,2])
        amom_x1 = plt.subplot(gs[1,0])
        amom_y1 = plt.subplot(gs[1,1])
        amom_z1 = plt.subplot(gs[1,2])
        frc_x1  = plt.subplot(gs[2,0])
        frc_y1  = plt.subplot(gs[2,1])
        frc_z1  = plt.subplot(gs[2,2])
        tplt_1  = plt.subplot(gs[3,:])

        self.nice_mom_plot(lmom_x1, time, lmomx, mom_line_color, mom_line_width, mom_line_styles[0], mom_grid_color, mom_axis_color, mom_x_size, lmomx_y_size, r"""Time $[sec]$""", r"""LinX""", mom_axis_label_size, mom_axis_label_color, False, mom_axis_tick_size, mom_axis_tick_color, True, False, linecolors, shading, linelabel='DynMom', showLabel=False)
        self.nice_mom_plot(lmom_y1, time, lmomy, mom_line_color, mom_line_width, mom_line_styles[0], mom_grid_color, mom_axis_color, mom_x_size, lmomy_y_size, r"""Time $[sec]$""", r"""LinY""", mom_axis_label_size, mom_axis_label_color, False, mom_axis_tick_size, mom_axis_tick_color, True, False, linecolors, shading, linelabel='DynMom', showLabel=False)
        self.nice_mom_plot(lmom_z1, time, lmomz, mom_line_color, mom_line_width, mom_line_styles[0], mom_grid_color, mom_axis_color, mom_x_size, lmomz_y_size, r"""Time $[sec]$""", r"""LinZ""", mom_axis_label_size, mom_axis_label_color, False, mom_axis_tick_size, mom_axis_tick_color, True, False, linecolors, shading, linelabel='DynMom', showLabel=False)
        self.nice_mom_plot(amom_x1, time, amomx, mom_line_color, mom_line_width, mom_line_styles[0], mom_grid_color, mom_axis_color, mom_x_size, amom_y_size, r"""Time $[sec]$""", r"""AngX""", mom_axis_label_size, mom_axis_label_color, False, mom_axis_tick_size, mom_axis_tick_color, True, False, linecolors, shading, linelabel='DynMom', showLabel=False)
        self.nice_mom_plot(amom_y1, time, amomy, mom_line_color, mom_line_width, mom_line_styles[0], mom_grid_color, mom_axis_color, mom_x_size, amom_y_size, r"""Time $[sec]$""", r"""AngY""", mom_axis_label_size, mom_axis_label_color, False, mom_axis_tick_size, mom_axis_tick_color, True, False, linecolors, shading, linelabel='DynMom', showLabel=False)
        self.nice_mom_plot(amom_z1, time, amomz, mom_line_color, mom_line_width, mom_line_styles[0], mom_grid_color, mom_axis_color, mom_x_size, amom_y_size, r"""Time $[sec]$""", r"""AngZ""", mom_axis_label_size, mom_axis_label_color, False, mom_axis_tick_size, mom_axis_tick_color, True, False, linecolors, shading, linelabel='DynMom', showLabel=False)

        self.nice_mom_plot(lmom_x1, time, lmomrefx, momref_line_color, momref_line_width, mom_line_styles[1], mom_grid_color, mom_axis_color, mom_x_size, lmomx_y_size, r"""Time $[sec]$""", r"""LinX""", mom_axis_label_size, mom_axis_label_color, False, mom_axis_tick_size, mom_axis_tick_color, True, False, linecolors, shading, linelabel='KinMom', showLabel=True)
        self.nice_mom_plot(lmom_y1, time, lmomrefy, momref_line_color, momref_line_width, mom_line_styles[1], mom_grid_color, mom_axis_color, mom_x_size, lmomy_y_size, r"""Time $[sec]$""", r"""LinY""", mom_axis_label_size, mom_axis_label_color, False, mom_axis_tick_size, mom_axis_tick_color, True, False, linecolors, shading, linelabel='KinMom', showLabel=False)
        self.nice_mom_plot(lmom_z1, time, lmomrefz, momref_line_color, momref_line_width, mom_line_styles[1], mom_grid_color, mom_axis_color, mom_x_size, lmomz_y_size, r"""Time $[sec]$""", r"""LinZ""", mom_axis_label_size, mom_axis_label_color, False, mom_axis_tick_size, mom_axis_tick_color, True, False, linecolors, shading, linelabel='KinMom', showLabel=False)
        self.nice_mom_plot(amom_x1, time, amomrefx, momref_line_color, momref_line_width, mom_line_styles[1], mom_grid_color, mom_axis_color, mom_x_size, amom_y_size, r"""Time $[sec]$""", r"""AngX""", mom_axis_label_size, mom_axis_label_color, False, mom_axis_tick_size, mom_axis_tick_color, True, False, linecolors, shading, linelabel='KinMom', showLabel=True)
        self.nice_mom_plot(amom_y1, time, amomrefy, momref_line_color, momref_line_width, mom_line_styles[1], mom_grid_color, mom_axis_color, mom_x_size, amom_y_size, r"""Time $[sec]$""", r"""AngY""", mom_axis_label_size, mom_axis_label_color, False, mom_axis_tick_size, mom_axis_tick_color, True, False, linecolors, shading, linelabel='KinMom', showLabel=False)
        self.nice_mom_plot(amom_z1, time, amomrefz, momref_line_color, momref_line_width, mom_line_styles[1], mom_grid_color, mom_axis_color, mom_x_size, amom_y_size, r"""Time $[sec]$""", r"""AngZ""", mom_axis_label_size, mom_axis_label_color, False, mom_axis_tick_size, mom_axis_tick_color, True, False, linecolors, shading, linelabel='KinMom', showLabel=False)

        self.nice_frc_plot(frc_x1, time, frcs_x, linecolors, linestyles, endeffectors, True, frc_line_width, frc_grid_color, frc_axis_tick_size, frc_axis_tick_color, frc_axis_color, frc_xsize, frc_x_ysize, r"""""", r"""FrcX""", frc_axis_label_size, frc_axis_label_color, frc_legend_textsize, frc_legend_color, frc_legend_frame_color, frc_legend_frame_linewidth, frc_legend_frame_background_color, shading)
        self.nice_frc_plot(frc_y1, time, frcs_y, linecolors, linestyles, endeffectors, False, frc_line_width, frc_grid_color, frc_axis_tick_size, frc_axis_tick_color, frc_axis_color, frc_xsize, frc_y_ysize, r"""""", r"""FrcY""", frc_axis_label_size, frc_axis_label_color, frc_legend_textsize, frc_legend_color, frc_legend_frame_color, frc_legend_frame_linewidth, frc_legend_frame_background_color, shading)
        self.nice_frc_plot(frc_z1, time, frcs_z, linecolors, linestyles, endeffectors, False, frc_line_width, frc_grid_color, frc_axis_tick_size, frc_axis_tick_color, frc_axis_color, frc_xsize, frc_z_ysize, r"""""", r"""FrcZ""", frc_axis_label_size, frc_axis_label_color, frc_legend_textsize, frc_legend_color, frc_legend_frame_color, frc_legend_frame_linewidth, frc_legend_frame_background_color, shading)

        tplt_1.set_title("Timesteps", fontsize=12, color=frc_axis_label_color)
        self.nice_timesteps_plot(tplt_1, timesteps1, timesteps2)

        fig1.subplots_adjust(left=0.03, bottom=0.08, right=0.94, top=0.95, wspace=0.36, hspace=0.65)
        plt.show()
