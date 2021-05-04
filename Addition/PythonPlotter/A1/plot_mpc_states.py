import numpy as np
import matplotlib.pyplot as plt
import yaml, sys, glob

def fetch_yaml():
    if len(sys.argv) <= 2:
        print('\n')
        print('file not specified')
        print('please call function as follows')
        print('python plot_mpc.py <path from current directory to directory containing yaml files> <numbers of yaml files to plot, separated by spaces>')
        print('\n')
        print('for example:')
        print('\n')
        print('python plot_mpc.py ./mpc_io/ 10 11')
        print('\n')
        print('would plot files named ./mpc_io/*_10.yaml and ./mpc_io/*_11.yaml')
        print('if there are multiple files with the same naming scheme in the directory, an error will be thrown')
        print('\n')
        print('please try again')
        print('\n')
        
        fnames = []
        return fnames

    else:
        yaml_dir = sys.argv[1]
        fnames = []
        for i in range(2,len(sys.argv)):
            fname = glob.glob(yaml_dir+'*_'+sys.argv[i]+'.yaml')
            if fname == []:
                print('\n')
                print('no file in '+yaml_dir+' with name '+'*_'+sys.argv[i]+'.yaml')
                print('please check that the directory contains the file you are looking for and that the naming scheme matches')
                print('files should be named ./dir/*_<number>.yaml')
                print('such as ./mpc_io/mpc_io_10.yaml')
                print('note that "*" is the wildcard operator and will fill any available text')
                print('please try again')
                print('\n')

                fnames = []
                break

            elif len(fname) > 1:
                print('\n')
                print('too many files in '+yaml_dir+' with name matching '+'*_'+sys.argv[i]+'.yaml')
                print('please check that the directory contains the file you are looking for and that the naming scheme matches')
                print('files should be named ./dir/*_<number>.yaml')
                print('such as ./mpc_io/mpc_io_10.yaml')
                print('note that "*" is the wildcard operator and will fill any available text')
                print('there should only be one file matching the naming scheme given above')
                print('please try again')
                print('\n')

                fnames = []
                break

            elif len(fname) == 1:
                fnames.append(fname[0])
        
        return fnames



def read_yaml(fnames,num_plans=10):
    if fnames == []:
        print('\n')
        print('error: fnames returns empty list')
        print('check arguments provided in the terminal')

        data = []
        return data

    data = {}
    for fname in fnames:
        fname_dict = {}
        with open(fname, 'r') as f:
            try:
                cfg = yaml.load(f)

                ## TEMPORAL PARAMS
                fname_dict['initial_time'] = cfg['temporal_parameters']['initial_time']
                fname_dict['final_time'] = cfg['temporal_parameters']['final_time']
                fname_dict['time_step'] = cfg['temporal_parameters']['time_step']
                
                ## INPUT
                fname_dict['com_pos'] = np.array(cfg['input']['com_pos'])
                fname_dict['com_vel_body_frame'] = np.array(cfg['input']['com_vel_body_frame'])
                fname_dict['com_rpy_zyx'] = np.array(cfg['input']['com_rpy_zyx'])
                fname_dict['ang_vel'] = np.array(cfg['input']['ang_vel'])
                fname_dict['com_vel_des'] = np.array(cfg['input']['com_vel_des'])
                fname_dict['yaw_vel_des'] = np.array(cfg['input']['yaw_vel_des'])
                fname_dict['flfoot_pos_body_frame'] = np.array(cfg['input']['flfoot_pos_body_frame'])
                fname_dict['frfoot_pos_body_frame'] = np.array(cfg['input']['frfoot_pos_body_frame'])
                fname_dict['rlfoot_pos_body_frame'] = np.array(cfg['input']['rlfoot_pos_body_frame'])
                fname_dict['rrfoot_pos_body_frame'] = np.array(cfg['input']['rrfoot_pos_body_frame'])
                
                ## OUTPUT
                #num_plans = 10 #THIS IS HARDCODED BEWARE, see function call
                for i in range(1,num_plans+1):
                    ii = str(i)
                    plan_dict = {}
                    plan_dict['flfoot_forces'] = np.array(cfg['output']['plan_time_'+ii]['flfoot_forces'])
                    plan_dict['frfoot_forces'] = np.array(cfg['output']['plan_time_'+ii]['frfoot_forces'])
                    plan_dict['rlfoot_forces'] = np.array(cfg['output']['plan_time_'+ii]['rlfoot_forces'])
                    plan_dict['rrfoot_forces'] = np.array(cfg['output']['plan_time_'+ii]['rrfoot_forces'])
                    fname_dict['plan_time_'+ii] = plan_dict

                ## INTERNAL STATES
                #num_plans = 10 #THIS IS HARDCODED BEWARE, see function call
                for i in range(1,num_plans+1):
                    ii = str(i)
                    internal_state_plan_dict = {}

                    internal_state_plan_dict['roll'] = cfg['internal_state']['plan_time_'+ii]['roll']
                    internal_state_plan_dict['pitch'] = cfg['internal_state']['plan_time_'+ii]['pitch']
                    internal_state_plan_dict['yaw'] = cfg['internal_state']['plan_time_'+ii]['yaw']
                    
                    internal_state_plan_dict['com_x'] = cfg['internal_state']['plan_time_'+ii]['com_x']
                    internal_state_plan_dict['com_y'] = cfg['internal_state']['plan_time_'+ii]['com_y']
                    internal_state_plan_dict['com_z'] = cfg['internal_state']['plan_time_'+ii]['com_z']
                    
                    internal_state_plan_dict['com_ang_vel0'] = cfg['internal_state']['plan_time_'+ii]['com_ang_vel0']
                    internal_state_plan_dict['com_ang_vel1'] = cfg['internal_state']['plan_time_'+ii]['com_ang_vel1']
                    internal_state_plan_dict['com_ang_vel2'] = cfg['internal_state']['plan_time_'+ii]['com_ang_vel2']
                    
                    internal_state_plan_dict['com_x_vel'] = cfg['internal_state']['plan_time_'+ii]['com_x_vel']
                    internal_state_plan_dict['com_y_vel'] = cfg['internal_state']['plan_time_'+ii]['com_y_vel']
                    internal_state_plan_dict['com_z_vel'] = cfg['internal_state']['plan_time_'+ii]['com_z_vel']
                    
                    fname_dict['internal_state_plan_time_'+ii] = internal_state_plan_dict
                
                data[fname] = fname_dict


            except yaml.YAMLError as err:
                print(err)
                data = []
                return data

    return data


def make_plots(fnames,data,num_plans=10):
    if fnames == [] or data == []:
        print('\n')
        print('error: fnames returns empty list')
        print('check arguments provided in the terminal')
        return
    
    else:

        leg = ['fl','fr','rl','rr']
        force = ['Fx','Fy','Fz']
        xyz = ['x','y','z']
        labels= ['Euler Angles','COM','COM Angle Vel','COM Vel']

        for i in range(len(fnames)):
            fname_dict = data[fnames[i]]

            com_pos = fname_dict['com_pos']
            com_vel_body_frame = fname_dict['com_vel_body_frame']
            com_rpy_zyx = fname_dict['com_rpy_zyx']

            ang_vel = fname_dict['ang_vel']
            com_vel_des = fname_dict['com_vel_des']
            yaw_vel_des = fname_dict['yaw_vel_des']

            flfoot_pos_body_frame = fname_dict['flfoot_pos_body_frame']
            frfoot_pos_body_frame = fname_dict['frfoot_pos_body_frame']
            rlfoot_pos_body_frame = fname_dict['rlfoot_pos_body_frame']
            rrfoot_pos_body_frame = fname_dict['rrfoot_pos_body_frame']
            
            ts = np.array([fname_dict['initial_time']+k*fname_dict['time_step'] for k in range(10)])

            fig,axes = plt.subplots(3,3)

            for f in range(3):
                axes[f,0].plot(ts,com_pos[f]*np.ones(ts.shape))
                axes[f,0].set_title(xyz[f]+" com_pos")
                axes[f,1].plot(ts,com_vel_body_frame[f]*np.ones(ts.shape))
                axes[f,1].set_title(xyz[f]+" com_vel_body_frame")
                axes[f,2].plot(ts,com_rpy_zyx[f]*np.ones(ts.shape))
                axes[f,2].set_title(xyz[f]+" com_rpy_zyx")

            fig.suptitle(fnames[i])

            fig2,axes2 = plt.subplots(3,3)

            for f in range(3):
                axes2[f,0].plot(ts,ang_vel[f]*np.ones(ts.shape))
                axes2[f,0].set_title(xyz[f]+" ang_vel")
                axes2[f,1].plot(ts,com_vel_des[f]*np.ones(ts.shape))
                axes2[f,1].set_title(xyz[f]+" com_vel_des")
                axes2[f,2].plot(ts,yaw_vel_des[f]*np.ones(ts.shape))
                axes2[f,2].set_title(xyz[f]+" yaw_vel_des")

            fig2.suptitle(fnames[i])

            fig3,axes3 = plt.subplots(3,4)
            for f in range(3):
                axes3[f,0].plot(ts,flfoot_pos_body_frame[f]*np.ones(ts.shape))
                axes3[f,0].set_title(xyz[f]+" flfoot_pos_body_frame")
                axes3[f,1].plot(ts,frfoot_pos_body_frame[f]*np.ones(ts.shape))
                axes3[f,1].set_title(xyz[f]+" frfoot_pos_body_frame")
                axes3[f,2].plot(ts,rlfoot_pos_body_frame[f]*np.ones(ts.shape))
                axes3[f,2].set_title(xyz[f]+" rlfoot_pos_body_frame")
                axes3[f,3].plot(ts,rrfoot_pos_body_frame[f]*np.ones(ts.shape))
                axes3[f,3].set_title(xyz[f]+" rrfoot_pos_body_frame")

            fig3.suptitle(fnames[i])

            fig4, axes4 = plt.subplots(3,4)
            
            foot_forces = int(0) #init -> this is sub optimal but python doesnt let you instantiate empty vars
            for j in range(1,num_plans+1):
                jj = str(j)
                foot_force = fname_dict['plan_time_'+jj]['flfoot_forces'].reshape((-1,1))
                foot_force = np.append(foot_force,fname_dict['plan_time_'+jj]['frfoot_forces'].reshape((-1,1)),axis=1)
                foot_force = np.append(foot_force,fname_dict['plan_time_'+jj]['rlfoot_forces'].reshape((-1,1)),axis=1)
                foot_force = np.append(foot_force,fname_dict['plan_time_'+jj]['rrfoot_forces'].reshape((-1,1)),axis=1)
                
                if isinstance(foot_forces,int):
                    foot_forces = foot_force.reshape((foot_force.shape[0],foot_force.shape[1],1))
                else:
                    foot_forces = np.append(foot_forces,foot_force.reshape((foot_force.shape[0],foot_force.shape[1],1)),axis=2)
            

            for f in range(3): #force XYZ
                for l in range(4): #leg
                    axes4[f,l].scatter(ts,foot_forces[f,l,:])
                    axes4[f,l].grid(True)
                    title = force[f]+" Force Profile for Foot " + leg[l] #maybe change to be shorter
                    axes4[f,l].set_title(title) 
            
            fig4.suptitle(fnames[i])

            fig5, axes5 = plt.subplots(3,4)
            
            internal_states = np.zeros((len(fname_dict['internal_state_plan_time_1']),num_plans)) #init -> this is sub optimal but python doesnt let you instantiate empty vars
            for j in range(1,num_plans+1):
                jj = str(j)

                internal_states[0,j-1] = fname_dict['internal_state_plan_time_'+jj]['roll']
                internal_states[1,j-1] = fname_dict['internal_state_plan_time_'+jj]['pitch']
                internal_states[2,j-1] = fname_dict['internal_state_plan_time_'+jj]['yaw']

                internal_states[3,j-1] = fname_dict['internal_state_plan_time_'+jj]['com_x']
                internal_states[4,j-1] = fname_dict['internal_state_plan_time_'+jj]['com_y']
                internal_states[5,j-1] = fname_dict['internal_state_plan_time_'+jj]['com_z']

                internal_states[6,j-1] = fname_dict['internal_state_plan_time_'+jj]['com_ang_vel0']
                internal_states[7,j-1] = fname_dict['internal_state_plan_time_'+jj]['com_ang_vel1']
                internal_states[8,j-1] = fname_dict['internal_state_plan_time_'+jj]['com_ang_vel2']

                internal_states[9,j-1] = fname_dict['internal_state_plan_time_'+jj]['com_x_vel']
                internal_states[10,j-1] = fname_dict['internal_state_plan_time_'+jj]['com_y_vel']
                internal_states[11,j-1] = fname_dict['internal_state_plan_time_'+jj]['com_z_vel']
            

            for f in range(3): #XYZ
                for l in range(4): #set of values
                    axes5[f,l].scatter(ts,internal_states[f+3*l,:])
                    axes5[f,l].grid(True)
                    title = xyz[f]+' '+labels[l]+" Profile"  #maybe change to be shorter
                    axes5[f,l].set_title(title) 
            
            fig5.suptitle(fnames[i])

        plt.show()



if __name__=='__main__':
    fnames = fetch_yaml()
    data = read_yaml(fnames,10)
    make_plots(fnames,data,10)
    

