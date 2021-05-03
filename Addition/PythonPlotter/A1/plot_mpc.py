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

            leg = ['fl','fr','rl','rr']
            force = ['Fx','Fy','Fz']
            xyz = ['x','y','z']
            
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

        plt.show()



if __name__=='__main__':
    fnames = fetch_yaml()
    data = read_yaml(fnames,10)
    make_plots(fnames,data,10)
    

