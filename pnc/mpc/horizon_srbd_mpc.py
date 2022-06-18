import os
import sys

import rospy
from horizon import problem, variables
from horizon.utils import utils, kin_dyn, mat_storer
from horizon.transcriptions.transcriptor import Transcriptor
from horizon.solvers import solver
from horizon.ros.replay_trajectory import *
from ttictoc import tic, toc
import tf
from geometry_msgs.msg import WrenchStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float32
import numpy as np
from scipy.spatial.transform import Rotation as R
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse

cwd = os.getcwd()
sys.path.append(cwd)
sys.path.append(cwd + '/utils/python_utils')
sys.path.append(cwd + '/simulator/pybullet')
sys.path.append(cwd + '/build/lib')

global start_bool

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def start_service(EmptyRequest):
    global start_bool
    start_bool = True
    return EmptyResponse()

contact_sequence = {
    0: {'name': 'l_foot_contact', 'pos': [0.215904, -0.0395778, -3.38305e-07], 'ori': [1.94791e-06, 8.42233e-07, -0.08949, 0.995988]},
    1: {'name': 'r_foot_contact', 'pos': [0.410509, -0.276642, -1.37213e-06], 'ori': [2.18896e-08, 7.54643e-07, -0.0463577, 0.998925]},
    2: {'name': 'l_foot_contact', 'pos': [0.604699, -0.113649, -9.42198e-07], 'ori': [1.9716e-06, 8.60281e-07, -0.0972253, 0.995262]},
    3: {'name': 'r_foot_contact', 'pos': [0.829269, -0.350677, -1.90514e-06], 'ori': [4.33844e-08, 7.54694e-07, -0.0716339, 0.997431]},
    4: {'name': 'l_foot_contact', 'pos': [1.13052, -0.187717, -0.000509611], 'ori': [1.95986e-06, 8.11842e-07, -0.073243, 0.997314]},
    5: {'name': 'r_foot_contact', 'pos': [1.30234, -0.369171, -2.43946e-06], 'ori': [1.92667e-06, 6.9898e-07, -0.0163611, 0.999866]},
    6: {'name': 'l_foot_contact', 'pos': [1.47579, -0.150112, -2.03872e-06], 'ori': [1.8735e-06, 5.57682e-07, 0.0587137, 0.998275]},
    7: {'name': 'r_foot_contact', 'pos': [1.6498, -0.33243, -2.97334e-06], 'ori': [1.87578e-06, 5.62706e-07, 0.0560531, 0.998428]},
    8: {'name': 'l_foot_contact', 'pos': [1.82564, -0.116004, -2.47358e-06], 'ori': [1.87968e-06, 5.72166e-07, 0.0508041, 0.998709]},
    9: {'name': 'r_foot_contact', 'pos': [2.00342, -0.300662, -3.50697e-06], 'ori': [1.88557e-06, 5.86759e-07, 0.0429647, 0.999077]},
    10: {'name': 'l_foot_contact', 'pos': [2.18472, -0.0884594, -2.74052e-06], 'ori': [1.89213e-06, 6.03083e-07, 0.0341195, 0.999418]},
    }

def fromContactSequenceToFrames(contact):
    rot = R.from_quat(contact['ori'])
    matrix_rot = rot.as_matrix()

    upper_left = np.array(contact['pos']) + np.dot(matrix_rot, np.array([0.1, 0.05, initial_foot_position[0][2]]))
    upper_right = np.array(contact['pos']) + np.dot(matrix_rot, np.array([0.1, -0.05, initial_foot_position[0][2]]))
    lower_left = np.array(contact['pos']) + np.dot(matrix_rot, np.array([-0.1, 0.05, initial_foot_position[0][2]]))
    lower_right = np.array(contact['pos']) + np.dot(matrix_rot, np.array([-0.1, -0.05, initial_foot_position[0][2]]))

    return list(upper_left) + list(upper_right) + list(lower_left) + list(lower_right)

class steps_phase:
    def __init__(self, f, c, nodes, number_of_legs, contact_model, max_force):
        self.f = f
        self.c = c

        self.number_of_legs = number_of_legs
        self.contact_model = contact_model

        self.nodes = nodes
        self.step_counter = 0
        self.contact_positions = list()

        #JUMP
        self.jump_f_bounds = []
        for k in range(0, 7):  # 7 nodes down
            self.jump_f_bounds.append([max_force, max_force, max_force])
        for k in range(0, 8):  # 8 nodes jump
            self.jump_f_bounds.append([0., 0., 0.])
        for k in range(0, 7):  # 6 nodes down
            self.jump_f_bounds.append([max_force, max_force, max_force])

        #NO STEP
        self.f_bounds = []
        for k in range(0, nodes):
            self.f_bounds.append([max_force, max_force, max_force])

        #STEP
        self.l_f_bounds = []
        for k in range(0, 2):  # 2 nodes down
            self.l_f_bounds.append([max_force, max_force, max_force])
        for k in range(0, 8):  # 8 nodes step
            self.l_f_bounds.append([0., 0., 0.])
        for k in range(0, 2):  # 2 nodes down
            self.l_f_bounds.append([max_force, max_force, max_force])
        for k in range(0, 8):  # 8 nodes down (other step)
            self.l_f_bounds.append([max_force, max_force, max_force])
        self.l_f_bounds.append([max_force, max_force, max_force])

        self.r_f_bounds = []
        for k in range(0, 2):  # 2 nodes down
            self.r_f_bounds.append([max_force, max_force, max_force])
        for k in range(0, 8):  # 8 nodes down (other step)
            self.r_f_bounds.append([max_force, max_force, max_force])
        for k in range(0, 2):  # 2 nodes down
            self.r_f_bounds.append([max_force, max_force, max_force])
        for k in range(0, 8):  # 8 nodes step
            self.r_f_bounds.append([0., 0., 0.])
        self.r_f_bounds.append([max_force, max_force, max_force])

        self.action = ""

    def setContactPositions(self, current_contacts, next_contacts):
        # current_contacts and next_contacts MUST be filled with the first left foot
        number_of_contacts = self.number_of_legs * self.contact_model
        if len(current_contacts) != 3*number_of_contacts:
            print(bcolors.FAIL + f"current_contacts must have size: {3*number_of_contacts}" + bcolors.ENDC)
            exit()
        elif len(next_contacts) != 3*number_of_contacts:
            print(bcolors.FAIL + f"next_contacts must have size: {3*number_of_contacts}" + bcolors.ENDC)
            exit()

        self.contact_positions = []
        for k in range(0, 2):  # 2 nodes down
            self.contact_positions.append(current_contacts)
        for k in range(0, 8):  # 8 nodes left footstep
            self.contact_positions.append(next_contacts[0:3*self.contact_model] + current_contacts[3*self.contact_model:])
        for k in range(0, 2):  # 2 nodes down
            self.contact_positions.append(next_contacts[0:3*self.contact_model] + current_contacts[3*self.contact_model:])
        for k in range(0, 8):  # 8 nodes step
            self.contact_positions.append(next_contacts)
        self.contact_positions.append(next_contacts)

        self.stance_contact_position = []
        for k in range(0, self.nodes):
            self.stance_contact_position.append(current_contacts)

    """
    This function moves values on the left 
    """
    def moveLeftParam(self, par):
        values = par.getValues()
        for i in range(0, self.nodes):
            par.assign(values[:, i+1], nodes=i)

    def moveLeftControl(self, ctrl):
        for i in range(0, self.nodes - 1):
            bounds = ctrl.getBounds(i + 1)
            ctrl.setBounds(bounds[0], bounds[1], nodes=i)

    def set(self, action):
        c_dict = dict()
        self.action = action

        if self.action == 'step':
            for i in range(0, contact_model):
                self.moveLeftParam(self.c[i])
                self.c[i].assign(self.contact_positions[0][(i*3):(i * 3 + 3)], nodes=self.nodes)
                c_dict['c' + str(i)] = self.c[i].getValues()[:, 0]

                self.moveLeftControl(f[i])
                self.f[i].setBounds(-1. * np.array(self.l_f_bounds[0]), np.array(self.l_f_bounds[0]), nodes=self.nodes-1)


            for i in range(contact_model, contact_model * number_of_legs):
                self.moveLeftParam(c[i])
                self.c[i].assign(self.contact_positions[0][(i*3):(i * 3 + 3)], nodes=self.nodes)
                c_dict['c' + str(i)] = self.c[i].getValues()[:, 0]

                self.moveLeftControl(f[i])
                self.f[i].setBounds(-1. * np.array(self.r_f_bounds[0]), np.array(self.r_f_bounds[0]), nodes=self.nodes-1)

            del self.contact_positions[0]
            self.r_f_bounds.append(self.r_f_bounds[0])
            self.l_f_bounds.append(self.l_f_bounds[0])
            del self.r_f_bounds[0]
            del self.l_f_bounds[0]

        elif self.action == 'stand':
            for i in range(0, len(c)):
                c_dict['c' + str(i)] = self.stance_contact_position[0][(i*3):(i*3+3)]
                self.c[i].assign(self.stance_contact_position[0][(i*3):(i*3+3)], nodes=self.nodes)
                self.f[i].setBounds(-1. * np.array(self.f_bounds[0]), np.array(self.f_bounds[0]), nodes=self.nodes)

        contactTfBroadcaster(c_dict)

def publishPointTrj(points, t, name, frame, color = [0.7, 0.7, 0.7]):
    marker = Marker()
    marker.header.frame_id = frame
    marker.header.stamp = t
    marker.ns = "SRBD"
    marker.id = 1000
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD

    for k in range(0, points.shape[1]):
        p = Point()
        p.x = points[0, k]
        p.y = points[1, k]
        p.z = points[2, k]
        marker.points.append(p)

    marker.color.a = 1.
    marker.scale.x = 0.005
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]

    rospy.Publisher(name + "_trj", Marker, queue_size=10).publish(marker)

def publishFootsteps(contact_sequence, z_offset):
    marker_array = MarkerArray()
    for contact in contact_sequence:
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'footsteps'
        marker.id = contact + 2
        marker.action = Marker.ADD
        marker.type = Marker.CUBE
        marker.scale.x = 0.2
        marker.scale.y = 0.1
        marker.scale.z = 0.02
        marker.color.r = 1
        marker.color.g = 0
        marker.color.b = 1
        marker.color.a = 0.5
        marker.pose.position.x = contact_sequence[contact]['pos'][0]
        marker.pose.position.y = contact_sequence[contact]['pos'][1]
        marker.pose.position.z = contact_sequence[contact]['pos'][2] + z_offset
        marker.pose.orientation.x = contact_sequence[contact]['ori'][0]
        marker.pose.orientation.y = contact_sequence[contact]['ori'][1]
        marker.pose.orientation.z = contact_sequence[contact]['ori'][2]
        marker.pose.orientation.w = contact_sequence[contact]['ori'][3]
        marker_array.markers.append(marker)

    rospy.Publisher('footsteps', MarkerArray, queue_size=10).publish(marker_array)

def publishContactForce(t, f, frame):
    f_msg = WrenchStamped()
    f_msg.header.stamp = t
    f_msg.header.frame_id = frame
    f_msg.wrench.force.x = f[0]
    f_msg.wrench.force.y = f[1]
    f_msg.wrench.force.z = f[2]
    f_msg.wrench.torque.x = f_msg.wrench.torque.y = f_msg.wrench.torque.z = 0.
    pub = rospy.Publisher('f' + frame, WrenchStamped, queue_size=10).publish(f_msg)

def SRBDTfBroadcaster(r, o, t):
    br = tf.TransformBroadcaster()
    br.sendTransform(r, o, t, "SRB", "world")

def contactTfBroadcaster(c_dict):
    br = tf.TransformBroadcaster()
    for key, val in c_dict.items():
        br.sendTransform(val, [0, 0, 0, 1], rospy.Time.now(), key, 'world')

def SRBDViewer(I, base_frame, t, number_of_contacts):
    marker = Marker()
    marker.header.frame_id = base_frame
    marker.header.stamp = t
    marker.ns = "SRBD"
    marker.id = 0
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = marker.pose.position.y = marker.pose.position.z = 0.
    marker.pose.orientation.x = marker.pose.orientation.y = marker.pose.orientation.z = 0.
    marker.pose.orientation.w = 1.
    a = I[0,0] + I[1,1] + I[2,2]
    marker.scale.x = 0.5*(I[2,2] + I[1,1])/a
    marker.scale.y = 0.5*(I[2,2] + I[0,0])/a
    marker.scale.z = 0.5*(I[0,0] + I[1,1])/a
    marker.color.a = 0.8
    marker.color.r = marker.color.g = marker.color.b = 0.7

    rospy.Publisher('box', Marker, queue_size=10).publish(marker)

    marker_array = MarkerArray()
    for i in range(0, number_of_contacts):
        m = Marker()
        m.header.frame_id = "c" + str(i)
        m.header.stamp = t
        m.ns = "SRBD"
        m.id = i + 1
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = marker.pose.position.y = marker.pose.position.z = 0.
        m.pose.orientation.x = marker.pose.orientation.y = marker.pose.orientation.z = 0.
        m.pose.orientation.w = 1.
        m.scale.x = m.scale.y = m.scale.z = 0.04
        m.color.a = 0.8
        m.color.r = m.color.g = 0.0
        m.color.b = 1.0
        marker_array.markers.append(m)

    pub2 = rospy.Publisher('contacts', MarkerArray, queue_size=10).publish(marker_array)

def setWorld(frame, kindyn, q, base_link="base_link"):
    FRAME = cs.Function.deserialize(kindyn.fk(frame))
    w_p_f = FRAME(q=q)['ee_pos']
    w_r_f = FRAME(q=q)['ee_rot']
    w_T_f = np.identity(4)
    w_T_f[0:3, 0:3] = w_r_f
    w_T_f[0:3, 3] = cs.transpose(w_p_f)

    BASE_LINK = cs.Function.deserialize(kindyn.fk(base_link))
    w_p_bl = BASE_LINK(q=q)['ee_pos']
    w_r_bl = BASE_LINK(q=q)['ee_rot']
    w_T_bl = np.identity(4)
    w_T_bl[0:3, 0:3] = w_r_bl
    w_T_bl[0:3, 3] = cs.transpose(w_p_bl)

    w_T_bl_new = np.dot(np.linalg.inv(w_T_f), w_T_bl)

    rho = R.from_matrix(w_T_bl_new[0:3, 0:3]).as_quat()

    q[0:3] = w_T_bl_new[0:3, 3]
    q[3:7] = rho

rospy.init_node('srbd_mpc_test', anonymous=True)
cpp_args = list()

start_srv = rospy.Service('start_service', Empty, start_service)

"""
Creates HORIZON problem. 
These parameters can not be tuned at the moment.
"""
ns = 20
prb = problem.Problem(ns)
T = 1.

urdf_file = open(cwd + '/robot_model/draco/draco.urdf')
urdf = urdf_file.read()
urdf_file.close()

contact_model = 4
number_of_legs = 2
nc = number_of_legs * contact_model
max_iteration = 5
foot_frames = ["l_foot_contact_upper_left", "l_foot_contact_upper_right", "l_foot_contact_lower_left", "l_foot_contact_lower_right",
               "r_foot_contact_upper_left", "r_foot_contact_upper_right", "r_foot_contact_lower_left", "r_foot_contact_lower_right"]
joint_init = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
         0.0, 0.0, -0.785, 0.785, 0.785, -0.785, 0.0,   # l_hip_ie, l_hip_aa, l_hip_fe, l_knee_fe_jp, l_knee_fe_jd, l_ankle_fe, l_ankle_ie
         0.0, 0.523, 0.0, -1.57, 0.0, 0.0,              # l_shoulder_fe, l_shoulder_aa, l_shoulder_ie, l_elbow_fe, l_wrist_ps, l_wrist_pitch
         0.0,                                           # neck_pitch
         0.0, 0.0, -0.785, 0.785, 0.785, -0.785, 0.0,   # r_hip_ie, r_hip_aa, r_hip_fe, r_knee_fe_jp, r_knee_fe_jd, r_ankle_fe, r_ankle_ie
         0.0, -0.523, 0.0, -1.57, 0.0, 0.0]             # r_shoulder_fe, r_shoulder_aa, r_shoulder_ie, r_elbow_fe, r_wrist_ps, r_wrist_pitch

kindyn = cas_kin_dyn.CasadiKinDyn(urdf)

"""
Creates problem STATE variables
"""
""" CoM Position """
r = prb.createStateVariable("r", 3)
""" Base orientation (quaternion) """
o = prb.createStateVariable("o", 4)

""" Variable to collect all position states """
q = variables.Aggregate()
q.addVariable(r)
q.addVariable(o)

""" Contacts position """
c = dict()
for i in range(0, nc):
    c[i] = prb.createParameter("c" + str(i), 3) # Contact i position as Parameter since it is not an opt variable

""" CoM Velocity and paramter to handle references """
rdot = prb.createStateVariable("rdot", 3) # CoM vel
rdot_ref = prb.createParameter('rdot_ref', 3)
rdot_ref.assign([0. ,0. , 0.], nodes=range(1, ns+1))

""" Base angular Velocity and parameter to handle references """
w = prb.createStateVariable("w", 3) # base vel
w_ref = prb.createParameter('w_ref', 3)
w_ref.assign([0. ,0. , 0.], nodes=range(1, ns+1))

""" Variable to collect all velocity states """
qdot = variables.Aggregate()
qdot.addVariable(rdot)
qdot.addVariable(w)

"""
Creates problem CONTROL variables
"""
"""
Creates problem CONTROL variables: CoM acceleration and base angular accelerations
"""
rddot = prb.createInputVariable("rddot", 3) # CoM acc
wdot = prb.createInputVariable("wdot", 3) # base acc

""" Variable to collect all acceleration controls """
qddot = variables.Aggregate()
qddot.addVariable(rddot)
qddot.addVariable(wdot)

"""
Contacts acceleration and forces
"""
f = dict()
for i in range(0, nc):
    f[i] = prb.createInputVariable("f" + str(i), 3) # Contact i forces

"""
Formulate discrete time dynamics using multiple_shooting and RK2 integrator
"""
x, xdot = utils.double_integrator_with_floating_base(q.getVars(), qdot.getVars(), qddot.getVars(), base_velocity_reference_frame=cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED)
prb.setDynamics(xdot)
prb.setDt(T/ns)
transcription_method = 'multiple_shooting'  # can choose between 'multiple_shooting' and 'direct_collocation'
transcription_opts = dict(integrator='RK2') # integrator used by the multiple_shooting
if transcription_method == 'direct_collocation':
    transcription_opts = dict()
th = Transcriptor.make_method(transcription_method, prb, opts=transcription_opts)

"""
Setting initial state, bounds and limits
"""

max_contact_force = 1000.
print(f"max_contact_force: {max_contact_force}")
max_contact_velocity = 10.
print(f"max_contact_velocity: {max_contact_velocity}")
i = 0
initial_foot_position = dict()
initial_foot_dict = dict()
for frame in foot_frames:
    FK = cs.Function.deserialize(kindyn.fk(frame))
    p = FK(q=joint_init)['ee_pos']
    print(f"{frame}: {p}")

    initial_foot_position[i] = p
    c[i].assign(p, nodes=range(0,ns+1))
    f[i].setBounds([-max_contact_force, -max_contact_force, -max_contact_force], [max_contact_force, max_contact_force, max_contact_force])
    i = i + 1

"""
Add the initial contact positions to the contact_sequence
"""
FK = cs.Function.deserialize(kindyn.fk("l_foot_contact"))
p = FK(q=joint_init)['ee_pos']
p = [p.toarray()[0][0].tolist(), p.toarray()[1][0].tolist(), 0]
ori = FK(q=joint_init)['ee_rot']
rot = R.from_matrix(ori)
quat_rot = rot.as_quat()
initial_foot_dict[-2] = {'name': 'l_foot_contact', 'pos': p, 'ori': quat_rot.tolist()}

FK = cs.Function.deserialize(kindyn.fk("r_foot_contact"))
p = FK(q=joint_init)['ee_pos']
p = [p.toarray()[0][0].tolist(), p.toarray()[1][0].tolist(), 0]

ori = FK(q=joint_init)['ee_rot']
rot = R.from_matrix(ori)
quat_rot = rot.as_quat()
initial_foot_dict[-1] = {'name': 'r_foot_contact', 'pos': p, 'ori': quat_rot.tolist()}

initial_foot_dict.update(contact_sequence)
contact_sequence = initial_foot_dict
"""
Initialize com state and com velocity
"""
COM = cs.Function.deserialize(kindyn.centerOfMass())
com = COM(q=joint_init)['com']
print(f"com: {com}")
r.setInitialGuess(com)
r.setBounds(com, com, 0)
rdot.setInitialGuess([0., 0., 0.])

"""
Initialize base state and base angular velocity
"""
print(f"base orientation: {joint_init[3:7]}")
o.setInitialGuess(joint_init[3:7])
o.setBounds(joint_init[3:7], joint_init[3:7], 0)
w.setInitialGuess([0., 0., 0.])
w.setBounds([0., 0., 0.], [0., 0., 0.], 0)

"""
Set up some therms of the COST FUNCTION
"""
"""
rz_tracking is used to keep the com height around the initial value
"""
rz_tracking_gain = rospy.get_param("rz_tracking_gain", 2e3)
print(f"rz_tracking_gain: {rz_tracking_gain}")
prb.createCost("rz_tracking", 2e3 * cs.sumsqr(r[2] - com[2]), nodes=range(1, ns+1))

"""
o_tracking is used to keep the base orientation at identity, its gain is initialize at 0 and set to non-0 only when a button is pressed
"""
Wo = prb.createParameter('Wo', 1)
Wo.assign(0.)
prb.createCost("o_tracking", Wo * cs.sumsqr(o - joint_init[3:7]), nodes=range(1, ns+1))

"""
rdot_tracking is used to track a desired velocity of the CoM
"""
rdot_tracking_gain = 1e4
print(f"rdot_tracking_gain: {rdot_tracking_gain}")
prb.createCost("rdot_tracking", rdot_tracking_gain * cs.sumsqr(rdot - rdot_ref), nodes=range(1, ns+1))

"""
w_tracking is used to track a desired angular velocity of the base
"""
w_tracking_gain = 1e4
print(f"w_tracking_gain: {w_tracking_gain}")
prb.createCost("w_tracking", w_tracking_gain * cs.sumsqr(w - w_ref), nodes=range(1, ns+1))

"""
min_qddot is to minimize the acceleration control effort
"""
min_qddot_gain = 1e0
print(f"min_qddot_gain: {min_qddot_gain}")
prb.createCost("min_qddot", min_qddot_gain * cs.sumsqr(qddot.getVars()), nodes=list(range(0, ns)))

"""
Set up som CONSTRAINTS
"""

min_f_gain = 1e-2
print(f"min_f_gain: {min_f_gain}")
for i in range(0, nc):
    """
    min_f try to minimze the contact forces (can be seen as distribute equally the contact forces)
    """
    prb.createCost("min_f" + str(i), min_f_gain * cs.sumsqr(f[i]), nodes=list(range(0, ns)))

"""
Friction cones and force unilaterality constraint
TODO: for now flat terrain is assumed (StanceR needs tio be used more or less everywhere for contacts)
"""
mu = 0.8
print(f"mu: {mu}")
for i, fi in f.items():
    # FRICTION CONE
    StanceR = np.identity(3, dtype=float)  # environment rotation wrt inertial frame
    fc, fc_lb, fc_ub = kin_dyn.linearized_friction_cone(fi, mu, StanceR)
    prb.createIntermediateConstraint(f"f{i}_friction_cone", fc, bounds=dict(lb=fc_lb, ub=fc_ub))

"""
Single Rigid Body Dynamics constraint: data are taken from the loaded urdf model in nominal configuration
        m(rddot - g) - sum(f) = 0
        Iwdot + w x Iw - sum(r - p) x f = 0
"""
M = cs.Function.deserialize(kindyn.crba())
m = M(q=joint_init)['B'][0, 0]
print(f"mass: {m}")
I = M(q=joint_init)['B'][3:6, 3:6]
print(f"I centroidal: {I}")
w_R_b = utils.toRot(o)

SRBD = kin_dyn.SRBD(m, w_R_b * I * w_R_b.T, f, r, rddot, c, w, wdot)
prb.createConstraint("SRBD", SRBD, bounds=dict(lb=np.zeros(6), ub=np.zeros(6)), nodes=list(range(0, ns)))

"""
online_solver
"""
hz = 20
print(f"hz: {hz}")
rate = rospy.Rate(hz)  # 10hz

solution_time_pub = rospy.Publisher("solution_time", Float32, queue_size=10)
srbd_pub = rospy.Publisher("srbd_constraint", WrenchStamped, queue_size=10)
srbd_msg = WrenchStamped()

opts = {
        #'ipopt.adaptive_mu_globalization': 'never-monotone-mode',
        #'ipopt.mu_allow_fast_monotone_decrease': 'no',
        #'ipopt.mu_linear_decrease_factor': 0.1,
        #'ipopt.max_cpu_time': 3e-2,
        #'ipopt.hessian_approximation': 'limited-memory',
        #'ipopt.hessian_approximation_space': 'all-variables',
        #'ipopt.limited_memory_aug_solver': 'extended',
        #'ipopt.linear_system_scaling': 'slack-based',
        #'ipopt.ma27_ignore_singularity': 'yes',
        #'ipopt.ma27_skip_inertia_check': 'yes',
        #'ipopt.hessian_constant': 'yes',
        #'ipopt.jac_c_constant' : 'yes',
        #'ipopt.nlp_scaling_method': 'none',
        #'ipopt.magic_steps': 'yes',
        'ipopt.accept_every_trial_step': 'yes',
        'ipopt.tol': 0.001,
        'ipopt.constr_viol_tol': 0.001,
        'ipopt.max_iter': max_iteration,
        'ipopt.linear_solver': 'ma27',
        #'ipopt.warm_start_entire_iterate': 'yes',
        #'ipopt.warm_start_same_structure': 'yes',
        'ipopt.warm_start_init_point': 'yes',
        'ipopt.fast_step_computation': 'yes',
        'ipopt.print_level': 0,
        'ipopt.suppress_all_output': 'yes',
        'ipopt.sb': 'yes',
        'print_time': 0
}

solver = solver.Solver.make_solver('ipopt', prb, opts)

r.setBounds([com[0], com[1], com[2]], [com[0], com[1], com[2]], 0)
rdot.setBounds([0., 0., 0.], [0., 0., 0.], 0)
o.setBounds(joint_init[3:7], joint_init[3:7], 0)
w.setBounds([0., 0., 0.], [0., 0., 0.], 0)

Wo.assign(1e0, nodes=range(0, ns+1))

solver.solve()
solution = solver.getSolutionDict()
solution['r'][:, 1] = [com[0], com[1], com[2]]
solution['rdot'][:, 1] = [0., 0., 0.]
solution['o'][:, 1] = joint_init[3:7]
solution['w'][:, 1] = [0., 0., 0.]

"""
Walking patter generator and scheduler
"""
wpg = steps_phase(f, c, ns, number_of_legs=number_of_legs, contact_model=contact_model, max_force=max_contact_force)
index = 0
start_bool = False

while not rospy.is_shutdown():
    """
    Initialize solution
    """
    # if index != 0:
    r.setBounds(solution['r'][:, 1], solution['r'][:, 1], 0)
    rdot.setBounds(solution['rdot'][:, 1], solution['rdot'][:, 1], 0)
    o.setBounds(solution['o'][:, 1], solution['o'][:, 1], 0)
    w.setBounds(solution['w'][:, 1], solution['w'][:, 1], 0)

    rdot_ref.assign([0., 0., 0.], nodes=range(0, ns+1))
    w_ref.assign([0., 0., 0.], nodes=range(0, ns+1))

    current_positions = fromContactSequenceToFrames(contact_sequence[list(contact_sequence)[0]]) + \
                        fromContactSequenceToFrames(contact_sequence[list(contact_sequence)[1]])
    wpg.setContactPositions(current_positions, current_positions)
    wpg.set('stand')

    solver.solve()
    solution = solver.getSolutionDict()

    publishFootsteps(initial_foot_dict, initial_foot_position[0][2])

    while contact_sequence and start_bool:
        """
        Set previous first element solution as bound for the variables to guarantee continuity
        """
        # if index != 0:
        r.setBounds(solution['r'][:, 1], solution['r'][:, 1], 0)
        rdot.setBounds(solution['rdot'][:, 1], solution['rdot'][:, 1], 0)
        o.setBounds(solution['o'][:, 1], solution['o'][:, 1], 0)
        w.setBounds(solution['w'][:, 1], solution['w'][:, 1], 0)

        rdot_ref.assign([0.4, 0.0, 0.0], nodes=range(0, ns+1))
        w_ref.assign([0.0, 0.0, 0.0], nodes=range(0, ns+1))

        """
        Set contact positions every ns iterations
        """
        if index % (ns+1) == 0:
            print(index)
            current_positions = fromContactSequenceToFrames(
                contact_sequence[list(contact_sequence)[0]]) + fromContactSequenceToFrames(
                contact_sequence[list(contact_sequence)[1]])
            next_positions = fromContactSequenceToFrames(
                contact_sequence[list(contact_sequence)[2]]) + fromContactSequenceToFrames(
                contact_sequence[list(contact_sequence)[3]])
            wpg.setContactPositions(current_positions, next_positions)

            del contact_sequence[list(contact_sequence)[0]]
            del contact_sequence[list(contact_sequence)[0]]

        wpg.set('step')

        """
        Solve
        """
        if not solver.solve():
            print(bcolors.FAIL + "Unable to solve!" + bcolors.ENDC)

        solution = solver.getSolutionDict()

        """
        Marker Publishers
        """
        t = rospy.Time.now()
        publishFootsteps(contact_sequence, initial_foot_position[0][2])
        SRBDTfBroadcaster(solution['r'][:, 0], solution['o'][:, 0], t)
        publishFootsteps(contact_sequence, initial_foot_position[0][2])
        SRBDViewer(I, "SRB", t, nc)
        publishPointTrj(solution["r"], t, "SRB", "world")
        for i in range(0, nc):
            publishContactForce(t, solution['f' + str(i)][:, 0], 'c' + str(i))

        ff = dict()
        for i in range(0, nc):
            ff[i] = solution["f" + str(i)][:, 0]
        srbd_0 = kin_dyn.SRBD(m, I, ff, solution["r"][:, 0], solution["rddot"][:, 0], c, solution["w"][:, 0],
                              solution["wdot"][:, 0])
        srbd_msg.header.stamp = t
        srbd_msg.wrench.force.x = srbd_0[0]
        srbd_msg.wrench.force.y = srbd_0[1]
        srbd_msg.wrench.force.z = srbd_0[2]
        srbd_msg.wrench.torque.x = srbd_0[3]
        srbd_msg.wrench.torque.y = srbd_0[4]
        srbd_msg.wrench.torque.z = srbd_0[5]
        srbd_pub.publish(srbd_msg)

        index += 1
        rate.sleep()

    """
    Marker Publishers
    """
    t = rospy.Time.now()
    publishFootsteps(contact_sequence, initial_foot_position[0][2])
    SRBDTfBroadcaster(solution['r'][:, 0], solution['o'][:, 0], t)
    publishFootsteps(contact_sequence, initial_foot_position[0][2])
    SRBDViewer(I, "SRB", t, nc)
    publishPointTrj(solution["r"], t, "SRB", "world")
    for i in range(0, nc):
        publishContactForce(t, solution['f' + str(i)][:, 0], 'c' + str(i))

    ff = dict()
    for i in range(0, nc):
        ff[i] = solution["f" + str(i)][:, 0]
    srbd_0 = kin_dyn.SRBD(m, I, ff, solution["r"][:, 0], solution["rddot"][:, 0], c, solution["w"][:, 0],
                          solution["wdot"][:, 0])
    srbd_msg.header.stamp = t
    srbd_msg.wrench.force.x = srbd_0[0]
    srbd_msg.wrench.force.y = srbd_0[1]
    srbd_msg.wrench.force.z = srbd_0[2]
    srbd_msg.wrench.torque.x = srbd_0[3]
    srbd_msg.wrench.torque.y = srbd_0[4]
    srbd_msg.wrench.torque.z = srbd_0[5]
    srbd_pub.publish(srbd_msg)

    # index += 1

    rate.sleep()
