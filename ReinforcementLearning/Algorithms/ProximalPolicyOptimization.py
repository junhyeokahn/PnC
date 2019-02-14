from collections import deque
import time

import tensorflow as tf
import numpy as np
from mpi4py import MPI

from stable_baselines.common import Dataset, explained_variance, fmt_row, zipsame, ActorCriticRLModel, SetVerbosity, \
    TensorboardWriter
from stable_baselines import logger
import stable_baselines.common.tf_util as tf_util
from stable_baselines.common.policies import LstmPolicy, ActorCriticPolicy
from stable_baselines.common.mpi_adam import MpiAdam
from stable_baselines.common.mpi_moments import mpi_moments
from stable_baselines.trpo_mpi.utils import traj_segment_generator, add_vtarg_and_adv, flatten_lists
from stable_baselines.a2c.utils import total_episode_reward_logger

from ruamel.yaml import YAML

class PPO(ActorCriticRLModel):
    """
    Proximal Policy Optimization algorithm (MPI version).
    Paper: https://arxiv.org/abs/1707.06347
    :param env: (Gym environment or str) The environment to learn from (if registered in Gym, can be str)
    :param policy: (ActorCriticPolicy or str) The policy model to use (MlpPolicy, CnnPolicy, CnnLstmPolicy, ...)
    :param timesteps_per_actorbatch: (int) timesteps per actor per update
    :param clip_param: (float) clipping parameter epsilon
    :param entcoeff: (float) the entropy loss weight
    :param optim_epochs: (float) the optimizer's number of epochs
    :param optim_stepsize: (float) the optimizer's stepsize
    :param optim_batchsize: (int) the optimizer's the batch size
    :param gamma: (float) discount factor
    :param lam: (float) advantage estimation
    :param adam_epsilon: (float) the epsilon value for the adam optimizer
    :param schedule: (str) The type of scheduler for the learning rate update ('linear', 'constant',
        'double_linear_con', 'middle_drop' or 'double_middle_drop')
    :param verbose: (int) the verbosity level: 0 none, 1 training information, 2 tensorflow debug
    :param tensorboard_log: (str) the log location for tensorboard (if None, no logging)
    :param _init_setup_model: (bool) Whether or not to build the network at the creation of the instance
    :param policy_kwargs: (dict) additional arguments to be passed to the policy on creation
    """
    def __init__(self, policy, env, data_generator, gamma=0.99, timesteps_per_actorbatch=256, clip_param=0.2, entcoeff=0.01,
                 optim_epochs=4, optim_stepsize=1e-3, optim_batchsize=64, lam=0.95, adam_epsilon=1e-5,
                 schedule='linear', verbose=0, tensorboard_log=None, _init_setup_model=True, policy_kwargs=None):

        super().__init__(policy=policy, env=env, verbose=verbose, requires_vec_env=False,
                         _init_setup_model=_init_setup_model, policy_kwargs=policy_kwargs)

        self.gamma = gamma
        self.timesteps_per_actorbatch = timesteps_per_actorbatch
        self.clip_param = clip_param
        self.entcoeff = entcoeff
        self.optim_epochs = optim_epochs
        self.optim_stepsize = optim_stepsize
        self.optim_batchsize = optim_batchsize
        self.lam = lam
        self.adam_epsilon = adam_epsilon
        self.schedule = schedule
        self.tensorboard_log = tensorboard_log
        self.data_generator = data_generator

        self.graph = None
        self.sess = None
        self.policy_pi = None
        self.loss_names = None
        self.lossandgrad = None
        self.adam = None
        self.assign_old_eq_new = None
        self.compute_losses = None
        self.params = None
        self.step = None
        self.proba_step = None
        self.initial_state = None
        self.summary = None
        self.episode_reward = None

        if _init_setup_model:
            self.setup_model()

    def setup_model(self):
        with SetVerbosity(self.verbose):

            self.graph = tf.Graph()
            with self.graph.as_default():
                self.sess = tf_util.single_threaded_session(graph=self.graph)

                # Construct network for new policy
                self.policy_pi = self.policy(self.sess, self.observation_space, self.action_space, self.n_envs, 1,
                                             None, reuse=False, **self.policy_kwargs)

                # Network for old policy
                with tf.variable_scope("oldpi", reuse=False):
                    old_pi = self.policy(self.sess, self.observation_space, self.action_space, self.n_envs, 1,
                                         None, reuse=False, **self.policy_kwargs)

                with tf.variable_scope("loss", reuse=False):
                    # Target advantage function (if applicable)
                    atarg = tf.placeholder(dtype=tf.float32, shape=[None])

                    # Empirical return
                    ret = tf.placeholder(dtype=tf.float32, shape=[None])

                    # learning rate multiplier, updated with schedule
                    lrmult = tf.placeholder(name='lrmult', dtype=tf.float32, shape=[])

                    # Annealed cliping parameter epislon
                    clip_param = self.clip_param * lrmult

                    obs_ph = self.policy_pi.obs_ph
                    action_ph = self.policy_pi.pdtype.sample_placeholder([None])

                    kloldnew = old_pi.proba_distribution.kl(self.policy_pi.proba_distribution)
                    ent = self.policy_pi.proba_distribution.entropy()
                    meankl = tf.reduce_mean(kloldnew)
                    meanent = tf.reduce_mean(ent)
                    pol_entpen = (-self.entcoeff) * meanent

                    # pnew / pold
                    ratio = tf.exp(self.policy_pi.proba_distribution.logp(action_ph) -
                                   old_pi.proba_distribution.logp(action_ph))

                    # surrogate from conservative policy iteration
                    surr1 = ratio * atarg
                    surr2 = tf.clip_by_value(ratio, 1.0 - clip_param, 1.0 + clip_param) * atarg

                    # PPO's pessimistic surrogate (L^CLIP)
                    pol_surr = - tf.reduce_mean(tf.minimum(surr1, surr2))
                    vf_loss = tf.reduce_mean(tf.square(self.policy_pi.value_fn[:, 0] - ret))
                    total_loss = pol_surr + pol_entpen + vf_loss
                    losses = [pol_surr, pol_entpen, vf_loss, meankl, meanent]
                    self.loss_names = ["pol_surr", "pol_entpen", "vf_loss", "kl", "ent"]

                    tf.summary.scalar('entropy_loss', pol_entpen)
                    tf.summary.scalar('policy_gradient_loss', pol_surr)
                    tf.summary.scalar('value_function_loss', vf_loss)
                    tf.summary.scalar('approximate_kullback-leiber', meankl)
                    tf.summary.scalar('clip_factor', clip_param)
                    tf.summary.scalar('loss', total_loss)

                    self.params = tf_util.get_trainable_vars("model")
                    self.policy_param = tf_util.get_trainable_vars("model/pi")
                    self.valfn_param = tf_util.get_trainable_vars("model/vf")

                    self.assign_old_eq_new = tf_util.function(
                        [], [], updates=[tf.assign(oldv, newv) for (oldv, newv) in
                                         zipsame(tf_util.get_globals_vars("oldpi"), tf_util.get_globals_vars("model"))])

                with tf.variable_scope("Adam_mpi", reuse=False):
                    self.adam = MpiAdam(self.params, epsilon=self.adam_epsilon, sess=self.sess)

                with tf.variable_scope("input_info", reuse=False):
                    tf.summary.scalar('discounted_rewards', tf.reduce_mean(ret))
                    tf.summary.histogram('discounted_rewards', ret)
                    tf.summary.scalar('learning_rate', tf.reduce_mean(self.optim_stepsize))
                    tf.summary.histogram('learning_rate', self.optim_stepsize)
                    tf.summary.scalar('advantage', tf.reduce_mean(atarg))
                    tf.summary.histogram('advantage', atarg)
                    tf.summary.scalar('clip_range', tf.reduce_mean(self.clip_param))
                    tf.summary.histogram('clip_range', self.clip_param)
                    if len(self.observation_space.shape) == 3:
                        tf.summary.image('observation', obs_ph)
                    else:
                        tf.summary.histogram('observation', obs_ph)

                self.step = self.policy_pi.step
                self.proba_step = self.policy_pi.proba_step
                self.initial_state = self.policy_pi.initial_state

                tf_util.initialize(sess=self.sess)

                self.summary = tf.summary.merge_all()

                self.lossandgrad = tf_util.function([obs_ph, old_pi.obs_ph, action_ph, atarg, ret, lrmult],
                                                    [self.summary, tf_util.flatgrad(total_loss, self.params)] + losses)
                self.compute_losses = tf_util.function([obs_ph, old_pi.obs_ph, action_ph, atarg, ret, lrmult],
                                                       losses)

    def learn(self, total_timesteps, callback=None, seed=None, log_interval=100, tb_log_name="PPO1"):
        with SetVerbosity(self.verbose), TensorboardWriter(self.graph, self.tensorboard_log, tb_log_name) as writer:
            self._setup_learn(seed)

            assert issubclass(self.policy, ActorCriticPolicy), "Error: the input policy for the PPO1 model must be " \
                                                               "an instance of common.policies.ActorCriticPolicy."

            with self.sess.as_default():
                self.adam.sync()

                # Prepare for rollouts
                # seg_gen = traj_segment_generator(self.policy_pi, self.env, self.timesteps_per_actorbatch)

                episodes_so_far = 0
                timesteps_so_far = 0
                iters_so_far = 0
                t_start = time.time()

                # rolling buffer for episode lengths
                lenbuffer = deque(maxlen=100)
                # rolling buffer for episode rewards
                rewbuffer = deque(maxlen=100)

                self.episode_reward = np.zeros((self.n_envs,))

                while True:
                    if callback is not None:
                        # Only stop training if return value is False, not when it is None. This is for backwards
                        # compatibility with callbacks that have no return statement.
                        if callback(locals(), globals()) == False:
                            break
                    if total_timesteps and timesteps_so_far >= total_timesteps:
                        break

                    if self.schedule == 'constant':
                        cur_lrmult = 1.0
                    elif self.schedule == 'linear':
                        cur_lrmult = max(1.0 - float(timesteps_so_far) / total_timesteps, 0)
                    else:
                        raise NotImplementedError

                    logger.log("********** Iteration %i ************" % iters_so_far)

                    # seg = seg_gen.__next__()
                    seg = self.data_generator.get_data_segment(self.sess, self.policy_param, self.valfn_param)
                    #### TEST : nn check
                    # obs = seg['ob']
                    # py_pol, py_val, _, _ = self.step(obs, deterministic=True)
                    # print("policy!!!")
                    # print(py_pol - seg['ac'])
                    # print("value!!!")
                    # print(py_val - seg['vpred'])
                    # __import__('ipdb').set_trace()
                    #### TEST
                    add_vtarg_and_adv(seg, self.gamma, self.lam)

                    # ob, ac, atarg, ret, td1ret = map(np.concatenate, (obs, acs, atargs, rets, td1rets))
                    obs_ph, action_ph, atarg, tdlamret = seg["ob"], seg["ac"], seg["adv"], seg["tdlamret"]

                    # true_rew is the reward without discount
                    if writer is not None:
                        self.episode_reward = total_episode_reward_logger(self.episode_reward,
                                                                          seg["true_rew"].reshape((self.n_envs, -1)),
                                                                          seg["dones"].reshape((self.n_envs, -1)),
                                                                          writer, timesteps_so_far)

                    # predicted value function before udpate
                    vpredbefore = seg["vpred"]

                    # standardized advantage function estimate
                    atarg = (atarg - atarg.mean()) / atarg.std()
                    dataset = Dataset(dict(ob=obs_ph, ac=action_ph, atarg=atarg, vtarg=tdlamret),
                                      shuffle=not issubclass(self.policy, LstmPolicy))
                    optim_batchsize = self.optim_batchsize or obs_ph.shape[0]

                    # set old parameter values to new parameter values
                    self.assign_old_eq_new(sess=self.sess)
                    logger.log("Optimizing...")
                    logger.log(fmt_row(13, self.loss_names))

                    # Here we do a bunch of optimization epochs over the data
                    for k in range(self.optim_epochs):
                        # list of tuples, each of which gives the loss for a minibatch
                        losses = []
                        for i, batch in enumerate(dataset.iterate_once(optim_batchsize)):
                            steps = (timesteps_so_far +
                                     k * optim_batchsize +
                                     int(i * (optim_batchsize / len(dataset.data_map))))
                            if writer is not None:
                                # run loss backprop with summary, but once every 10 runs save the metadata
                                # (memory, compute time, ...)
                                if (1 + k) % 10 == 0:
                                    run_options = tf.RunOptions(trace_level=tf.RunOptions.FULL_TRACE)
                                    run_metadata = tf.RunMetadata()
                                    summary, grad, *newlosses = self.lossandgrad(batch["ob"], batch["ob"], batch["ac"],
                                                                                 batch["atarg"], batch["vtarg"],
                                                                                 cur_lrmult, sess=self.sess,
                                                                                 options=run_options,
                                                                                 run_metadata=run_metadata)
                                    writer.add_run_metadata(run_metadata, 'step%d' % steps)
                                else:
                                    summary, grad, *newlosses = self.lossandgrad(batch["ob"], batch["ob"], batch["ac"],
                                                                                 batch["atarg"], batch["vtarg"],
                                                                                 cur_lrmult, sess=self.sess)
                                writer.add_summary(summary, steps)
                            else:
                                _, grad, *newlosses = self.lossandgrad(batch["ob"], batch["ob"], batch["ac"],
                                                                       batch["atarg"], batch["vtarg"], cur_lrmult,
                                                                       sess=self.sess)

                            self.adam.update(grad, self.optim_stepsize * cur_lrmult)
                            losses.append(newlosses)
                        logger.log(fmt_row(13, np.mean(losses, axis=0)))

                    logger.log("Evaluating losses...")
                    losses = []
                    for batch in dataset.iterate_once(optim_batchsize):
                        newlosses = self.compute_losses(batch["ob"], batch["ob"], batch["ac"], batch["atarg"],
                                                        batch["vtarg"], cur_lrmult, sess=self.sess)
                        losses.append(newlosses)
                    mean_losses, _, _ = mpi_moments(losses, axis=0)
                    logger.log(fmt_row(13, mean_losses))
                    for (loss_val, name) in zipsame(mean_losses, self.loss_names):
                        logger.record_tabular("loss_" + name, loss_val)
                    logger.record_tabular("ev_tdlam_before", explained_variance(vpredbefore, tdlamret))

                    # local values
                    lrlocal = (seg["ep_lens"], seg["ep_rets"])

                    # list of tuples
                    listoflrpairs = MPI.COMM_WORLD.allgather(lrlocal)
                    lens, rews = map(flatten_lists, zip(*listoflrpairs))
                    lenbuffer.extend(lens)
                    rewbuffer.extend(rews)
                    if len(lenbuffer) > 0:
                        logger.record_tabular("EpLenMean", np.mean(lenbuffer))
                        logger.record_tabular("EpRewMean", np.mean(rewbuffer))
                    logger.record_tabular("EpThisIter", len(lens))
                    episodes_so_far += len(lens)
                    timesteps_so_far += MPI.COMM_WORLD.allreduce(seg["total_timestep"])
                    iters_so_far += 1
                    logger.record_tabular("EpisodesSoFar", episodes_so_far)
                    logger.record_tabular("TimestepsSoFar", timesteps_so_far)
                    logger.record_tabular("TimeElapsed", time.time() - t_start)
                    if self.verbose >= 1 and MPI.COMM_WORLD.Get_rank() == 0:
                        logger.dump_tabular()

        return self

    def save(self, save_path):
        data = {
            "gamma": self.gamma,
            "timesteps_per_actorbatch": self.timesteps_per_actorbatch,
            "clip_param": self.clip_param,
            "entcoeff": self.entcoeff,
            "optim_epochs": self.optim_epochs,
            "optim_stepsize": self.optim_stepsize,
            "optim_batchsize": self.optim_batchsize,
            "lam": self.lam,
            "adam_epsilon": self.adam_epsilon,
            "schedule": self.schedule,
            "verbose": self.verbose,
            "policy": self.policy,
            "observation_space": self.observation_space,
            "action_space": self.action_space,
            "n_envs": self.n_envs,
            "_vectorize_action": self._vectorize_action,
            "policy_kwargs": self.policy_kwargs
        }

        params = self.sess.run(self.params)

        self._save_to_file(save_path, data=data, params=params)

        policy_param = self.sess.run(self.policy_param)
        val_fn_param = self.sess.run(self.valfn_param)

        if (len(policy_param) % 2 == 0):
            p_b_stochastic = False
            p_num_layer = (int) (len(policy_param) / 2)
        else:
            p_b_stochastic = True
            p_num_layer = (int) ((len(policy_param)-1) / 2)

        if (len(val_fn_param) % 2 == 0):
            v_b_stochastic = False
            v_num_layer = (int) (len(val_fn_param) / 2)
        else:
            v_b_stochastic = True
            v_num_layer = (int) ((len(val_fn_param)-1) / 2)

        pol_params = {}
        pol_params['b_stochastic'] = p_b_stochastic
        #### TEST
        # pol_params['b_stochastic'] = False
        #### TEST
        pol_params['num_layer'] = p_num_layer
        for layer_idx in range(p_num_layer):
            pol_params['w'+str(layer_idx)] = policy_param[2*layer_idx].tolist()
            pol_params['b'+str(layer_idx)] = (policy_param[2*layer_idx+1]).reshape(1, (policy_param[2*layer_idx+1]).shape[0]).tolist()
            if (layer_idx == (p_num_layer - 1)):
                pol_params['act_fn'+str(layer_idx)] = 0
            else:
                pol_params['act_fn'+str(layer_idx)] = 1
        if p_b_stochastic:
            pol_params['logstd'] = policy_param[-1].tolist()

        valfn_params = {}
        valfn_params['b_stochastic'] = v_b_stochastic
        valfn_params['num_layer'] = v_num_layer
        for layer_idx in range(p_num_layer):
            valfn_params['w'+str(layer_idx)] = val_fn_param[2*layer_idx].tolist()
            valfn_params['b'+str(layer_idx)] = val_fn_param[2*layer_idx+1].reshape(1, (val_fn_param[2*layer_idx+1]).shape[0]).tolist()
            if (layer_idx == (v_num_layer - 1)):
                valfn_params['act_fn'+str(layer_idx)] = 0
            else:
                valfn_params['act_fn'+str(layer_idx)] = 1
        if v_b_stochastic:
            valfn_params['logstd'] = val_fn_param[-1].tolist()

        data = {"pol_params": pol_params, "valfn_params": valfn_params}
        with open(save_path + '.yaml', 'w') as f:
            yaml = YAML()
            yaml.dump(data, f)

        #### TEST
        # obs = np.array([[0]])
        # while(True):
            # __import__('ipdb').set_trace()
            # first = np.tanh( np.matmul(obs, policy_param[0]) + policy_param[1].reshape(1, policy_param[1].shape[0] ) )
            # second = np.tanh( np.matmul(first, policy_param[2]) + policy_param[3].reshape(1, policy_param[3].shape[0]) )
            # third = np.tanh( np.matmul(second, policy_param[4]) + policy_param[5].reshape(1, policy_param[5].shape[0]) )
            # self.step(obs, deterministic=True)
        #### TEST
