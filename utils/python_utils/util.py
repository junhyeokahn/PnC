from collections import OrderedDict
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
import numpy as np
import json
import multiprocessing as mp
from tqdm import tqdm


def PrettyPrint(ob):
    print(json.dumps(ob, indent=4))


def euler_to_rot(angles):
    # Euler ZYX to Rot
    # Note that towr has (x, y, z) order
    x = angles[0]
    y = angles[1]
    z = angles[2]
    ret = np.array([
        np.cos(y) * np.cos(z),
        np.cos(z) * np.sin(x) * np.sin(y) - np.cos(x) * np.sin(z),
        np.sin(x) * np.sin(z) + np.cos(x) * np.cos(z) * np.sin(y),
        np.cos(y) * np.sin(z),
        np.cos(x) * np.cos(z) + np.sin(x) * np.sin(y) * np.sin(z),
        np.cos(x) * np.sin(y) * np.sin(z) - np.cos(z) * np.sin(x), -np.sin(y),
        np.cos(y) * np.sin(x),
        np.cos(x) * np.cos(y)
    ]).reshape(3, 3)
    return np.copy(ret)


def quat_to_rot(quat):
    """
    Parameters
    ----------
    quat (np.array): scalar last quaternion

    Returns
    -------
    ret (np.array): SO3

    """
    return np.copy((R.from_quat(quat)).as_matrix())


def rot_to_quat(rot):
    """
    Parameters
    ----------
    rot (np.array): SO3

    Returns
    -------
    quat (np.array): scalar last quaternion

    """
    return np.copy(R.from_matrix(rot).as_quat())


def QuatToExp(quat):
    img_vec = np.array([quat[0], quat[1], quat[2]])
    w = quat[3]
    theta = 2.0 * np.arcsin(
        np.sqrt(img_vec[0] * img_vec[0] + img_vec[1] * img_vec[1] +
                img_vec[2] * img_vec[2]))

    if np.abs(theta) < 1e-4:
        return np.zeros(3)
    ret = img_vec / np.sin(theta / 2.0)

    return np.copy(ret * theta)


def ExpToQuat(exp):
    theta = np.sqrt(exp[0] * exp[0] + exp[1] * exp[1] + exp[2] * exp[2])
    ret = np.zeros(4)
    if theta > 1e-4:
        ret[0] = np.sin(theta / 2.0) * exp[0] / theta
        ret[1] = np.sin(theta / 2.0) * exp[1] / theta
        ret[2] = np.sin(theta / 2.0) * exp[2] / theta
        ret[3] = np.cos(theta / 2.0)
    else:
        ret[0] = 0.5 * exp[0]
        ret[1] = 0.5 * exp[1]
        ret[2] = 0.5 * exp[2]
        ret[3] = 1.0
    return np.copy(ret)


def weighted_pinv(A, W, rcond=1e-15):
    return np.dot(
        W,
        np.dot(A.transpose(),
               np.linalg.pinv(np.dot(np.dot(A, W), A.transpose()), rcond)))


def get_sinusoid_trajectory(start_time, mid_point, amp, freq, eval_time):
    dim = amp.shape[0]
    p, v, a = np.zeros(dim), np.zeros(dim), np.zeros(dim)
    p = amp * np.sin(2 * np.pi * freq * (eval_time - start_time)) + mid_point
    v = amp * 2 * np.pi * freq * np.cos(2 * np.pi * freq *
                                        (eval_time - start_time))
    a = -amp * (2 * np.pi * freq)**2 * np.sin(2 * np.pi * freq *
                                              (eval_time - start_time))

    return p, v, a


def normalize_data(data):
    mean = np.mean(np.stack(data, axis=0), axis=0)
    std = np.std(np.stack(data, axis=0), axis=0)

    return mean, std, normalize(data, mean, std)


def normalize(x, mean, std):
    assert std.shape == mean.shape
    if type(x) is list:
        assert x[0].shape == mean.shape
        ret = []
        for val in x:
            ret.append((val - mean) / std)
        return ret
    else:
        assert x.shape == mean.shape
        return (x - mean) / std


def denormalize(x, mean, std):
    assert std.shape == mean.shape
    if type(x) is list:
        assert x[0].shape == mean.shape
        ret = []
        for val in x:
            ret.append(val * std + mean)
        return ret
    else:
        assert x.shape == mean.shape
        return x * std + mean


def print_attrs(ob):
    attr = vars(ob)
    print(", \n".join("%s: %s" % item for item in attr.items()))


def try_multiprocess(args_list, num_cpu, f, max_timeouts=1):
    """
    Multiprocessing wrapper function.
    """
    if max_timeouts == 0:
        return None

    if num_cpu == 1:
        return [f(args_list)]
    else:
        pool = mp.Pool(processes=num_cpu,
                       maxtasksperchild=1,
                       initargs=(mp.RLock(), ),
                       initializer=tqdm.set_lock)
        pruns = []
        for i in range(num_cpu):
            rseed = np.random.randint(1000000)
            pruns.append(pool.apply_async(f, args=(args_list + [rseed, i], )))
        try:
            results = [p.get(timeout=36000) for p in pruns]
        except Exception as e:
            print(str(e))
            print('WARNING: error raised in multiprocess, trying again')

            pool.close()
            pool.terminate()
            pool.join()

            return try_multiprocess(args_list, num_cpu, f, max_timeouts - 1)

        pool.close()
        pool.terminate()
        pool.join()

    return results
