'''
Applying 3D mixture gaussian model to generate taskset
Here the 3rd dimension is the execution time Et.
First and second dimensions are the location of the task, i.e. 
- First dimension can be the radius from center, 
- second dimension can be the angle from center, 

Here "mixture" means a task is weighted combination of multiple 3D gaussian models
When generating execution time for a task, we sample from the mixture model to pick one of the 
3D gaussian models first, and then sample from the conditional distribution of Et given 
first and second dimensions of the task.

usage:
python gen_taskset.py TaskData/taskset_cfg_1.json --dir_path TaskData/taskset_cfg_1_gen_1 \
--n_sec 1000 --n_path_per_task 4 --add_perf_records

- taskset_cfg_1.json: configuration file for task generation
- --n_sec 100: number of seconds to generate taskset for
- --dir_path taskset_cfg_1_gen_2: output directory path
- --interact: enable interactive mode (default is False)

The main files generated in [dir_path] are:
- path_Et_task_[i].txt: 
  -- this is the path and execution time of the i-th task
  -- each line is x,y,Et
- taskset_param.yaml: 
  -- this is the parameters of the taskset (Et_mean, Et_sigma, period, weights ..)

OTHER IMPORTANT NOTES to make simulation more realistic (having variance):
- one task (biggest Et range) have performance_records_time/perf; weight being 2;
  other task weight being 1
- total weigths should be summed to 5 (scaled)
- CPU_UTIL_TEMPORAL_VARIANCE_RANGE should be bigger [0.0, 0.5] ?
- generarte Et_variance once or once per path set?

'''

from re import T
import os, sys
import numpy as np
import json, yaml
import matplotlib.pyplot as plt
import random
import argparse

# make sure min execution time is at least (mean - 2*sigma)
# otherwise we often found min execution time is 1
g_et_min_2sigma = True

# for performance_records_time, detla is at least g_min_performance_records_time_step
g_min_performance_records_time_step = 0.5
g_sel_one_perf_record_task = True
g_total_weights = 5
g_fix_weigths = True


###################################################################################################
# utility functions
###################################################################################################

# read the global config file for task generation
def read_cfg_file(cfg_fpath):
    with open(cfg_fpath, 'r') as f:
        cfgs = json.load(f)
        return cfgs
    
    print(f'cannot open config file: {cfg_fpath}')
    return None

# params not printable (np array)
nprintable_params = ["coeffs", "mean_vec", "cov_matrix"]

# these params are common across gaussian tasks for a mixture gaussian task
gaussian_task_params_in_mix_gaussian_task = [
    "period", "D1_MIN", "D1_MAX", "D1_sigma", "D2_MIN", "D2_MAX", "D2_sigma"]

# get the printable parameters for a non-mixture gaussian task
# task_param is the parameters of the task
def get_printable_params_in_gaussian_task(task_param):
    global nprintable_params

    p = {}
    for k,v in task_param.items():
        if k not in nprintable_params: 
            # these are np arrrays, not printable
            p[k] = v
    
    return p

# print the parameters for a non-mixture gaussian task
# task_param is the parameters of the task
def print_params_in_gaussian_task(task_param):
    p = get_printable_params_in_gaussian_task(task_param)
    print(json.dumps(p, indent=4))


# get the printable parameters for a mixture gaussian task
# task_param is the parameters of the task
def get_printable_params_in_mix_gaussian_task(task_param):
    global nprintable_params

    p = {'tasks': []}
    for k,v in task_param.items():
        if k not in nprintable_params: 
            if k != 'tasks':
                p[k] = v
            else:
                for tsk in v:
                    t = {}
                    for tk, tv in tsk.items():
                        if tk not in nprintable_params:
                            t[tk] = tv
                    p['tasks'].append(t)
    
    return p


# print the parameters for a mixture gaussian task
# task_param is the parameters of the task
def print_params_in_mix_gaussian_task(task_param):
    p = get_printable_params_in_mix_gaussian_task(task_param)
    print(json.dumps(p, indent=4))


# get the printable parameters for a set of mixture gaussian tasks
def get_printable_params_in_taskset(taskset_param):
    global nprintable_params
    p = {}
    for k,v in taskset_param.items():
        if k != "tasks":
            p[k] = v

    t = taskset_param['tasks']
    p['tasks'] = []
    for i in range(len(t)):
        # print(f'-- mix gaussian task {i} params:--')
        p['tasks'].append(get_printable_params_in_mix_gaussian_task(t[i]))

    return p


# print the parameters for a set of mixture gaussian tasks
def print_params_in_taskset(taskset_param):
    p = get_printable_params_in_taskset(taskset_param)
    print(json.dumps(p, indent=4))

###################################################################################################
# main functions
###################################################################################################

# This function generate Et, given D1 and D2, following the non-mixture 3D gaussian model
# NOTE: this is reference code (not actually used). Because coeffs can be precomputed, instead of
# computing them every time
def gen_Et_for_gaussian_given_D1_D2(D1, D2, mean_vector, cov_matrix, n_samples=1):
    """
    Generate samples of Et ~ N(...), conditioned on R=r and Theta=theta,
    from an underlying 3D multivariate normal over (R, Theta, Et).

    Parameters
    ----------
    D1 : float, Fixed D1 (like radius)
    D2 : float, Fixed D2 (like angle)
    mean_vector : ndarray of shape (3,), [mu_D1, mu_D2, mu_Et]
    cov_matrix : ndarray of shape (3,3), Covariance matrix for 3 gaussian components.
    n_samples : int, Number of output samples from the conditional distribution.

    Returns
    -------
    samples : ndarray of shape (n_samples,)
        Draws from p(Et | D1=d1, D2=d2).
    """
    # Unpack the mean vector
    mu_D1, mu_D2, mu_et = mean_vector
    
    # Partition the covariance matrix:
    # Sigma_11 = Cov([R,Theta], [R,Theta]) -> top-left 2x2
    ########### NOTE: Sigma11 is diagonal and inverse can be computed directly
    Sigma11 = cov_matrix[:2, :2]
    
    # Sigma_22 = Var(Et) -> bottom-right (single value)
    Sigma22 = cov_matrix[2, 2]
    # Sigma_12 = Cov([R,Theta], Et) -> top-right 2x1
    Sigma12 = cov_matrix[:2, 2]   # shape (2,)
    # Sigma_21 = Cov(Et, [R,Theta]) -> bottom-left 1x2
    Sigma21 = cov_matrix[2, :2]   # shape (2,)

    # Vector forms
    x1 = np.array([D1, D2])       # Observed (D1, D2)
    mu1 = np.array([mu_D1, mu_D2])
    mu2 = mu_et

    # Invert the 2x2 covariance
    inv_Sigma11 = np.linalg.inv(Sigma11)

    # Conditional mean of Et
    # matrix multiplication: 1x2 * 2x2 & 2x1 -> 1x1
    ############# x1 is depending on input r and theta
    cond_mean = mu2 + Sigma21 @ inv_Sigma11 @ (x1 - mu1)

    # Conditional variance of Et
    # matrix multiplication: 1x2 * 2x2 & 2x1 -> 1x1
    cond_var  = Sigma22 - Sigma21 @ inv_Sigma11 @ Sigma12

    # Draw samples from 1D Normal(cond_mean, cond_var)
    samples = np.random.normal(cond_mean, np.sqrt(cond_var), size=n_samples)
    return samples


# This function calcualte the coefficients used in gen_Et_for_gaussian_given_D1_D2
# When generating execution time for a non-mixture 3D gaussian task, we can use these coefficients 
# (to avoid matrix inversion etc), so that it is faster than sample_Et_given_R_theta
def get_Et_calc_coeffs(cov_matrix,mean_vector):
    rt = {}

    # Unpack the mean vector
    mu_r, mu_theta, mu_et = mean_vector
    
    # Partition the covariance matrix:
    # Sigma_11 = Cov([R,Theta], [R,Theta]) -> top-left 2x2
    ########### NOTE: Sigma11 is diagonal and inverse can be computed directly
    Sigma11 = cov_matrix[:2, :2]
    
    # Sigma_22 = Var(Et) -> bottom-right (single value)
    Sigma22 = cov_matrix[2, 2]
    # Sigma_12 = Cov([R,Theta], Et) -> top-right 2x1
    Sigma12 = cov_matrix[:2, 2]   # shape (2,)
    # Sigma_21 = Cov(Et, [R,Theta]) -> bottom-left 1x2
    Sigma21 = cov_matrix[2, :2]   # shape (2,)

    # Vector forms
    #x1 = np.array([r, theta])       # Observed (R, Theta)
    mu1 = np.array([mu_r, mu_theta])
    mu2 = mu_et

    # Invert the 2x2 covariance
    inv_Sigma11 = np.linalg.inv(Sigma11)

    # Conditional mean of Et
    # matrix multiplication: 1x2 * 2x2 & 2x1 -> 1x1
    ############# x1 is depending on input r and theta
    #cond_mean = mu2 + Sigma21 @ inv_Sigma11 @ (x1 - mu1)
    rt['mu2'] = mu2
    rt['mu1'] = mu1
    rt['Sigma21_mul_inv_Sigma11'] = Sigma21 @ inv_Sigma11

    # Conditional variance of Et
    # matrix multiplication: 1x2 * 2x2 & 2x1 -> 1x1
    rt['sqrt_cond_var']  = np.sqrt(Sigma22 - Sigma21 @ inv_Sigma11 @ Sigma12)

    rt['Et_mean'] = mu2
    rt['Et_sigma'] = np.sqrt(Sigma22)

    return rt

# calculate the sigma for mix gaussian tasks
def calc_mix_Et_sigma(Et_mean, weights, gaussian_task_params):
    Et_sigma = 0.0
    n_weights = len(weights)
    for i in range(n_weights):
        Et_sigma += (gaussian_task_params[i]['Et_sigma']**2) * weights[i]
        Et_sigma += ((gaussian_task_params[i]['Et_mean']-Et_mean)**2) * weights[i]
    if Et_sigma < 0:
        Et_sigma = 0
    else:
        Et_sigma = np.sqrt(Et_sigma)
    
    return Et_sigma


PICKED_HZ = []
# generarte the parameters for a non-mixture 3D gaussian task
# cfgs is the global config information
# if period, Et_mean, Et_sigma are not None, they will be used, 
# otherwise they will be randomly generated based on cfgs
def gen_gaussian_task_param(cfgs,period=None,Et_mean=None,Et_sigma=None):
    global PICKED_HZ

    rt = {}

    # randomly select a period from cfgs["PERIODS"]
    if period is not None:
        rt["period"] = period
    else:
        for i in range(10):
            hz  = np.random.choice(cfgs["HZ"])
            if hz not in PICKED_HZ:
                PICKED_HZ.append(hz)
                break
        rt["period"] = int(1000.0/hz)

    if Et_mean is not None:
        rt["Et_mean"] = Et_mean
    else:
        # randomly generate a number between 
        # params["Et_OVER_PERIOD_RANGE"][0] and params["Et_OVER_PERIOD_RANGE"][1]
        et_over_period = np.random.uniform(cfgs["Et_OVER_PERIOD_RANGE"][0], cfgs["Et_OVER_PERIOD_RANGE"][1])
        rt["Et_mean"] = max(1, rt["period"] * et_over_period)

    if Et_sigma is not None:
        rt["Et_sigma"] = Et_sigma
    else:
        # randomly generate a number between 
        # params["SIGMA_OVER_Et_RANGE"][0] and params["SIGMA_OVER_Et_RANGE"][1]
        sigma_over_et = np.random.uniform(cfgs["SIGMA_OVER_Et_RANGE"][0], cfgs["SIGMA_OVER_Et_RANGE"][1])
        rt["Et_sigma"] = max(0.001, rt["Et_mean"] * sigma_over_et)

    # generate D1 sigma
    # just assuming 95% envolope of normal distribution
    rt['D1_MIN'] = cfgs["D1_RANGE"][0]
    rt['D1_MAX'] = cfgs["D1_RANGE"][1]
    D1_RANGE = rt['D1_MAX'] - rt['D1_MIN']
    rt["D1_sigma"] = D1_RANGE/4.0

    # generate D2 sigma
    # just assuming 95% envolope of normal distribution
    rt['D2_MIN'] = cfgs["D2_RANGE"][0]
    rt['D2_MAX'] = cfgs["D2_RANGE"][1]
    D2_RANGE = rt['D2_MAX'] - rt['D2_MIN']
    rt["D2_sigma"] = D2_RANGE/4.0

    # randomly generate a number between cfgs["RO_1_Et_RANGE"][0] and cfgs["RO_1_Et_RANGE"][1]
    ro_1_et = np.random.uniform(cfgs["RO_1_Et_RANGE"][0], cfgs["RO_1_Et_RANGE"][1])
    if ro_1_et < -1:
        ro_1_et = -1
    elif ro_1_et > 1:
        ro_1_et = 1
    rt["ro_1_Et"] = ro_1_et
    
    # randomly generate a number between cfgs["RO_2_Et_RANGE"][0] and cfgs["RO_2_Et_RANGE"][1]
    ro_2_et = np.random.uniform(cfgs["RO_2_Et_RANGE"][0], cfgs["RO_2_Et_RANGE"][1])
    if ro_2_et < -1:
        ro_2_et = -1
    elif ro_2_et > 1:
        ro_2_et = 1
    rt["ro_2_Et"] = ro_2_et


    # Define covariance matrix  
    STD_R  = rt["D1_sigma"]
    STD_THETA  = rt["D2_sigma"]
    STD_Et = rt["Et_sigma"]
    ro_R_Et = rt["ro_1_Et"]
    ro_THETA_Et = rt["ro_2_Et"]
    Et_mean = rt["Et_mean"]

    cov_matrix = np.array([
        [STD_R**2,                 0,                                ro_R_Et * STD_R * STD_Et        ],
        [0,                        STD_THETA**2,                     ro_THETA_Et * STD_THETA * STD_Et],
        [ro_R_Et * STD_R * STD_Et, ro_THETA_Et * STD_THETA * STD_Et, STD_Et**2                       ]
    ])
    
    # define mean vector
    D1_mean = (rt['D1_MAX'] + rt['D1_MIN']) / 2.0
    D2_mean = (rt['D2_MAX'] + rt['D2_MIN']) / 2.0
    mean_vec = np.array([D1_mean, D2_mean, Et_mean])    

    # compute Et coefficients
    #rt['cov_matrix'] = cov_matrix.tolist() # we can dump to json
    #rt['mean_vec'] = mean_vec.tolist()     # we can dump to json
    rt['coeffs'] = get_Et_calc_coeffs(cov_matrix,mean_vec)

    return rt


# generarte the parameters for a mixture 3D gaussian task
def gen_mix_gaussian_task_param(cfgs):
    global gaussian_task_params_in_mix_gaussian_task

    # generate weights and task params
    gaussian_task_params = [] # this is individual non-mixture gaussian task parameters
    weights = []    
    n_weigths = cfgs['N_MIX_WEIGHTS_PER_TASK']

    total = 0
    period = None
    for i in range(n_weigths):
        task_param = gen_gaussian_task_param(cfgs,period=period)
        period = task_param['period']
        gaussian_task_params.append(task_param) # task_params[i] = task_param
        value = random.random()
        if value < 0.1:
            value = 0.1 # min weight is 0.1
        total += value
        weights.append(value)

    for i in range(n_weigths): # normalize the weights
        weights[i] /= total
    
    rt = {}
    rt['weights'] = weights
    rt['n_weights'] = n_weigths

    # put common parameters into rt
    for p in gaussian_task_params_in_mix_gaussian_task:
        rt[p] = gaussian_task_params[0][p]
    
    # compute mixture Et_mean and Et_sigma
    Et_mean = 0.0
    for i in range(n_weigths):
        Et_mean += gaussian_task_params[i]['Et_mean'] * weights[i]
    
    Et_sigma = calc_mix_Et_sigma(Et_mean, weights, gaussian_task_params)
    rt['Et_mean'] = Et_mean
    rt['Et_sigma'] = Et_sigma
    
    # individual gaussian task parameters; removing the common parameters
    rt['tasks'] = []
    for t in gaussian_task_params:
        pp = {}
        for k,v in t.items():
            if k in gaussian_task_params_in_mix_gaussian_task:
                continue
            pp[k] = v
        rt['tasks'].append(pp)

    return rt



# This function generate Et, given D1 and D2, following the non-mixture 3D gaussian model
# it is faster than gen_Et_for_gaussian_given_D1_D2 because it uses pre-computed coefficients
def quickgen_Et_for_gaussian_given_D1_D2(D1, D2, coeffs, n_samples=1):
    global g_et_min_2sigma
    # cond_mean = mu2 + Sigma21 @ inv_Sigma11 @ (x1 - mu1)    
    cond_mean = coeffs['mu2'] + coeffs['Sigma21_mul_inv_Sigma11'] @ (np.array([D1, D2]) - coeffs['mu1'])

    # Draw samples from 1D Normal(cond_mean, cond_var)
    samples = np.random.normal(cond_mean, coeffs['sqrt_cond_var'], size=n_samples)
    if g_et_min_2sigma:
        min_Et = coeffs['Et_mean'] - 2*coeffs['Et_sigma']
    for i in range(n_samples):
        if samples[i] < 1:
            samples[i] = 1
        if g_et_min_2sigma and samples[i] < min_Et:
            samples[i] = min_Et
    return samples



# This function generate Et, given D1 and D2, following the mixture 3D gaussian model
# task_params is a dict including 'mean_vec':   array-like of shape (K, 3)
def quickgen_Et_for_mix_gaussian_given_D1_D2(D1, D2, weights, task_params, n_samples=1):
    global g_et_min_2sigma

    """
    Sample Et given (R=r, Theta=theta) from a mixture of K three-dimensional
    Gaussians. Each component k has:
      - weight[k]
      - means[k] = [mu_R, mu_Theta, mu_Et]
      - covs[k]  = 3x3 covariance matrix

    Parameters
    ----------
    D1, D2 : float, input first and second dimensions data
    'weights': array-like of shape (K,);  Mixture coefficients that sum to 1
    task_params : array of parameters for each non-mixture gaussian task
        Key components are:
        - 'mean_vec':   array-like of shape (K, 3)
        - 'cov_matrix': array-like of shape (K, 3, 3)
        - 'coeffs': quick computation coeffs
    n_samples : int, Number of samples to generate (Et1, Et2, ...)

    Returns
    -------
    et_samples : np.ndarray of shape (n_samples,)
        Samples of Et from the conditional mixture distribution.
    """

    et_samples = np.zeros(n_samples)

    # for each sample to be generated, select one of the gaussian components based on weights
    K = len(weights)
    chosen_components = np.random.choice(K, size=n_samples, p=weights)

    # for each sample, calculate Et based on the selected gaussian component
    for i, k_star in enumerate(chosen_components):
        coeffs = task_params[k_star]['coeffs']
        if g_et_min_2sigma:
            min_Et = coeffs['Et_mean'] - 2*coeffs['Et_sigma']

        m = coeffs['mu2'] + coeffs['Sigma21_mul_inv_Sigma11'] @ (np.array([D1, D2]) - coeffs['mu1'])
        s = coeffs['sqrt_cond_var']
        et_samples[i] = np.random.normal(m, s)
        if et_samples[i] < 1:
            et_samples[i] = 1
        if g_et_min_2sigma and et_samples[i] < min_Et:
            et_samples[i] = min_Et

    return et_samples




###################################################################################################
# test functions
###################################################################################################


# test: for a gaussian task, generate Et for each (D1, D2)
# cfgs: global config parameters
# here: we assume D1 is radius, D2 is angle!
def test_gaussian_task_Et_gen(cfgs,task_param=None,draw=True,save_draw_path=None):
    if task_param is None:
        task_param = gen_gaussian_task_param(cfgs)

    # map is square, -X_MAX <= x <= X_MAX, -Y_MAX <= y <= Y_MAX    
    X_MAX = task_param['D1_MAX']
    X_MIN = task_param['D1_MIN']
    Y_MAX = X_MAX
    Y_MIN = X_MIN   

    # coeffs for quick sampling of execution time
    coeffs = task_param['coeffs']

    # result of execution time; 2D array of shape (2X_MAX+1, 2Y_MAX+1)
    Et_result = np.zeros((X_MAX-X_MIN+1, Y_MAX-Y_MIN+1))
    Et_min = 100000000
    Et_max = -1 
    for x in range(X_MIN, X_MAX+1, 1):
        for y in range(Y_MIN, Y_MAX+1, 1):
            # get radius and angle from x and y
            r = np.sqrt(x**2 + y**2)
        
            # clip r
            #if r > task_param['D1_MAX']:
                #Et_result[x+X_MAX, y+Y_MAX] = 0
                #continue
            #    r = task_param['D1_MAX']

            a = np.arctan2(y, x) * 180 / np.pi
            # np.arctan2(y, x) gives angle in [-180, 180] range. We need it in [0, 360] range
            # convert a to angle between 0 and 360
            if a < 0:
                a = a + 360 # region 3 and 4

            Et_samples = quickgen_Et_for_gaussian_given_D1_D2(r, a, coeffs, n_samples=2)
            Et_result[x+X_MAX, y+Y_MAX] = Et_samples[0]
            Et_min = min(Et_min, Et_samples[0])
            Et_max = max(Et_max, Et_samples[0])

    print('\n\n-------- gaussian task params ----------')
    print_params_in_gaussian_task(task_param)

    print('\n\n-------- min and max Et ----------')
    print(f'Et min: {Et_min}, Et max: {Et_max}')

    if draw or save_draw_path is not None:
        # draw 3D of Et_result
        X, Y = np.meshgrid(np.arange(X_MIN, X_MAX+1), np.arange(Y_MIN, Y_MAX+1))
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot_surface(X, Y, Et_result)

        if save_draw_path is not None:
            plt.savefig(save_draw_path)
        
        if draw:
            plt.show()

    return Et_result


# test: for a mixture gaussian task, generate Et for each (D1, D2)
# cfgs: global config parameters
# here: we assume D1 is radius, D2 is angle!
def test_mix_gaussian_task_Et_gen(cfgs,task_param=None,draw=True,save_draw_path=None):
    if task_param is None:
        task_param = gen_mix_gaussian_task_param(cfgs)

    weights = task_param['weights']
    task_params = task_param['tasks']

    # map is square, -X_MAX <= x <= X_MAX, -Y_MAX <= y <= Y_MAX    
    X_MAX = task_param['D1_MAX']
    X_MIN = task_param['D1_MIN']
    Y_MAX = X_MAX
    Y_MIN = X_MIN

    # result of execution time; 2D array of shape (2X_MAX+1, 2Y_MAX+1)
    Et_result = np.zeros((X_MAX-X_MIN+1, Y_MAX-Y_MIN+1))

    Et_min = 100000000
    Et_max = -1
    for x in range(X_MIN, X_MAX+1, 1):
        for y in range(Y_MIN, Y_MAX+1, 1):
            # get radius and angle from x and y
            r = np.sqrt(x**2 + y**2)
            
            # CLIP radius
            #if r > task_param['D1_MAX']:
            #    #Et_result[x+X_MAX, y+Y_MAX] = 0
            #    #continue
            #    r = task_param['D1_MAX']

            a = np.arctan2(y, x) * 180 / np.pi
            # np.arctan2(y, x) gives angle in [-180, 180] range. We need it in [0, 360] range
            # convert a to angle between 0 and 360
            if a < 0:
                a = a + 360 # region 3 and 4

            Et_samples = quickgen_Et_for_mix_gaussian_given_D1_D2(r, a, weights, task_params, n_samples=1)

            Et_result[x+X_MAX, y+Y_MAX] = Et_samples[0]

            Et_min = min(Et_min, Et_samples[0])
            Et_max = max(Et_max, Et_samples[0])

    print('\n\n-------- mixture gaussian task params ----------')
    print_params_in_mix_gaussian_task(task_param)

    print('\n\n-------- min and max Et ----------')
    print(f'Et min: {Et_min}, Et max: {Et_max}')

    if draw or save_draw_path is not None:
        # draw 3D of Et_result
        X, Y = np.meshgrid(np.arange(-X_MAX, X_MAX+1), np.arange(-Y_MAX, Y_MAX+1))
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot_surface(X, Y, Et_result)

        if save_draw_path is not None:
            plt.savefig(save_draw_path)

        if draw:
            plt.show()

    return Et_result


# generate stop stations in map
# when generating path, assuming robots moving between randomly selected two stops
# cfg is the global config params
def gen_stops_in_map(cfgs,X_STEP_RATIO=0.1,Y_STEP_RATIO=0.1):
    X_MIN = cfgs['D1_RANGE'][0]
    X_MAX = cfgs['D1_RANGE'][1]
    Y_MIN = X_MIN
    Y_MAX = X_MAX

    X_STEP = int((X_MAX - X_MIN)*X_STEP_RATIO)
    Y_STEP = int((Y_MAX - Y_MIN)*Y_STEP_RATIO)

    stops = []
    for x in range(X_MIN, X_MAX+1, int(X_STEP)):
        for y in range(Y_MIN, Y_MAX+1, int(Y_STEP)):
            stops.append((x,y))

    return stops


# generate the path
# - taskset_param is typically a mixture gaussian task params
# - stops is typically generated by gen_stops_in_map; a list of (D1, D2) representing stops in map
# - cb is a callback function that is called after each step to generate Et.
#   it is typically quickgen_Et_for_mix_gaussian_given_D1_D2
# - n_steps: assuming 1 step is per execution, and min execution time is 1 ms
#   10000 may be enough
# - reverse_prob: the probability of reverse direction (giving some noise)
# - draw is a flag to indicate whether to draw the path
# - pic_path: the path to save the drawn path
# - dump_path: the path to save the generated path and Et
def gen_path(cfgs,stops,n_steps=1000,taskset_param=None,cb=None,reverse_prob=0.1,draw=True,
    pic_path=None,dump_path=None,
    Et_vary_factors=[],Et_vary_t_dlt=1):

    N = len(stops)
    X_MIN = cfgs['D1_RANGE'][0]
    X_MAX = cfgs['D1_RANGE'][1]
    Y_MIN = X_MIN
    Y_MAX = X_MAX

    src_stop = random.randint(0, N-1)    
    dst_stop = random.randint(0, N-1)
    if dst_stop == src_stop:
        dst_stop = (src_stop+1) % N
    #print(f'######## select stop: src: {src_stop}, dst: {dst_stop}')
    curr = stops[src_stop]
    dst  = stops[dst_stop]

    steps = []

    Et_min = 100000000
    Et_max = -1

    i = 0
    Et_vary_factor_idx = 0
    curr_Et_vary_s = 0.0
    while i < n_steps:
        if curr == dst:
            # generate new source and destination since we reached the current destination
            src_stop = dst_stop
            dst_stop = random.randint(0, N-1)
            if dst_stop == src_stop:
                dst_stop = (src_stop+1) % N
            #print(f'######## select stop: src: {src_stop}, dst: {dst_stop}')
            curr = stops[src_stop]
            dst  = stops[dst_stop]
        
        dy = dst[1] - curr[1]
        dx = dst[0] - curr[0]
        
        # probability of moving in each direction
        # 0 - left, 1 - up, 2 - right, 3 - down
        move_direction_p = [0,0,0,0]
        if dx > 0:
            # move right with more priority
            move_direction_p[2] = dx
            move_direction_p[0] = dx * reverse_prob
        elif dx < 0:
            # move left with more priority
            move_direction_p[0] = -dx
            move_direction_p[2] = -dx * reverse_prob
        
        if dy > 0:
            # move up with more priority
            move_direction_p[1] = dy
            move_direction_p[3] = dy * reverse_prob
        elif dy < 0:
            # move down with more priority
            move_direction_p[3] = -dy
            move_direction_p[1] = -dy * reverse_prob
        
        total_prob = 0.0
        for k in range(4):
            total_prob += move_direction_p[k]
        for k in range(4):
            move_direction_p[k] /= total_prob
        
        # for each sample, select one of the gaussian components based on weights
        chosen_components = np.random.choice(4, size=1, p=move_direction_p)
        move_direction = chosen_components[0]

        if move_direction == 0:
            # left 1 step
            if curr[0] <= X_MIN:
                # bounce back
                curr = (X_MIN+1, curr[1])
            else:
                curr = (curr[0]-1, curr[1])
        elif move_direction == 1:
            # up 1 step
            if curr[1] >= Y_MAX:
                # bounce back
                curr = (curr[0], Y_MAX-1)
            else:
                curr = (curr[0], curr[1]+1)
        elif move_direction == 2:
            # right 1 step
            if curr[0] >= X_MAX:
                # bounce back
                curr = (X_MAX-1, curr[1])
            else:
                curr = (curr[0]+1, curr[1])
        elif move_direction == 3:
            # down 1 step
            if curr[1] <= Y_MIN:
                # bounce back
                curr = (curr[0], Y_MIN+1)
            else:
                curr = (curr[0], curr[1]-1)

        if cb is not None and taskset_param is not None:
            et = cb(curr,taskset_param)
        else:
            et = 0.0

        if len(Et_vary_factors)>0 and taskset_param is not None:
            et *= (1.0+Et_vary_factors[Et_vary_factor_idx])
            if et<1:
                et = 1
            curr_Et_vary_s += taskset_param['period']/1000.0
            if curr_Et_vary_s >= Et_vary_t_dlt:
                Et_vary_factor_idx = (Et_vary_factor_idx+1) % len(Et_vary_factors)
                #print(f'########## next Et_vary_factor_idx: {Et_vary_factor_idx}, curr_Et_vary_s: {curr_Et_vary_s}')
                curr_Et_vary_s -= Et_vary_t_dlt

        Et_min = min(Et_min,et)
        Et_max = max(Et_max,et)

        steps.append((curr[0],curr[1],et))    

        # GLOBAL STEP += 1
        i += 1

    if dump_path is not None:
        with open(dump_path, "w") as f:
            for x, y, et in steps:
                f.write(f"{x},{y},{et}\n")

    if draw or pic_path is not None:
        grid_size = stops[1][0] - stops[0][0]
        if grid_size == 0:
            grid_size = stops[1][1] - stops[0][1]
        if grid_size < 0:
            grid_size = -grid_size
        if grid_size == 0:
            grid_size = int((X_MAX-X_MIN)*0.1)

        # Unzip the list of tuples into x and y coordinates
        x_values, y_values, et_values = zip(*steps)

        # Create a figure and axis for the plot
        plt.figure(figsize=(6, 6))

        # Plot the points
        plt.plot(x_values, y_values, marker='o', color='b', linestyle='-', markersize=6)

        # Set limits to match the specified range
        plt.xlim(X_MIN, X_MAX)
        plt.ylim(Y_MIN, Y_MAX)

        # Add grid
        plt.grid(True)

        # Set grid interval (spacing)
        plt.xticks(np.arange(X_MIN, X_MAX+1, grid_size))  # Grid lines for x-axis
        plt.yticks(np.arange(Y_MIN, Y_MAX+1, grid_size))  # Grid lines for y-axis

        # Label the axes
        plt.xlabel('X')
        plt.ylabel('Y')

        # Title of the plot
        plt.title('Moving Path (x, y)')

        if pic_path is not None:
            plt.savefig(pic_path)

        # Show the plot
        if draw:
            plt.show()

    return steps,[float(Et_min),float(Et_max)]

# convert taskset parameter to old task_characteristics.yaml format
def conv_taskset_param_to_old_fmt(inp,n_sec=None,add_perf_records=True):
    global g_min_performance_records_time_step
    global g_sel_one_perf_record_task
    global g_total_weights, g_fix_weigths

    out = {'tasks': []}
    n = len(inp['tasks'])
    if n_sec is None:
        n_ms = 100000
    else:
        n_ms = n_sec*1000

    performance_records_time_idx = -1
    performance_records_t = {}
    total_weights = 0.0
    for i in range(n):
        t = {'id': i}
        t['execution_time_mu'] = inp['tasks'][i]['Et_mean']
        t['execution_time_sigma'] = inp['tasks'][i]['Et_sigma']
        t['execution_time_min'] = inp['tasks'][i]['Et_min']
        t['execution_time_max'] = inp['tasks'][i]['Et_max']
        t['period'] = inp['tasks'][i]['period']
        t['deadline'] = t['period']
        t['processorId'] = 0
        t['name'] = f'task_{i+1}'
        t['sp_threshold'] = inp['tasks'][i]['sp_threshold']
        if g_fix_weigths:
            t['sp_weight'] = 1
        else:
            t['sp_weight'] = inp['tasks'][i]['sp_weight']
        total_weights += t['sp_weight']
        t['total_running_time'] = n_ms

        if add_perf_records:
            n_steps = 9 # 0.1 to 1
            performance_records_time = []
            performance_records_perf = []
            step = (t['execution_time_max'] - t['execution_time_min']) / n_steps
            t_s = t['execution_time_min']
            for ii in range(1,n_steps+2):
                performance_records_time.append(t_s)
                performance_records_perf.append(ii*0.1)
                t_s += step                

            if step >= g_min_performance_records_time_step:
                performance_records_time_s = " ".join(f"{x:.3f}" for x in performance_records_time)
                performance_records_perf_s = " ".join(f"{x:.1f}" for x in performance_records_perf)
                prd = inp['tasks'][i]['period']
                if g_sel_one_perf_record_task:
                    # only keep one 
                    if performance_records_time_idx < 0 or prd < performance_records_t['period']:
                        performance_records_time_idx = i
                        performance_records_t['performance_records_time'] = performance_records_time_s
                        performance_records_t['performance_records_perf'] = performance_records_perf_s
                        performance_records_t['period'] = prd
                else:
                    # add to all tasks meeting requirement 
                    t['performance_records_time'] = performance_records_time_s
                    t['performance_records_perf'] = performance_records_perf_s
                    if g_fix_weigths:
                        t['sp_weight'] = 2
                        total_weights += 1

 
        out['tasks'].append(t)

    if g_sel_one_perf_record_task and performance_records_time_idx >= 0:
        #print(f'performance_records_time_idx={performance_records_time_idx}')
        out['tasks'][performance_records_time_idx]['sp_weight'] = 2
        out['tasks'][performance_records_time_idx]['performance_records_time'] = \
            performance_records_t['performance_records_time']
        out['tasks'][performance_records_time_idx]['performance_records_perf'] = \
            performance_records_t['performance_records_perf']
        if g_fix_weigths:
            total_weights += 1

    if total_weights != g_total_weights:
        for i in range(n):
            out['tasks'][i]['sp_weight'] *= g_total_weights/total_weights

    return out

# Custom YAML dumper to format the list as a space-separated string
class SpaceSeparatedListDumper(yaml.Dumper):
    def increase_indent(self, flow=False, indentless=False):
        return super(SpaceSeparatedListDumper, self).increase_indent(flow=True)

###################################################################################################
# major functions
###################################################################################################

# generate parameters for a set of tasks, each of which is a mixture of 3D gaussians
def gen_taskset_param(cfgs,dump_dir=None,save_task_Et_plot=False,n_sec=None):
    global gaussian_task_params_in_mix_gaussian_task

    # generate task until total utilization >= cpu_util
    cpu_util = cfgs['MEAN_CPU_UTIL']
    total_util = 0.0

    # this is the list for each mixture gaussian task
    taskset_param = []

    # generate mixture gaussian task one by one until total utilization >= cpu_util
    while total_util < cpu_util:
        task_param = gen_mix_gaussian_task_param(cfgs)
        taskset_param.append(task_param)
        total_util += task_param['Et_mean']/task_param['period']
    
    # scale Et of every individual gaussian task, and re-generate the whole taskset_param
    util_scale_factor = cpu_util/total_util
    # print(f'util_scale_factor={util_scale_factor}')
    n_tasks = len(taskset_param)
    for i in range(n_tasks):
        n_weights = taskset_param[i]['n_weights']
        period = taskset_param[i]['period']
        Et_mean_new = 0.0
        for k in range(n_weights):
            #print(f'i={i}, k={k}, old Et_mean={taskset_param[i]["tasks"][k]["Et_mean"]}, old Et_sigma={taskset_param[i]["tasks"][k]["Et_sigma"]}')
            Et_mean    = taskset_param[i]['tasks'][k]['Et_mean'] * util_scale_factor
            Et_sigma   = taskset_param[i]['tasks'][k]['Et_sigma'] * util_scale_factor
            task_param = gen_gaussian_task_param(cfgs,period=period,Et_mean=Et_mean,Et_sigma=Et_sigma)

            if dump_dir is not None and save_task_Et_plot:
                os.makedirs(dump_dir, exist_ok=True)
                save_draw_path = os.path.join(dump_dir, f"gaussian_task_{i}_{k}.png")
                test_gaussian_task_Et_gen(cfgs,task_param=task_param,draw=False,save_draw_path=save_draw_path)

            # remove the common parameters
            for key in gaussian_task_params_in_mix_gaussian_task:
                if key in task_param:
                    del task_param[key]

            taskset_param[i]['tasks'][k] = task_param
            #print(f'new Et_mean={taskset_param[i]["tasks"][k]["Et_mean"]}, new Et_sigma={taskset_param[i]["tasks"][k]["Et_sigma"]}')
            Et_mean_new += task_param['Et_mean'] * taskset_param[i]['weights'][k]
        
        Et_sigma = calc_mix_Et_sigma(Et_mean_new, taskset_param[i]['weights'], taskset_param[i]['tasks'])
        Et_sigma = float(Et_sigma)

        # reflect new Et_mean and Et_sigma
        taskset_param[i]['Et_mean'] = Et_mean_new
        taskset_param[i]['Et_sigma'] = Et_sigma

        if dump_dir is not None and save_task_Et_plot:
            os.makedirs(dump_dir, exist_ok=True)
            save_draw_path = os.path.join(dump_dir, f"mix_gaussian_task_{i}.png")
            test_mix_gaussian_task_Et_gen(cfgs,task_param=taskset_param[i],draw=False,save_draw_path=save_draw_path)

    weight_min = cfgs['SP_WEIGHT_RANGE'][0]
    weight_max = cfgs['SP_WEIGHT_RANGE'][1]
    trd_min = cfgs['SP_THRESHOLD_RANGE'][0]
    trd_max = cfgs['SP_THRESHOLD_RANGE'][1]	
    for i in range(len(taskset_param)):
        # generate sp_weight and sp_threshold
        taskset_param[i]['sp_weight'] = random.uniform(weight_min, weight_max)
        taskset_param[i]['sp_threshold'] = random.uniform(trd_min, trd_max)		
       
    rt = {'n_tasks': n_tasks, 'cpu_util': cpu_util, 'tasks': taskset_param}

    if dump_dir is not None:
        os.makedirs(dump_dir, exist_ok=True)
        dump_yml_fpath = os.path.join(dump_dir, "taskset_param.yaml")
        with open(dump_yml_fpath, "w") as f:
            #### make sure get printable taskset_param and then dump to yaml
            pp = get_printable_params_in_taskset(rt)
            yaml.dump(pp, f, sort_keys=False,default_flow_style=False, width=float("inf"), Dumper=SpaceSeparatedListDumper) 
            # sort_keys=False keeps original order
    
    return rt


# read taskset params back from yaml file, and fill the coeffs for each gaussian task
def load_and_fill_taskset_param_file(taskset_param_fpath):
    with open(taskset_param_fpath, "r") as f:
        # load params
        taskset_param = yaml.safe_load(f)

        # generate coeffs for each gaussan task in each mixture gaussian task
        n_tasks = taskset_param['n_tasks']
        for i in range(n_tasks):
            task_param = taskset_param['tasks'][i]
            n_weights = task_param['n_weights']
            for k in range(n_weights):
                rt = task_param['tasks'][k]
                STD_R  = task_param["D1_sigma"]
                STD_THETA  = task_param["D2_sigma"]
                STD_Et = rt["Et_sigma"]
                ro_R_Et = rt["ro_1_Et"]
                ro_THETA_Et = rt["ro_2_Et"]
                Et_mean = rt["Et_mean"]

                cov_matrix = np.array([
                    [STD_R**2,                 0,                                ro_R_Et * STD_R * STD_Et        ],
                    [0,                        STD_THETA**2,                     ro_THETA_Et * STD_THETA * STD_Et],
                    [ro_R_Et * STD_R * STD_Et, ro_THETA_Et * STD_THETA * STD_Et, STD_Et**2                       ]
                ])
                
                D1_mean = (task_param['D1_MAX'] + task_param['D1_MIN']) / 2.0
                D2_mean = (task_param['D2_MAX'] + task_param['D2_MIN']) / 2.0
                mean_vec = np.array([D1_mean, D2_mean, Et_mean])    

                taskset_param['tasks'][i]['tasks'][k]['coeffs'] = get_Et_calc_coeffs(cov_matrix,mean_vec)

        return taskset_param
    
    return None
  

################################ main api ################################
# read cfg file, generate taskset params, and paths, and dump to dir_path
def gen_taskset_param_path(cfg_file,n_sec=1000,dir_path=None,
                           add_perf_records=True,interact=False,n_path_per_task=4):

    OPT_SP_PROJECT_PATH = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    #print(OPT_SP_PROJECT_PATH)
    #print(f'add_perf_records: {add_perf_records}')
    #quit()



    if cfg_file is None:
        print(f'cannot find config file: {cfg_file}')
        return

    if not cfg_file.startswith('/'):
        cfg_file = os.path.join(OPT_SP_PROJECT_PATH, cfg_file)

    if not os.path.exists(cfg_file):
        print(f'cannot find config file: {cfg_file}')
        return

    cfgs = read_cfg_file(cfg_file)
    #print('\n\n-------- global config ----------')
    #print(json.dumps(cfgs, indent=4))

    if dir_path is None:
        # get the file name of cfg_file
        cfg_file_name = os.path.basename(cfg_file)
        dir_path = os.path.join(OPT_SP_PROJECT_PATH, 'TaskData', cfg_file_name)
        os.makedirs(dir_path, exist_ok=True)
    else:
        if not dir_path.startswith('/'):
            dir_path = os.path.join(OPT_SP_PROJECT_PATH, dir_path)
        os.makedirs(dir_path, exist_ok=True)

    # generate taskset params
    params = gen_taskset_param(cfgs,dir_path,save_task_Et_plot=True,n_sec=n_sec)

    # reload and fill params
    fpath = os.path.join(dir_path, 'taskset_param.yaml')
    params = load_and_fill_taskset_param_file(fpath)
    #print('\n\n-------- taskset param ----------')
    #print_params_in_taskset(params)

    # generate stops
    stops = gen_stops_in_map(cfgs,X_STEP_RATIO=0.1,Y_STEP_RATIO=0.1)
    n_ms = n_sec*1000;
    
    def gen_et_cb(currpos, task_param):
        #quickgen_Et_for_mix_gaussian_given_D1_D2(r, a, weights, task_params, n_samples=1)
        
        # calc r and a from currpos
        x = currpos[0]
        y = currpos[1]
        r = np.sqrt(x**2 + y**2)
        if r>task_param['D1_MAX']:
            r = task_param['D1_MAX']
        a = np.arctan2(y, x) * 180 / np.pi
        if a < 0:
            a = a + 360

        weights = task_param['weights']
        Et_s = quickgen_Et_for_mix_gaussian_given_D1_D2(r, a, weights, task_param['tasks'], n_samples=1)
        Et = Et_s[0]
        if Et<1:
            Et = 1
        return Et

    i = 0
    draw = False
    if interact:
        draw = True

    # check CPU_UTIL_TEMPORAL_VARIANCE_RANGE and CPU_UTIL_TEMPORAL_VARIANCE_T_DLT
    Et_vary_factors = []
    if 'CPU_UTIL_TEMPORAL_VARIANCE_RANGE' in cfgs and 'CPU_UTIL_TEMPORAL_VARIANCE_T_DLT' in cfgs:
        min_val = cfgs['CPU_UTIL_TEMPORAL_VARIANCE_RANGE'][0]
        max_val = cfgs['CPU_UTIL_TEMPORAL_VARIANCE_RANGE'][1]
        u_scale_steps = 4;
        u_scale_step = (max_val - min_val) / u_scale_steps
        if u_scale_step>0:
            for i in range(0,n_sec,cfgs['CPU_UTIL_TEMPORAL_VARIANCE_T_DLT']):
                v = random.uniform(min_val,max_val) - min_val
                v = round(v / u_scale_step) * u_scale_step + min_val
                Et_vary_factors.append(v)
                #print(f'######## Et_vary_factors: {Et_vary_factors}')
    
    # generate paths for each task
    # i is task index
    i = 0
    for task in params['tasks']:
        #Et_mean = task['Et_mean']
        #n_steps = (n_ms/Et_mean)
        prd = task['period']
        n_steps = (n_ms/prd)
        cb = gen_et_cb

        # generate paths for the task i
        t_min = 0
        t_max = 0
        for k in range(n_path_per_task):
            if dir_path is None:
                pic_path = None
                dump_path = None
            else:
                pic_path = os.path.join(dir_path, f"path_task_{i}_{k}.png")
                dump_path = os.path.join(dir_path, f"path_Et_task_{i}_{k}.txt")

            print(f'generating path for task {i}, {n_steps} steps, {k}th instance')
            steps, task_Et_min_max = gen_path(cfgs,stops,n_steps=n_steps,taskset_param=task,cb=cb,
                reverse_prob=0.2,draw=draw,pic_path=pic_path,dump_path=dump_path,
                Et_vary_factors=Et_vary_factors,Et_vary_t_dlt=cfgs['CPU_UTIL_TEMPORAL_VARIANCE_T_DLT'])
            if k==0:
                t_min = task_Et_min_max[0]
                t_max = task_Et_min_max[1]
            else:
                if task_Et_min_max[0]<t_min:
                    t_min = task_Et_min_max[0]
                if task_Et_min_max[1]>t_max:
                    t_max = task_Et_min_max[1]
        
        params['tasks'][i]['Et_min'] = t_min
        params['tasks'][i]['Et_max'] = t_max
		
        i += 1


    if dir_path is not None:
        old_task_char = conv_taskset_param_to_old_fmt(params,n_sec=n_sec,add_perf_records=add_perf_records)
        os.makedirs(dir_path, exist_ok=True)
        dump_yml_fpath = os.path.join(dir_path, "taskset_characteristics.yaml")
        with open(dump_yml_fpath, "w") as f:
            yaml.dump(old_task_char, f, sort_keys=False,default_flow_style=False, width=float("inf"), Dumper=SpaceSeparatedListDumper)  
            # sort_keys=False keeps original order
			
    if 'CPU_UTIL_TEMPORAL_VARIANCE_RANGE' in cfgs and 'CPU_UTIL_TEMPORAL_VARIANCE_T_DLT' in cfgs:
        print(f'######## Et_vary_factors: {Et_vary_factors}')


if __name__ == "__main__":
    # Set up argument parser
    parser = argparse.ArgumentParser(description="Taskset and path generation for Et.")

    # Required argument: cfg_file (string)
    parser.add_argument("cfg_file", type=str, help="Path to the configuration file")

    # Optional argument: n_sec (int)
    parser.add_argument("--n_sec", type=int, help="Number of seconds (integer)",default=1000)

    # Optional argument: n_path_per_task (int)
    parser.add_argument("--n_path_per_task", type=int, help="Generate how many paths per task",default=4)

    # Optional argument: dir_path (string)
    parser.add_argument("--dir_path", type=str, help="Output directory path")

    # Optional argument: add_perf_records (boolean flag)
    parser.add_argument("--add_perf_records", action="store_true", help="Add performance records")

    # Optional argument: interact (boolean flag)
    parser.add_argument("--interact", action="store_true", help="Enable interactive mode")

    # Parse arguments
    args = parser.parse_args()

    # Call the main function with parsed arguments
    gen_taskset_param_path(args.cfg_file, n_sec=args.n_sec, dir_path=args.dir_path, 
        add_perf_records=args.add_perf_records, interact=args.interact,n_path_per_task=args.n_path_per_task)


# run
# if draw interact, add --interact
# python gen_taskset.py TaskData/taskset_cfg_1.json --n_sec 1000 --dir_path TaskData/taskset_cfg_1_gen_2


'''
# other unit tests
cfg_file = 'taskset_cfg_1.json'
cfgs = read_cfg_file(cfg_file)
print('\n\n-------- global config ----------')
print(json.dumps(cfgs, indent=4))

# single gaussian task
test_gaussian_task_Et_gen(cfgs,draw=True)

# mixture gaussian task
task_params = gen_mix_gaussian_task_param(cfgs)
print_params_in_mix_gaussian_task(task_params)
test_mix_gaussian_task_Et_gen(cfgs,task_param=task_params,draw=True)
'''