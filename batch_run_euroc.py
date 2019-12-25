import os
import sys
import copy
import numpy as np
import matplotlib.pyplot as plt

launch_file = '/home/symao/workspace/catkin_ws_ov/src/open_vins/ov_msckf/launch/pgeneva_eth.launch'
bag_dir = '/home/symao/data/euroc/rosbag'
# bag_name:bag_start in sec
bag_list = {'MH_01_easy':0,
            'MH_02_easy':0,
            'MH_03_medium':0,
            'MH_04_difficult':0,
            'MH_05_difficult':3,
            'V1_01_easy':5,
            'V1_02_medium':10,
            'V1_03_difficult':6,
            'V2_01_easy':4,
            'V2_02_medium':4,
            'V2_03_difficult':5}
run_cmd = 'roslaunch ov_msckf pgeneva_eth.launch'
run_res = '/home/symao/temp_rmse.txt'
py_path = os.path.dirname(os.path.abspath(sys.argv[0]))
res_table = os.path.join(py_path,'ov_msckf_result.md')
png_dir = os.path.join(py_path,'figure')

marker_table = ['o','*','^','s','p','+','x','d','h','v','<','>','1','2','3']
png_idx = 0

def run_once():
    # return tuple((np.random.rand(2)*3).tolist())
    if os.path.exists(run_res):
        os.remove(run_res)
    os.system(run_cmd)
    if os.path.exists(run_res):
        return [float(x) for x in open(run_res).readlines()[0].split(' ')]
    else:
        return [-1,-1]

# [name:(type,value)]
def modify_launch(params):
    lines = open(launch_file, "r").readlines()
    fp = open(launch_file, "w")
    for line in lines:
        for name in params.keys():
            if name in line:
                a, b = params[name]
                line = '    <param name="%s" type="%s" value="%s" />\n'%(name,a,b)
                break
        fp.write(line)
    fp.close()

def average_rmse(rmse):
    deg, meter = np.mean([(a,b) for a,b in rmse if a>0 and b>0], axis=0)
    return deg, meter

def plot_rmse(res_list, save_png):
    fig = plt.figure(figsize=(16, 8))
    names = [s[:5] for s in bag_list.keys()] + ['avg']
    plt.subplot(121)
    for i, res in enumerate(res_list):
        lege, rmse = res
        data = np.array(rmse)
        plt.plot(data[:,0], marker_table[i]+'-', label=lege)
    plt.legend()
    plt.xticks(range(len(names)),names,rotation=60)
    plt.ylabel('orientation error[degree]')
    plt.ylim([0,5])
    plt.subplot(122)
    for i, res in enumerate(res_list):
        lege, rmse = res
        data = np.array(rmse)
        plt.plot(data[:,1], marker_table[i]+'-', label=lege)
    plt.legend()
    plt.xticks(range(len(names)),names,rotation=60)
    plt.ylabel('position error[m]')
    plt.ylim([0,1])
    plt.savefig(save_png)

def loop_rosbag(params):
    rmse = []
    params = copy.deepcopy(params)
    for bag,start_ts in bag_list.items():
        fbag = os.path.join(bag_dir,bag+'.bag')
        fcsv = "$(find ov_data)/euroc_mav/%s.csv"%bag
        if os.path.exists(fbag):
            params['path_bag'] = ('string', fbag)
            params['path_gt'] = ('string', fcsv)
            params['bag_start'] = ('double', '%f'%float(start_ts))
            modify_launch(params)
            res = run_once()
            rmse.append(res)
        else:
            rmse.append((-1,-1))
    rmse.append(average_rmse(rmse))
    return rmse

def batch_run_single_change(param_table, default_params, fp):
    default_res = loop_rosbag(default_params)
    for param_name, param_type, choice in param_table:
        fp.write('|%s|%s|avg|\n'%(param_name, '|'.join([s[:5] for s in bag_list.keys()])))
        fp.write('|'+'--|'*(len(bag_list.keys())+2)+'\n')
        fp.write('|%s|%s|\n'%(choice[0],'|'.join(['%.2f,%.2f'%(a,b) for a,b in default_res])))
        params = copy.deepcopy(default_params)
        res_list = [('%s=%s'%(param_name,choice[0]), default_res)]
        for c in choice[1:]:
            params[param_name] = (param_type, c)
            res = loop_rosbag(params)
            res_list.append(('%s=%s'%(param_name,c), res))
            fp.write('|%s|%s|\n'%(c,'|'.join(['%.2f,%.2f'%(a,b) for a,b in res])))
        fp.write('\n')

        global png_idx
        png_name = os.path.join(png_dir, '%06d.jpg'%png_idx)
        plot_rmse(res_list, png_name)
        fp.write('![fig](./figure/%06d.jpg)\n'%(png_idx))
        fp.flush()
        png_idx = png_idx + 1

def run_special_case(params, case_name, fej, intrin, extrin, dt, slam, fp):
    params = copy.deepcopy(params)
    params['use_fej'] = ('bool', 'true' if fej else 'false')
    params['calib_cam_intrinsics'] = ('bool', 'true' if intrin else 'false')
    params['calib_cam_extrinsics'] = ('bool', 'true' if extrin else 'false')
    params['calib_cam_timeoffset'] = ('bool', 'true' if dt else 'false')
    params['max_slam'] = ('int', str(slam))
    fp.write('|%s|%d|%d|%d|%d|%d|%s|\n'%(case_name,fej,intrin,extrin,dt,slam,
                '|'.join(['%.2f,%.2f'%(a,b) for a,b in loop_rosbag(params)])))

if __name__ == '__main__':
    # the first choice always be default
    param_table = [('use_fej',              'bool', ['true','false']),
                   ('calib_cam_intrinsics', 'bool', ['true','false']),
                   ('calib_cam_extrinsics', 'bool', ['true','false']),
                   ('calib_cam_timeoffset', 'bool', ['true','false']),
                   ('max_clones',           'int',  ['11','5','10','20','30']),
                   ('max_slam',             'int',  ['50','0','20','100','200']),
                   ('feat_representation',  'string',['GLOBAL_3D','GLOBAL_FULL_INVERSE_DEPTH',
                                                     'ANCHORED_3D','ANCHORED_FULL_INVERSE_DEPTH',
                                                     'ANCHORED_MSCKF_INVERSE_DEPTH'])]
    default_params = {a:(b,c[0]) for a,b,c in param_table}

    # clean roslog
    os.system('rm -rf ~/.ros/log')

    # record result in markdown
    if not os.path.exists(png_dir):
        os.makedirs(png_dir)
    fp = open(res_table,'w')
    fp.write('# OpenVINS evaluation on EuROC dataset\n')
    fp.write('NOTE: we log the RMSE of orientation and postion which print by ov_msckf. Unit: [deg, m]\n')

    # single variable-controlling
    fp.write('## 1. Single param comparision\n')

    fp.write('### 1.1 Mono Version\n')
    params = copy.deepcopy(default_params)
    params['max_cameras'] = ('int', '1')
    batch_run_single_change(param_table, params, fp)

    fp.write('### 1.2 Stereo Version\n')
    params = copy.deepcopy(default_params)
    params['max_cameras'] = ('int', '2')
    batch_run_single_change(param_table, params, fp)

    # user define combination
    fp.write('## 2. special cases comparision\n')
    fp.write('We test several special cases\n\n')

    fp.write('### 2.1 Effect from fej, calib, dt, slam\n')
    fp.write('We use default param for: sliding window(11), feature representation(GLOBAL_3D).\n\n')
    fp.write('|case|fej|intr|extr|dt|slam|%s|average|\n'%('|'.join([s[:5] for s in bag_list.keys()])))
    fp.write('|'+'--|'*(len(bag_list.keys())+7)+'\n')
    params = copy.deepcopy(default_params)
    params['feat_representation'] = ('string','GLOBAL_3D')
    params['max_clones'] = ('int','11')
    run_special_case(params,'Naive',0,0,0,0,0,fp)
    run_special_case(params,'FEJ',1,0,0,0,0,fp)
    run_special_case(params,'Extrin',0,0,1,0,0,fp)
    run_special_case(params,'Extrin+Intrin',0,1,1,0,0,fp)
    run_special_case(params,'Extrin+Intrin+camdt',0,1,1,1,0,fp)
    run_special_case(params,'SLAM',0,0,0,0,50,fp)
    run_special_case(params,'All Open',1,1,1,1,50,fp)
    fp.close()
