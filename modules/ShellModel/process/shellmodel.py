############################################################################################################
# 1. 本文件定义了一个壳牌精馏塔模型（shell process）
# 2. 文件中的类：ShellSim 定义了模型的响应，可以被实例化后直接使用（一般用于高速仿真调试）
# 3. 若直接运行此python文件，则会在TaiJiOPCGate中创建ShellModel仿真器（一般用于常规仿真）
#   3.1 TaiJiOPCGate中的'__Process_Random_CV_ONOFF'与'__Process_Random_DV_ONOFF' 位号
#       可以控制ShellModel中的随机干扰
############################################################################################################

# 配置被控对象模型
plant_model = { 'CV1': {'MV1': {'mode': 'tf_s', 'num_s': [0, 4.05], 'den_s': [50, 1], 'iodelay_s': 25},
                        'MV2': {'mode': 'tf_s', 'num_s': [0, 1.77], 'den_s': [60, 1], 'iodelay_s': 27},
                        'MV3': {'mode': 'tf_s', 'num_s': [0, 5.88], 'den_s': [50, 1], 'iodelay_s': 25},
                        'DV1': {'mode': 'tf_s', 'num_s': [0, 1.20], 'den_s': [45, 1], 'iodelay_s': 25},
                        'DV2': {'mode': 'tf_s', 'num_s': [0, 1.44], 'den_s': [40, 1], 'iodelay_s': 25}},

                'CV2': {'MV1': {'mode': 'tf_s', 'num_s': [0, 5.39], 'den_s': [50, 1], 'iodelay_s': 17},
                        'MV2': {'mode': 'tf_s', 'num_s': [0, 5.72], 'den_s': [60, 1], 'iodelay_s': 17},
                        'MV3': {'mode': 'tf_s', 'num_s': [0, 6.90], 'den_s': [40, 1], 'iodelay_s': 15},
                        'DV1': {'mode': 'tf_s', 'num_s': [0, 1.52], 'den_s': [25, 1], 'iodelay_s': 15},
                        'DV2': {'mode': 'tf_s', 'num_s': [0, 1.83], 'den_s': [20, 1], 'iodelay_s': 15}},

                'CV3': {'MV1': {'mode': 'tf_s', 'num_s': [0, 3.66], 'den_s': [9, 1], 'iodelay_s': 4},
                        'MV2': {'mode': 'tf_s', 'num_s': [0, 1.65], 'den_s': [30, 1], 'iodelay_s': 4},
                        'MV3': {'mode': 'tf_s', 'num_s': [0, 5.53], 'den_s': [40, 1], 'iodelay_s': 6},
                        'DV1': {'mode': 'tf_s', 'num_s': [0, 1.16], 'den_s': [11, 1], 'iodelay_s': 4},
                        'DV2': {'mode': 'tf_s', 'num_s': [0, 1.27], 'den_s': [ 6, 1], 'iodelay_s': 4}},

                'CV4': {'MV1': {'mode': 'tf_s', 'num_s': [0, 5.92], 'den_s': [12, 1], 'iodelay_s': 8},
                        'MV2': {'mode': 'tf_s', 'num_s': [0, 2.54], 'den_s': [27, 1], 'iodelay_s': 9},
                        'MV3': {'mode': 'tf_s', 'num_s': [0, 8.10], 'den_s': [20, 1], 'iodelay_s': 9},
                        'DV1': {'mode': 'tf_s', 'num_s': [0, 1.73], 'den_s': [ 5, 1], 'iodelay_s': 8},
                        'DV2': {'mode': 'tf_s', 'num_s': [0, 1.79], 'den_s': [19, 1], 'iodelay_s': 9}},

                'CV5': {'MV1': {'mode': 'tf_s', 'num_s': [0, 4.13], 'den_s': [8, 1], 'iodelay_s': 5},
                        'MV2': {'mode': 'tf_s', 'num_s': [0, 2.38], 'den_s': [19, 1], 'iodelay_s': 7},
                        'MV3': {'mode': 'tf_s', 'num_s': [0, 6.23], 'den_s': [10, 1], 'iodelay_s': 5},
                        'DV1': {'mode': 'tf_s', 'num_s': [0, 1.31], 'den_s': [ 2, 1], 'iodelay_s': 5},
                        'DV2': {'mode': 'tf_s', 'num_s': [0, 1.26], 'den_s': [22, 1], 'iodelay_s': 5}},

                'CV6': {'MV1': {'mode': 'tf_s', 'num_s': [0, 4.06], 'den_s': [13, 1], 'iodelay_s': 7},
                        'MV2': {'mode': 'tf_s', 'num_s': [0, 4.18], 'den_s': [33, 1], 'iodelay_s': 7},
                        'MV3': {'mode': 'tf_s', 'num_s': [0, 6.53], 'den_s': [9, 1], 'iodelay_s': 5},
                        'DV1': {'mode': 'tf_s', 'num_s': [0, 1.19], 'den_s': [19, 1], 'iodelay_s': 5},
                        'DV2': {'mode': 'tf_s', 'num_s': [0, 1.17], 'den_s': [24, 1], 'iodelay_s': 5}},

                'CV7': {'MV1': {'mode': 'tf_s', 'num_s': [0, 4.38], 'den_s': [33, 1], 'iodelay_s': 15},
                        'MV2': {'mode': 'tf_s', 'num_s': [0, 4.42], 'den_s': [44, 1], 'iodelay_s': 15},
                        'MV3': {'mode': 'tf_s', 'num_s': [0, 7.20], 'den_s': [19, 1], 'iodelay_s': 15},
                        'DV1': {'mode': 'tf_s', 'num_s': [0, 1.14], 'den_s': [27, 1], 'iodelay_s': 16},
                        'DV2': {'mode': 'tf_s', 'num_s': [0, 1.26], 'den_s': [32, 1], 'iodelay_s': 17}}}


# 定义仿真初值（必须包含所有位号）
sim_ini_dict = {'MV1': 15,
                'MV2': 20,
                'MV3': 35,
                'DV1': 10,
                'DV2': 10,
                'CV1': 31,
                'CV2': 32,
                'CV3': 33,
                'CV4': 34,
                'CV5': 35,
                'CV6': 36,
                'CV7': 37,
                '__Random_DV_ONOFF': 0,  
                '__Random_CV_ONOFF': 0, 
                }

import sys
import numpy as np
from scipy import signal
# sys.path.insert(0, r'D:\PythonFiles\tjdcs')  # 填写tjdcs目录,tjdcs版本>=0.9.23
from collections import ChainMap
from tjdcs import Simulink, TJProcSim2  

# 定义仿真流程
class ShellSim(Simulink):
    def __init__(self) -> None:
        super().__init__(data = sim_ini_dict)
        self.plant = TJProcSim2(plant_model, sim_ini_dict, Ts = 1)

        # 配置输出噪声序列
        self.N = 10000
        np.random.seed(1)
        v1 = signal.lfilter([1, 0.7], np.convolve([1, -0.95], [1, -0.95]), np.random.randn(self.N))
        self.v1 = 3.0 * v1 / np.std(v1)
        v2 = signal.lfilter([1, 0.7], np.convolve([1, -0.95], [1, -0.95]), np.random.randn(self.N))
        self.v2 = 3.0 * v2 / np.std(v2)
        v3 = signal.lfilter([1, 0.7], np.convolve([1, -0.95], [1, -0.95]), np.random.randn(self.N))
        self.v3 = 3.0 * v3 / np.std(v3)
        v4 = signal.lfilter([1, 0.7], np.convolve([1, -0.95], [1, -0.95]), np.random.randn(self.N))
        self.v4 = 3.0 * v4 / np.std(v4)
        v5 = signal.lfilter([1, 0.7], np.convolve([1, -0.95], [1, -0.95]), np.random.randn(self.N))
        self.v5 = 3.0 * v5 / np.std(v5)
        v6 = signal.lfilter([1, 0.7], np.convolve([1, -0.95], [1, -0.95]), np.random.randn(self.N))
        self.v6 = 3.0 * v6 / np.std(v6)
        v7 = signal.lfilter([1, 0.7], np.convolve([1, -0.95], [1, -0.95]), np.random.randn(self.N))
        self.v7 = 3.0 * v7 / np.std(v7)

        # 配置可测干扰序列
        dv1 = signal.lfilter([1, 0.7], np.convolve([1, -0.95], [1, -0.95]), np.random.randn(self.N))
        self.dv1 = 2.0 * dv1 / np.std(dv1) + sim_ini_dict['DV1']
        dv2 = signal.lfilter([1, 0.7], np.convolve([1, -0.95], [1, -0.95]), np.random.randn(self.N))
        self.dv2 = 1.5 * dv2 / np.std(dv2) + sim_ini_dict['DV2']

    def task(self) -> None:
        data = self.get_data()
        # 获取CV的不可测干扰 (干扰的均值为0)
        if data['__Random_CV_ONOFF'] > 0:
            k = self.get_task_count() % self.N
            v_dict = {'CV1': self.v1[k],
                      'CV2': self.v2[k],
                      'CV3': self.v3[k],
                      'CV4': self.v4[k],
                      'CV5': self.v5[k],
                      'CV6': self.v6[k],
                      'CV7': self.v7[k],}
        else:
            v_dict = {}

        # 获取DV
        if data['__Random_DV_ONOFF'] > 0:
            k = self.get_task_count() % self.N
            dv_dict = {'DV1': self.dv1[k],
                       'DV2': self.dv2[k],}
        else:
            dv_dict = {}
        data.update(dv_dict)

        # 获取MV
        mv_dict = {k:v for k, v in data.items() if k in {'MV1', 'MV2', 'MV3'}}

        # 计算CV
        cv_dict = self.plant.run(u_Value_Dict = ChainMap(mv_dict, dv_dict), v_Value_Dict=v_dict)
        data.update(cv_dict)

ShellModel = ShellSim()

if __name__ == '__main__':
    from tjdcs import SimulinkOPCGateTask
    task = SimulinkOPCGateTask(Simulator = ShellModel, group_tag = '')
    task.run()

