import numpy as np
from scipy import signal
from collections import ChainMap
from tjdcs import Simulink, MIMOSim  
import copy
from collections import deque

# 配置被控对象模型
plant_model_APC = { 
                   'APC.CV1': {'APC.MV1': {'mode': 'tf_s', 'num_s': [0, 4.05], 'den_s': [20, 1], 'iodelay_s': 5},
                                'APC.MV2': {'mode': 'tf_s', 'num_s': [0, 1.77], 'den_s': [60, 1], 'iodelay_s': 8},
                                'COMMON.DV1': {'mode': 'tf_s', 'num_s': [0, 3.3], 'den_s': [20, 1], 'iodelay_s': 4},
                                },


                    'APC.CV2': {'APC.MV1': {'mode': 'tf_s', 'num_s': [0, -1.39], 'den_s': [50, 1], 'iodelay_s': 7},
                                'APC.MV2': {'mode': 'tf_s', 'num_s': [0, 5.72], 'den_s': [15, 1], 'iodelay_s': 7},
                                'COMMON.DV1': {'mode': 'tf_s', 'num_s': [0, 1.2], 'den_s': [25, 1], 'iodelay_s': 4},
                                },
                
                    'APC.CV3': {'APC.MV1': {'mode': 'tf_s', 'num_s': [0, 1.2], 'den_s': [40, 1], 'iodelay_s': 5},
                                'APC.MV2': {'mode': 'tf_s', 'num_s': [0, 1.5], 'den_s': [35, 1], 'iodelay_s': 6},
                                'COMMON.DV1': {'mode': 'tf_s', 'num_s': [0, 2.0], 'den_s': [25, 1], 'iodelay_s': 5},
                                },
                    }


plant_model_OPEN = { 
                   'OPEN.CV1': {'OPEN.MV1': {'mode': 'tf_s', 'num_s': [0, 4.05], 'den_s': [20, 1], 'iodelay_s': 5},
                                'OPEN.MV2': {'mode': 'tf_s', 'num_s': [0, 1.77], 'den_s': [60, 1], 'iodelay_s': 8},
                                'COMMON.DV1': {'mode': 'tf_s', 'num_s': [0, 3.3], 'den_s': [20, 1], 'iodelay_s': 4},
                                },


                    'OPEN.CV2': {'OPEN.MV1': {'mode': 'tf_s', 'num_s': [0, -1.39], 'den_s': [50, 1], 'iodelay_s': 7},
                                'OPEN.MV2': {'mode': 'tf_s', 'num_s': [0, 5.72], 'den_s': [15, 1], 'iodelay_s': 7},
                                'COMMON.DV1': {'mode': 'tf_s', 'num_s': [0, 1.2], 'den_s': [25, 1], 'iodelay_s': 4},
                                },
                
                    'OPEN.CV3': {'OPEN.MV1': {'mode': 'tf_s', 'num_s': [0, 1.2], 'den_s': [40, 1], 'iodelay_s': 5},
                                'OPEN.MV2': {'mode': 'tf_s', 'num_s': [0, 1.5], 'den_s': [35, 1], 'iodelay_s': 6},
                                'COMMON.DV1': {'mode': 'tf_s', 'num_s': [0, 2.0], 'den_s': [25, 1], 'iodelay_s': 5},
                                },
                    }


# 定义仿真初值（必须包含所有位号）
sim_ini_dict = {'APC.MV1': 35,
                'APC.MV2': 40,
                'APC.CV1': 41,
                'APC.CV2': 42,
                'APC.CV3': 43,

                'OPEN.MV1': 35,
                'OPEN.MV2': 40,
                'OPEN.CV1': 41,
                'OPEN.CV2': 42,
                'OPEN.CV3': 43,

                
                'COMMON._v1': 0.0,
                'COMMON._v2': 0.0,
                'COMMON._v3': 0.0,
                'COMMON.SET_v123_std': 1.0,

                'COMMON.DV1': 33,
                'COMMON.SET_DV1std': 1.0,
                
                'COMMON._OPENLOOP_CV1std_10min': 0.0,
                'COMMON._OPENLOOP_CV2std_10min': 0.0,
                'COMMON._OPENLOOP_CV3std_10min': 0.0,
                'COMMON._APC_CV1std_10min': 0.0,
                'COMMON._APC_CV2std_10min': 0.0,
                'COMMON._APC_CV3std_10min': 0.0,
                
                }


# 定义仿真流程
class Sim(Simulink):
    def __init__(self) -> None:
        super().__init__(data = sim_ini_dict)
        self.plant_apc = MIMOSim(plant_model_APC, sim_ini_dict, Ts = 1)
        self.plant_open = MIMOSim(plant_model_OPEN, sim_ini_dict, Ts = 1)
        

        # 配置输出噪声序列
        self.N = 10000
        np.random.seed(1)
        v1 = signal.lfilter([1, 0.7], np.convolve([1, -0.95], [1, -0.94]), np.random.randn(self.N))
        self.v1 = 1.0 * v1 / np.std(v1)
        v2 = signal.lfilter([1, 0.7], np.convolve([1, -0.95], [1, -0.93]), np.random.randn(self.N))
        self.v2 = 1.0 * v2 / np.std(v2)
        v3 = signal.lfilter([1, 0.7], np.convolve([1, -0.95], [1, -0.94]), np.random.randn(self.N))
        self.v3 = 1.0 * v3 / np.std(v3)

        dv1 = signal.lfilter([1, 0.7], np.convolve([1, -0.97], [1, -0.97]), np.random.randn(self.N))
        dv1 += 10*signal.lfilter([1, 0.7], np.convolve([1, -0.15], [1, -0.19]), np.random.randn(self.N))
        self.dv1 = 1.0 * dv1 / np.std(dv1)
        
        self.OPENLOOP_CV1std_10min = deque(maxlen = 600)
        self.OPENLOOP_CV2std_10min = deque(maxlen = 600)
        self.OPENLOOP_CV3std_10min = deque(maxlen = 600)
        
        self.APC_CV1std_10min = deque(maxlen = 600)
        self.APC_CV2std_10min = deque(maxlen = 600)
        self.APC_CV3std_10min = deque(maxlen = 600)
        

    def task(self) -> None:
        data = self.get_data()
        
        # 获取CV的不可测干扰 (干扰的均值为0)
        Vstd = max(0, data['COMMON.SET_v123_std'])

        k = self.get_task_count() % self.N
        v_dict = {'APC.CV1': Vstd*self.v1[k],
                  'APC.CV2': Vstd*self.v2[k],
                  'APC.CV3': Vstd*self.v3[k],
                  'OPEN.CV1': Vstd*self.v1[k],
                  'OPEN.CV2': Vstd*self.v2[k],
                  'OPEN.CV3': Vstd*self.v3[k]}
        data['COMMON._v1'] = Vstd*self.v1[k]
        data['COMMON._v2'] = Vstd*self.v2[k]
        data['COMMON._v3'] = Vstd*self.v1[k]

        # 获取DV
        data['COMMON.DV1'] = data['COMMON.SET_DV1std']*self.dv1[k] + sim_ini_dict['COMMON.DV1']

        # 计算CV
        apc_cv_dict = self.plant_apc.run(u_Value_Dict = data, v_Value_Dict = v_dict)
        data.update(apc_cv_dict)
        open_cv_dict = self.plant_open.run(u_Value_Dict = data, v_Value_Dict = v_dict)
        data.update(open_cv_dict)

        self.OPENLOOP_CV1std_10min.append(data['OPEN.CV1'])
        data['COMMON._OPENLOOP_CV1std_10min'] = np.std(list(self.OPENLOOP_CV1std_10min))  
        self.OPENLOOP_CV2std_10min.append(data['OPEN.CV2'])
        data['COMMON._OPENLOOP_CV2std_10min'] = np.std(list(self.OPENLOOP_CV2std_10min))
        self.OPENLOOP_CV3std_10min.append(data['OPEN.CV3'])
        data['COMMON._OPENLOOP_CV3std_10min'] = np.std(list(self.OPENLOOP_CV3std_10min))
        
        self.APC_CV1std_10min.append(data['APC.CV1'])
        data['COMMON._APC_CV1std_10min'] = np.std(list(self.APC_CV1std_10min))
        self.APC_CV2std_10min.append(data['APC.CV2'])
        data['COMMON._APC_CV2std_10min'] = np.std(list(self.APC_CV2std_10min))
        self.APC_CV3std_10min.append(data['APC.CV3'])
        data['COMMON._APC_CV3std_10min'] = np.std(list(self.APC_CV3std_10min))
        

# 实例化仿真对象
MIMOModel_2x3 = Sim()

if __name__ == '__main__':
    from tjdcs import SimulinkOPCGateTask
    task = SimulinkOPCGateTask(Simulator = MIMOModel_2x3, group_tag = 'T3')
    task.run()

