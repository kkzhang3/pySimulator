import numpy as np
from scipy import signal
from collections import ChainMap
from tjdcs import Simulink, MIMOSim  

# 配置被控对象模型
plant_model = { 'CV1': {'MV1': {'mode': 'tf_s', 'num_s': [0, 4.05], 'den_s': [20, 1], 'iodelay_s': 5},
                        'MV2': {'mode': 'tf_s', 'num_s': [0, 1.77], 'den_s': [60, 1], 'iodelay_s': 8},},


                'CV2': {'MV1': {'mode': 'tf_s', 'num_s': [0, -1.39], 'den_s': [50, 1], 'iodelay_s': 7},
                        'MV2': {'mode': 'tf_s', 'num_s': [0, 5.72], 'den_s': [15, 1], 'iodelay_s': 7},},}


# 定义仿真初值（必须包含所有位号）
sim_ini_dict = {'MV1': 15,
                'MV2': 20,
                'CV1': 31,
                'CV2': 32,
                'SP1': 31,
                'SP2': 32,
                'CV1_disturbance': 0,
                'CV2_disturbance': 0,
                '__Random_CV_ONOFF': 0, 
                }


# 定义仿真流程
class Sim(Simulink):
    def __init__(self) -> None:
        super().__init__(data = sim_ini_dict)
        self.plant = MIMOSim(plant_model, sim_ini_dict, Ts = 1)

        # 配置输出噪声序列
        self.N = 10000
        np.random.seed(1)
        v1 = signal.lfilter([1, 0.7], np.convolve([1, -0.95], [1, -0.95]), np.random.randn(self.N))
        self.v1 = 3.0 * v1 / np.std(v1)
        v2 = signal.lfilter([1, 0.7], np.convolve([1, -0.95], [1, -0.95]), np.random.randn(self.N))
        self.v2 = 3.0 * v2 / np.std(v2)

    def task(self) -> None:
        data = self.get_data()
        # 获取CV的不可测干扰 (干扰的均值为0)
        if data['__Random_CV_ONOFF'] > 0:
            k = self.get_task_count() % self.N
            v_dict = {'CV1_disturbance': self.v1[k],
                      'CV2_disturbance': self.v2[k]}

        else:
            v_dict = {'CV1_disturbance': 0.0,
                      'CV2_disturbance': 0.0}

        # 获取MV
        mv_dict = {k:v for k, v in data.items() if k in {'MV1', 'MV2'}}

        # 计算CV
        cv_dict = self.plant.run(u_Value_Dict = ChainMap(mv_dict), 
                                 v_Value_Dict = {k.rstrip('_disturbance'):v for k,v in v_dict.items()})
        data.update(cv_dict)

        # 组合输出至OPCGate的字典
        write_dict = {}
        write_dict.update(cv_dict)
        write_dict.update(v_dict)
        return write_dict

# 实例化仿真对象
MIMOModel_2x2 = Sim()

if __name__ == '__main__':
    from tjdcs import SimulinkOPCGateTask
    task = SimulinkOPCGateTask(Simulator = MIMOModel_2x2, group_tag = 'S3')
    task.run()

