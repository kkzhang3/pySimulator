from collections import ChainMap
from tjdcs import MIMOSim, Simulink
import numpy as np
from scipy import signal

# 被控对象传递函数
plant_model = { 'CV1': {'MV1': {'mode': 'tf_z', 'num_z': [0, 1, 0.5], 'den_z': [1, -1.5, 0.7], 'iodelay_z': 30},
                        'DV1': {'mode': 'tf_s', 'num_s': [2.6], 'den_s': [10.0, 1.0], 'iodelay_s': 1}}}
# plant_model = { 'CV1': {'MV1': {'mode': 'tf_s', 'num_s': [0.6], 'den_s': [20.0, 1.0], 'iodelay_s': 3},
#                         'DV1': {'mode': 'tf_s', 'num_s': [2.6], 'den_s': [10.0, 1.0], 'iodelay_s': 1}}}

# 定义仿真初值（必须包含所有位号）
sim_ini_dict = {'MV1': 15,
                'CV1': 31,
                'DV1': 10,
                'v1': 0.0,
                '__Random_CV_ONOFF': 0,
                '__Random_DV_ONOFF': 0,}

# 定义仿真流程
class SISOSim(Simulink):
    def __init__(self) -> None:
        super().__init__(data = sim_ini_dict)
        self.plant1 = MIMOSim(plant_model, sim_ini_dict, Ts = 1)
        # 配置输出噪声序列 
        self.N = 10000
        np.random.seed(2)
        v1 = signal.lfilter([1], np.convolve([1, -0.95], [1, -0.95]), np.random.randn(self.N))
        self.v1 = 2.0 * v1 / np.std(v1)

        dv1 = signal.lfilter([1, 0.7], np.convolve([1, -0.95], [1, -0.95]), np.random.randn(self.N))
        self.dv1 = 2.0 * dv1 / np.std(dv1) + sim_ini_dict['DV1']

    def task(self):
        data = self.get_data()
        # 获取CV的不可测干扰 (干扰的均值为0)
        if data['__Random_CV_ONOFF'] > 0:
            k = self.get_task_count() % self.N
            v_dict = {'CV1': self.v1[k]}
        else:
            v_dict = {'CV1': data['v1']}

        # 获取DV
        if data['__Random_DV_ONOFF'] > 0:
            k = self.get_task_count() % self.N
            dv_dict = {'DV1': self.dv1[k]}
        else:
            dv_dict = {}
        data.update(dv_dict)

        # 获取MV
        mv_dict = {k:v for k,v in data.items() if k in {'MV1'}}
        # 计算CV
        y_data = self.plant1.run(u_Value_Dict = ChainMap(mv_dict, dv_dict), v_Value_Dict=v_dict)
        data.update(y_data)

SISOModel1 = SISOSim()

if __name__ == '__main__':
    from tjdcs import SimulinkOPCGateTask
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=9997, help="OPC RPC server port")
    parser.add_argument("--group", type=str, default="S1", help="OPC group tag")
    args = parser.parse_args()
    task = SimulinkOPCGateTask(
        port=args.port, group_tag=args.group, Simulator=SISOModel1)
    task.run()
