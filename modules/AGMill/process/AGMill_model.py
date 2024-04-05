import numpy as np
from scipy import signal
from collections import ChainMap
from tjdcs import Simulink, MIMOSim  

# 定义仿真初值（必须包含所有位号）
sim_ini_dict = {
                'AGMWeight': 1402,   # 磨机重量(t)
                'AGMPower': 22.1,       # 磨机功率(MW)
                'AGMCurrent': 3467,       # 磨机电流(A)
                'AGMSpeed': 9.5,       # 磨机转速(rpm)
    
                'FreshFeedWeight': 1299,   # 新鲜给料重量(t/h)
                'CRU1Weight': 559,   # 破碎机1重量(t/h)
                'BypassWeight': 0.0,  # 顽石旁路重量(t/h)
                'CYCWeight': 1449,  # 旋流器返回磨机重量(t/h)
                
                'PebbleReturn': 481.0,  # 顽石返回重量(t/h)
                'Bin1Level': 36.9,      # 顽石仓1料位(%)
                
                'F80': 120,  # 进料粒度 (mm)
               
                # 'CV1_disturbance': 0,
                # 'CV2_disturbance': 0,
                # '__Random_CV_ONOFF': 0, 
                }


# 配置被控对象模型
AGM_Power = { 'AGMPower': {
                            'FreshFeedWeight': {'mode': 'tf_s', 'num_s': [0, 1/220], 'den_s': [220, 1], 'iodelay_s': 20*60},
                            'CRU1Weight': {'mode': 'tf_s', 'num_s': [0, 1/300], 'den_s': [190, 1], 'iodelay_s': 10*60},
                            'BypassWeight': {'mode': 'tf_s', 'num_s': [0, 1/150], 'den_s': [230, 1], 'iodelay_s': 10*60},
                            'AGMSpeed': {'mode': 'tf_s', 'num_s': [0, 1/0.3], 'den_s': [220, 1], 'iodelay_s': 1*60},
                            'CYCWeight': {'mode': 'tf_s', 'num_s': [0, 1/350], 'den_s': [130, 1], 'iodelay_s': 3*60},
                            'F80': {'mode': 'tf_s', 'num_s': [0, 1/50], 'den_s': [300, 1], 'iodelay_s': 20*60},
                           },


              'AGMCurrent': {
                            'FreshFeedWeight': {'mode': 'tf_s', 'num_s': [0, 100/30], 'den_s': [180, 1], 'iodelay_s': 20*60},
                            'CRU1Weight': {'mode': 'tf_s', 'num_s': [0, 100/50], 'den_s': [150, 1], 'iodelay_s': 10*60},
                            'BypassWeight': {'mode': 'tf_s', 'num_s': [0, 100/15], 'den_s': [200, 1], 'iodelay_s': 10*60},
                            'AGMSpeed': {'mode': 'tf_s', 'num_s': [0, 100/0.15], 'den_s': [80, 1], 'iodelay_s': 1*60},
                            'CYCWeight': {'mode': 'tf_s', 'num_s': [0, 100/50], 'den_s': [100, 1], 'iodelay_s': 3*60},
                            'F80': {'mode': 'tf_s', 'num_s': [0, 100/15], 'den_s': [300, 1], 'iodelay_s': 20*60},
                           },
              }



# 定义仿真流程
class Sim(Simulink):
    def __init__(self) -> None:
        super().__init__(data = sim_ini_dict)
        self.AGM_power_sim = MIMOSim(AGM_Power, sim_ini_dict, Ts = 30)

        # 配置输出噪声序列
        # self.N = 10000
        # np.random.seed(1)
        # v1 = signal.lfilter([1, 0.7], np.convolve([1, -0.95], [1, -0.95]), np.random.randn(self.N))
        # self.v1 = 3.0 * v1 / np.std(v1)
        # v2 = signal.lfilter([1, 0.7], np.convolve([1, -0.95], [1, -0.95]), np.random.randn(self.N))
        # self.v2 = 3.0 * v2 / np.std(v2)

    def task(self) -> None:
        data = self.get_data()
        # 获取CV的不可测干扰 (干扰的均值为0)
        # if data['__Random_CV_ONOFF'] > 0:
        #     k = self.get_task_count() % self.N
        #     v_dict = {'CV1_disturbance': self.v1[k],
        #               'CV2_disturbance': self.v2[k]}

        # else:
        #     v_dict = {'CV1_disturbance': 0.0,
        #               'CV2_disturbance': 0.0}

        # 获取MV
        # mv_dict = {k:v for k, v in data.items() if k in {'MV1', 'MV2'}}

        # 计算CV
        cv_dict = self.AGM_power_sim.run(u_Value_Dict = data)
        data.update(cv_dict)

        # 组合输出至OPCGate的字典
        # write_dict = {}
        # write_dict.update(cv_dict)
        # write_dict.update(v_dict)
        # return write_dict

# 实例化仿真对象
AGMSim = Sim()



if __name__ == '__main__':
    for k in range(0,100):
        data = AGMSim.get_data()
        if k > 10:
            data['FreshFeedWeight'] = 1600
        AGMSim.run()
    
    from matplotlib import pyplot as plt
    AGMSim.plot_record_data_in_one_figure(['FreshFeedWeight'])
    # plt.show()
    
    AGMSim.plot_record_data_in_one_figure(['AGMPower'])
    plt.show()

# if __name__ == '__main__':
#     from tjdcs import SimulinkOPCGateTask
#     task = SimulinkOPCGateTask(Simulator = AGMSim, group_tag = 'S5')
#     task.run()

