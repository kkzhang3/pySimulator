import numpy as np
from scipy import signal
from collections import ChainMap
from tjdcs import Simulink, MIMOSim  

def FreshFeed2PebbleReturn(FreshFeedWeight, F80):
    FxF80 = np.interp(F80, [80, 220], [0.1, 0.7]) 
    return FreshFeedWeight*FxF80

def Bypass2PebbleReturn(BypassWeight, rate = 0.7):
    return BypassWeight*rate

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
                
                'PebbleReturn': 650.0,  # 顽石返回重量(t/h)
                'PebbleReturn_other': 0.0,
                'Bin1Level': 36.9,      # 顽石仓1料位(%)
                
                'F80': 140,  # 进料粒度 (mm)
               
                '__Random_DV_ONOFF': 1, 
                }

sim_ini_dict.update({
    'FreshFeed2Pebble': FreshFeed2PebbleReturn(sim_ini_dict['FreshFeedWeight'], sim_ini_dict['F80']),
    'Bypass2Pebble': Bypass2PebbleReturn(sim_ini_dict['BypassWeight'])
})


# 配置被控对象模型
AGMPower_Model = {
              'AGMPower': {
                            'FreshFeedWeight': {'mode': 'tf_s', 'num_s': [0, 1/220], 'den_s': [220, 1], 'iodelay_s': 20*60},
                            'CRU1Weight': {'mode': 'tf_s', 'num_s': [0, 1/300], 'den_s': [190, 1], 'iodelay_s': 10*60},
                            'BypassWeight': {'mode': 'tf_s', 'num_s': [0, 1/150], 'den_s': [230, 1], 'iodelay_s': 10*60},
                            'AGMSpeed': {'mode': 'tf_s', 'num_s': [0, 1/0.3], 'den_s': [220, 1], 'iodelay_s': 1*60},
                            'CYCWeight': {'mode': 'tf_s', 'num_s': [0, 1/400], 'den_s': [130, 1], 'iodelay_s': 3*60},
                            'F80': {'mode': 'tf_s', 'num_s': [0, 1/60], 'den_s': [300, 1], 'iodelay_s': 20*60},
                           },

              'AGMCurrent': {
                            'FreshFeedWeight': {'mode': 'tf_s', 'num_s': [0, 100/75], 'den_s': [180, 1], 'iodelay_s': 20*60},
                            'CRU1Weight': {'mode': 'tf_s', 'num_s': [0, 100/160], 'den_s': [150, 1], 'iodelay_s': 10*60},
                            'BypassWeight': {'mode': 'tf_s', 'num_s': [0, 100/65], 'den_s': [200, 1], 'iodelay_s': 10*60},
                            'AGMSpeed': {'mode': 'tf_s', 'num_s': [0, 100/0.15], 'den_s': [80, 1], 'iodelay_s': 1*60},
                            'CYCWeight': {'mode': 'tf_s', 'num_s': [0, 100/210], 'den_s': [100, 1], 'iodelay_s': 3*60},
                            'F80': {'mode': 'tf_s', 'num_s': [0, 100/35], 'den_s': [300, 1], 'iodelay_s': 20*60},
                           },
              
              'AGMWeight': {
                            'FreshFeedWeight': {'mode': 'tf_s', 'num_s': [0, 0.25], 'den_s': [120, 1], 'iodelay_s': 20*60},
                            'CRU1Weight': {'mode': 'tf_s', 'num_s': [0, 0.1], 'den_s': [120, 1], 'iodelay_s': 10*60},
                            'BypassWeight': {'mode': 'tf_s', 'num_s': [0, 0.4], 'den_s': [120, 1], 'iodelay_s': 10*60},
                            'CYCWeight': {'mode': 'tf_s', 'num_s': [0, 0.1], 'den_s': [80, 1], 'iodelay_s': 3*60},
                           },
              }


PebbleReturn_Model = {
    'PebbleReturn': {
        'FreshFeed2Pebble': {'mode': 'tf_s', 'num_s': [0, 1], 'den_s': [250, 1], 'iodelay_s': 20*60},
        'BypassWeight': {'mode': 'tf_s', 'num_s': [0, 1], 'den_s': [230, 1], 'iodelay_s': 10*60},
        'AGMSpeed': {'mode': 'tf_s', 'num_s': [0, 100/0.2], 'den_s': [200, 1], 'iodelay_s': 1*60},
        'PebbleReturn_other': {'mode': 'tf_s', 'num_s': [0, 1], 'den_s': [20, 1], 'iodelay_s': 0}
    },
}


BinLevel_Model = {
    'Bin1Level': {
        'PebbleReturn': {'mode': 'tf_s', 'num_s': [0, 0.01/100], 'den_s': [80, 1], 'iodelay_s': 1*60, 'IntegralMark': 1},
        'BypassWeight': {'mode': 'tf_s', 'num_s': [0, -0.01/100], 'den_s': [60, 1], 'iodelay_s': 1*60, 'IntegralMark': 1},
        'CRU1Weight': {'mode': 'tf_s', 'num_s': [0, -0.01/100], 'den_s': [80, 1], 'iodelay_s': 1*60, 'IntegralMark': 1},
    }
}


# 定义仿真流程
class Sim(Simulink):
    def __init__(self) -> None:
        super().__init__(data = sim_ini_dict)
        self.AGMpower_sim = MIMOSim(AGMPower_Model, sim_ini_dict, Ts = 30)
        self.RebbleReturn_sim = MIMOSim(PebbleReturn_Model, sim_ini_dict, Ts = 30)
        self.BinLevel_sim = MIMOSim(BinLevel_Model, sim_ini_dict, Ts = 30)

        # 配置输出噪声序列
        self.N = 10000
        np.random.seed(1)
        v1 = signal.lfilter([1, 0.7], np.convolve([1, -0.9], [1, -0.95]), np.random.randn(self.N))
        self.CRU1Weight_random = sim_ini_dict['CRU1Weight'] + 70.0*v1/np.std(v1)

        v2 = signal.lfilter([1, 0.7], np.convolve([1, -0.98], [1, -0.97]), np.random.randn(self.N))
        self.CYCWeight_random = sim_ini_dict['CYCWeight'] + 70.0 * v2 / np.std(v2)

        v3 = signal.lfilter([1, 0.7], np.convolve([1, -0.99], [1, -0.99]), np.random.randn(self.N))
        self.F80_random = sim_ini_dict['F80'] + 10.0 * v3 / np.std(v3)

    def task(self) -> None:
        data = self.get_data()

        if data['__Random_DV_ONOFF'] > 0:
            k = self.get_task_count() % self.N
            data['CRU1Weight'] = self.CRU1Weight_random[k]
            data['CYCWeight'] = self.CYCWeight_random[k]
            data['F80'] = self.F80_random[k]

        # 计算CV
        out_dict = self.AGMpower_sim.run(u_Value_Dict = data)
        data.update(out_dict)

        data.update({
            'FreshFeed2Pebble': FreshFeed2PebbleReturn(data['FreshFeedWeight'], data['F80']),
            'Bypass2Pebble': Bypass2PebbleReturn(data['BypassWeight'])
        })

        out_dict2 = self.RebbleReturn_sim.run(u_Value_Dict = data)
        data.update(out_dict2)
                
        out_dict3 = self.BinLevel_sim.run(u_Value_Dict = data)
        data.update(out_dict3)

        data['Bin1Level'] = max(min(data['Bin1Level'], 210), 0)


# 实例化仿真对象
AGMSim = Sim()

# if __name__ == '__main__':
#     for k in range(0,1240*2):
#         data = AGMSim.get_data()
#         data['__Random_DV_ONOFF'] = 1
#         data['FreshFeedWeight'] = 1599
#         # if k > 300:
#         #     data['BypassWeight'] = 300
#         # #     data['FreshFeedWeight'] = 1340
#         AGMSim.run()
    
#     AGMSim.plot_record_data(taglist=['CRU1Weight','CYCWeight', 'F80'])
#     AGMSim.plot_record_data(taglist=['AGMPower', 'AGMCurrent'])
    
#     AGMSim.plot_record_data(taglist=['PebbleReturn', 'Bin1Level'])
    
#     from matplotlib import pyplot as plt
#     plt.show()
    
    

if __name__ == '__main__':
    from tjdcs import SimulinkOPCGateTask
    task = SimulinkOPCGateTask(Simulator = AGMSim, group_tag = 'S5')
    task.run()

