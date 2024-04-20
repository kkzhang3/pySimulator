import numpy as np
from scipy import signal
from collections import deque
from tjdcs import MIMOSim, Simulink, PID
import copy



# 被控对象传递函数
plant_model_APC = { 'APC.CV1': {    'APC.MV1':  {'mode': 'tf_s', 'num_s': [0, 0.8], 'den_s': [30, 1], 'iodelay_s': 17}, 
                                 'COMMON.DV1':  {'mode': 'tf_s', 'num_s': [0, 3.3], 'den_s': [20, 1], 'iodelay_s': 4 }}}


plant_model_PID = { 'PID.CV1': {    'PID.MV1':  {'mode': 'tf_s', 'num_s': [0, 0.8], 'den_s': [30, 1], 'iodelay_s': 17}, 
                                 'COMMON.DV1':  {'mode': 'tf_s', 'num_s': [0, 3.3], 'den_s': [20, 1], 'iodelay_s': 4 }}}

plant_model_OPEN = { 'OPEN.CV1': {    'OPEN.MV1':  {'mode': 'tf_s', 'num_s': [0, 0.8], 'den_s': [30, 1], 'iodelay_s': 17}, 
                                 'COMMON.DV1':  {'mode': 'tf_s', 'num_s': [0, 3.3], 'den_s': [20, 1], 'iodelay_s': 4 }}}




# 定义仿真初值（必须包含所有位号）
sim_ini_dict = {'APC.MV1': 29,
                'APC.CV1': 31,
                
                'PID.MV1': 29,
                'PID.CV1': 31,
                
                'OPEN.MV1': 29,
                'OPEN.CV1': 31,
               
                'PID.KP': 1.3,
                'PID.TI': 28,
                'PID.TD': 0.0,
                'PID.N': 10.0,
                
                'COMMON.OP_HI': 100.0,
                'COMMON.OP_LO': 0.0,               
                'COMMON.SP1': 31,
                'COMMON.DV1': 33,
                
                'COMMON._v1': 0.0,
                'COMMON.SET_v1std': 0.0,
                'COMMON.SET_DV1std': 0.0,
                
                'COMMON._OPENLOOP_CV1std_10min': 0.0,
                'COMMON._APC_CV1std_10min': 0.0,
                'COMMON._PID_CV1std_10min': 0.0,
                
                }

# 定义仿真流程
class SISOSim(Simulink):
    def __init__(self) -> None:
        super().__init__(data = sim_ini_dict)
        self.plant_open = MIMOSim(plant_model_OPEN, sim_ini_dict, Ts = self.get_sampling_time())
        self.plant_apc = MIMOSim(plant_model_APC, sim_ini_dict, Ts = self.get_sampling_time())
        self.plant_pid = MIMOSim(plant_model_PID, sim_ini_dict, Ts = self.get_sampling_time())
        self.pid = PID(TS = self.get_sampling_time())


        
        # 配置输出噪声序列 
        self.N = 10000
        np.random.seed(2)
        v1 = signal.lfilter([1], np.convolve([1, -0.95], [1, -0.99]), np.random.randn(self.N))
        self.v1 = 1.0 * v1 / np.std(v1)

        dv1 = signal.lfilter([1, 0.7], np.convolve([1, -0.95], [1, -0.95]), np.random.randn(self.N))
        dv1 += 10*signal.lfilter([1, 0.7], np.convolve([1, -0.15], [1, -0.19]), np.random.randn(self.N))
        self.dv1 = 1.0 * dv1 / np.std(dv1)
        
        self.OPENLOOP_CV1std_10min = deque(maxlen = 600)
        self.APC_CV1std_10min = deque(maxlen = 600)
        self.PID_CV1std_10min = deque(maxlen = 600)
        
    def task(self):
        data = self.get_data()
        # 获取CV的不可测干扰 (干扰的均值为0)
        if data['COMMON.SET_v1std'] > 0:
            k = self.get_task_count() % self.N
            vk = data['COMMON.SET_v1std']*self.v1[k]
            v_dict = {'PID.CV1': vk,
                      'APC.CV1': vk,
                      'OPEN.CV1': vk}
            data['COMMON._v1'] = vk
        else:
            v_dict = {'PID.CV1': 0.0,
                      'APC.CV1': 0.0,
                      'OPEN.CV1': 0.0}
            data['COMMON._v1'] = 0.0

        # 获取DV
        if data['COMMON.SET_DV1std'] > 0:
            k = self.get_task_count() % self.N
            data['COMMON.DV1'] = data['COMMON.SET_DV1std']*self.dv1[k] + sim_ini_dict['COMMON.DV1']

        # PID控制器
        self.pid.setPIDParam(KP = data['PID.KP'], TI = data['PID.TI'], TD = data['PID.TD'], N = data['PID.N'])
        self.pid.setOPHighLowLimit(OP_HI = data['COMMON.OP_HI'], OP_LO = data['COMMON.OP_LO'])
        data['PID.MV1'] = self.pid.run(MODE = 1, SP = data['COMMON.SP1'], PV = data['PID.CV1'], TV = data['PID.MV1'])
        
        # 被控对象
        open_loop_out = self.plant_open.run(u_Value_Dict = data, v_Value_Dict = v_dict)
        data.update(open_loop_out)
        pid_loop_out = self.plant_pid.run(u_Value_Dict = data, v_Value_Dict = v_dict)
        data.update(pid_loop_out)
        apc_loop_out = self.plant_apc.run(u_Value_Dict = data, v_Value_Dict = v_dict)
        data.update(apc_loop_out)

        # 计算标准差
        self.OPENLOOP_CV1std_10min.append(data['OPEN.CV1'])
        data['COMMON._OPENLOOP_CV1std_10min'] = np.std(list(self.OPENLOOP_CV1std_10min))        
        self.APC_CV1std_10min.append(data['APC.CV1'])
        data['COMMON._APC_CV1std_10min'] = np.std(list(self.APC_CV1std_10min))
        self.PID_CV1std_10min.append(data['PID.CV1'])
        data['COMMON._PID_CV1std_10min'] = np.std(list(self.PID_CV1std_10min))

TrainingSISO = SISOSim()

if __name__ == '__main__':
    from tjdcs import SimulinkOPCGateTask
    task = SimulinkOPCGateTask(Simulator = TrainingSISO, group_tag = 'T1')
    task.run()
