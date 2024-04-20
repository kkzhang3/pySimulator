from collections import ChainMap
from tjdcs import MIMOSim, Simulink, PID
import numpy as np
from scipy import signal



# 被控对象传递函数
plant_model_APC = { 'APC.CV1': {'APC.MV1':      {'mode': 'tf_s', 'num_s': [0, 0.08], 'den_s': [30, 1], 'iodelay_s': 17, 'IntegralMark': 1}, 
                                'COMMON.DV1':  {'mode': 'tf_s', 'num_s': [0, 0.21], 'den_s': [20, 1], 'iodelay_s': 4, 'IntegralMark': 1 }}}

plant_model_PID = { 'PID.CV1': {'PID.MV1':      {'mode': 'tf_s', 'num_s': [0, 0.08], 'den_s': [30, 1], 'iodelay_s': 17, 'IntegralMark': 1},
                                'COMMON.DV1':  {'mode': 'tf_s', 'num_s': [0, 0.21], 'den_s': [20, 1], 'iodelay_s': 4, 'IntegralMark': 1 }}}


# 定义仿真初值（必须包含所有位号）
sim_ini_dict = {'APC.MV1': 29,
                'APC.CV1': 51,

                'PID.MV1': 29,
                'PID.CV1': 51,
                'PID.KP': 0.2,
                'PID.TI': 150,
                'PID.TD': 20.0,
                'PID.N': 100.0,
                
                'COMMON.OP_HI': 100.0,
                'COMMON.OP_LO': 0.0,               
                'COMMON.SP1': 51,
                'COMMON.DV1': 33,
                
                'COMMON.__v1': 0.0,
                'COMMON.__Random_CV_ONOFF': 0,
                'COMMON.__Random_DV_ONOFF': 0,}


class AntiWindupLimter:
    def __init__(self):
        self._u0 = None
        self._y0 = None
        
    def __call__(self, u:float, HI = 100, LO = 0) -> float:
        if self._u0 is None:
            self._u0 = u

        if self._y0 is None:
            self._y0 = min(max(u, LO), HI)
            
        delta_u = u - self._u0
        y = min(max(self._y0 + delta_u, LO), HI)
        self._u0 = u
        self._y0 = y
        return y

    def run(self, u, HI = 100, LO = 0) -> float:
        return self.__call__(u, HI, LO)     

# 定义仿真流程
class SISOSim(Simulink):
    def __init__(self) -> None:
        super().__init__(data = sim_ini_dict)
        self.plant_apc = MIMOSim(plant_model_APC, sim_ini_dict, Ts = self.get_sampling_time())
        self.plant_pid = MIMOSim(plant_model_PID, sim_ini_dict, Ts = self.get_sampling_time())
        self.limiter1 = AntiWindupLimter()
        self.limiter2 = AntiWindupLimter()
        
        self.pid = PID(TS = self.get_sampling_time())
        
        # 配置输出噪声序列 
        self.N = 10000
        np.random.seed(85)
        v1 = signal.lfilter([1], np.convolve([1, -0.99], [1, -0.99]), np.random.randn(self.N))
        self.v1 = 2.0 * v1 / np.std(v1)

        dv1 = signal.lfilter([1, 0.7], np.convolve([1, -0.92], [1, -0.92]), np.random.randn(self.N))
        dv1 += 10*signal.lfilter([1, 0.7], np.convolve([1, -0.15], [1, -0.19]), np.random.randn(self.N))
        dv1 -= np.mean(dv1)
        
        self.dv1 = 0.3 * dv1 / np.std(dv1) + sim_ini_dict['COMMON.DV1']

    def task(self):
        data = self.get_data()
        # 获取CV的不可测干扰 (干扰的均值为0)
        if data['COMMON.__Random_CV_ONOFF'] > 0:
            k = self.get_task_count() % self.N
            v_dict = {'PID.CV1': self.v1[k],
                      'APC.CV1': self.v1[k]}
        else:
            v_dict = {'PID.CV1': 0.0,
                      'APC.CV1': 0.0}

        # 获取DV
        if data['COMMON.__Random_DV_ONOFF'] > 0:
            k = self.get_task_count() % self.N
            data['COMMON.DV1'] = self.dv1[k]

        self.pid.setPIDParam(KP = data['PID.KP'], TI = data['PID.TI'], TD = data['PID.TD'], N = data['PID.N'])
        self.pid.setOPHighLowLimit(OP_HI = data['COMMON.OP_HI'], OP_LO = data['COMMON.OP_LO'])
        data['PID.MV1'] = self.pid.run(MODE = 1, SP = data['COMMON.SP1'], PV = data['PID.CV1'], TV = data['PID.MV1'])
        
        pid_loop_out = self.plant_pid.run(u_Value_Dict = data, v_Value_Dict = v_dict)
        pid_cv1 = self.limiter1.run(pid_loop_out['PID.CV1'], 100, 0)
        data.update({'PID.CV1': pid_cv1})
        
        apc_loop_out = self.plant_apc.run(u_Value_Dict = data, v_Value_Dict = v_dict)
        apc_cv1 = self.limiter2.run(apc_loop_out['APC.CV1'], 100, 0)
        data.update({'APC.CV1': apc_cv1})

TrainingSISO = SISOSim()

if __name__ == '__main__':
    from tjdcs import SimulinkOPCGateTask
    task = SimulinkOPCGateTask(Simulator = TrainingSISO, group_tag = 'T2')
    task.run()
