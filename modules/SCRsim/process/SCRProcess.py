process_model1 = {'A_NH3_FLOW': {
                        'A_NH3_VALVE': 
                                {'mode': 'tf_s', 'num_s': [0, 0.83], 'den_s': [2, 1], 'iodelay_s': 0}},
                  'B_NH3_FLOW': {
                        'B_NH3_VALVE': 
                                {'mode': 'tf_s', 'num_s': [0, 0.77], 'den_s': [2, 1], 'iodelay_s': 0}}}

process_model2 = {'A_NOX_OUT': {
                        'A_NH3_FLOW': 
                                {'mode': 'tf_s', 'num_s': [0, -5.85], 'den_s': [200, 1], 'iodelay_s': 55},
                        'A_NOX_IN': 
                                {'mode': 'tf_s', 'num_s': [0, 0.55], 'den_s': [100, 1], 'iodelay_s': 5},
                        'POWER': 
                                {'mode': 'tf_s', 'num_s': [0, 0.22], 'den_s': [130, 1], 'iodelay_s': 10}},
                'B_NOX_OUT': {
                        'B_NH3_FLOW': 
                                {'mode': 'tf_s', 'num_s': [0, -5.11], 'den_s': [300, 1], 'iodelay_s': 65},
                        'B_NOX_IN': 
                                {'mode': 'tf_s', 'num_s': [0, 0.51], 'den_s': [250, 1], 'iodelay_s': 5},
                        'POWER': 
                                {'mode': 'tf_s', 'num_s': [0, 0.31], 'den_s': [280, 1], 'iodelay_s': 20}}}

process_model3 = {'TAIL_NOX':  {
                        'A_NOX_OUT': 
                                {'mode': 'tf_s', 'num_s': [0, 0.44], 'den_s': [400, 1], 'iodelay_s': 45},
                        'B_NOX_OUT': 
                                {'mode': 'tf_s', 'num_s': [0, 0.505], 'den_s': [400, 1], 'iodelay_s': 80}}}

# 定义仿真器初值（必须包含所有位号）
sim_ini_dict = {'A_NH3_FLOW': 40,
                'B_NH3_FLOW': 40,
                'A_NOX_OUT': 40,
                'B_NOX_OUT': 40,
                'A_NOX_IN': 200,
                'B_NOX_IN': 200,
                'A_EFFT': 90,
                'B_EFFT': 90,
                'POWER': 150,
                'TAIL_NOX': 40,
                'TAIL_NOX_COPY': 40,
                'A_NH3_VALVE': 40,
                'B_NH3_VALVE': 40,
                'A_NOX_OUT_SP': 40,
                'B_NOX_OUT_SP': 40,
                'A_NH3_FLOW_SP': 40,
                'B_NH3_FLOW_SP': 40,
                'AB_EFFT_BIAS': 0.0,
                
                '__MPCWATCHDOG': 0, 
                '__Random_DV_ONOFF': 0,  
                '__Random_CV_ONOFF': 0,}



############################################################################################################
# 定义具体的仿真流程
import sys
import numpy as np
from scipy import signal
# sys.path.insert(0, r'D:\PythonFiles\tjdcs')  # 填写tjdcs目录,tjdcs版本>=0.9.23

from tjdcs import Model
from tjdcs import TJProcSim, Simulink
import numpy as np
from scipy import signal
from collections import deque
import os

class SCRSim(Simulink):
    def __init__(self) -> None:
        super().__init__(data = sim_ini_dict)
        data = self.get_data()  # 获得数据集的引用

        # 配置DV
        path, _ = os.path.split(os.path.realpath(__file__))
        with open(os.path.join(path,'20190815-3days-1s-DVonly.csv')) as f:
            csv_data = np.loadtxt(f,float,delimiter = ",", skiprows=1, usecols = [1,2,3])
                
        self.DV_A_NOX_IN = deque(csv_data[:,0])
        self.DV_B_NOX_IN = deque(csv_data[:,1])
        self.DV_POWER = deque(csv_data[:,2])

        data.update({'A_NOX_IN': self.DV_A_NOX_IN[0],
                     'B_NOX_IN': self.DV_B_NOX_IN[0],
                     'POWER': self.DV_POWER[0],})

        data.update(self.efft_calculation())

        self.Simulator1 = TJProcSim(Model(process_model1, Ts = 1), ini_Value_Dict = data)
        self.Simulator2 = TJProcSim(Model(process_model2, Ts = 1), ini_Value_Dict = data)
        self.Simulator3 = TJProcSim(Model(process_model3, Ts = 1), ini_Value_Dict = data)

        # # 配置输出噪声序列
        N = 10000
        np.random.seed(1)
        v_A_NOX_OUT = signal.lfilter([1, 0.7], np.convolve([1, -0.95], [1, -0.95]), np.random.randn(N))
        self.v_A_NOX_OUT = deque(3.0 * v_A_NOX_OUT / np.std(v_A_NOX_OUT))
        v_B_NOX_OUT = signal.lfilter([1, 0.7], np.convolve([1, -0.95], [1, -0.95]), np.random.randn(N))
        self.v_B_NOX_OUT = deque(3.0 * v_B_NOX_OUT / np.std(v_B_NOX_OUT))
        v_TAIL_NOX = signal.lfilter([1, 0.7], np.convolve([1, -0.95], [1, -0.95]), np.random.randn(N))
        self.v_TAIL_NOX = deque(3.0 * v_TAIL_NOX / np.std(v_TAIL_NOX))
        v_A_NH3_FLOW = signal.lfilter([1, 0.7], np.convolve([1, -0.95], [1, -0.95]), np.random.randn(N))
        self.v_A_NH3_FLOW = deque(3.0 * v_A_NH3_FLOW / np.std(v_A_NH3_FLOW))
        v_B_NH3_FLOW = signal.lfilter([1, 0.7], np.convolve([1, -0.95], [1, -0.95]), np.random.randn(N))
        self.v_B_NH3_FLOW = deque(3.0 * v_B_NH3_FLOW / np.std(v_B_NH3_FLOW))

    def task(self) -> None:
        data = self.get_data()

        # self._calculate_taglist = ['A_NH3_FLOW', 'B_NH3_FLOW', 'A_NOX_OUT', 'B_NOX_OUT',
        #                            'A_EFFT', 'B_EFFT', 'TAIL_NOX', 'TAIL_NOX_COPY',
        #                            '__MPCWATCHDOG', 'AB_EFFT_BIAS' ]

        # 配置输出端的不可测干扰 (初值的偏置)
        v_Value_Dict = {}
        if data['__Random_CV_ONOFF'] > 0:
            v_Value_Dict.update({'A_NH3_FLOW': self.v_A_NH3_FLOW[0],
                                 'B_NH3_FLOW': self.v_B_NH3_FLOW[0],
                                 'A_NOX_OUT': self.v_A_NOX_OUT[0],
                                 'B_NOX_OUT': self.v_B_NOX_OUT[0],
                                 'TAIL_NOX': self.v_TAIL_NOX[0]})
            self.v_A_NH3_FLOW.rotate(-1)
            self.v_B_NH3_FLOW.rotate(-1)
            self.v_A_NOX_OUT.rotate(-1)
            self.v_B_NOX_OUT.rotate(-1)
            self.v_TAIL_NOX.rotate(-1)

        # 配置DV方法2
        if data['__Random_DV_ONOFF'] > 0:
            data.update({'A_NOX_IN': self.DV_A_NOX_IN[0],
                                       'B_NOX_IN': self.DV_B_NOX_IN[0],
                                       'POWER': self.DV_POWER[0],})
            self.DV_A_NOX_IN.rotate(-1)
            self.DV_B_NOX_IN.rotate(-1)
            self.DV_POWER.rotate(-1)
        #     self._calculate_taglist += ['A_NOX_IN', 'B_NOX_IN', 'POWER']

        # 调用仿真器    
        y_Value_Dict1 = self.Simulator1(u_Value_Dict=data, v_Value_Dict=v_Value_Dict)
        data.update(y_Value_Dict1)

        y_Value_Dict2 = self.Simulator2(u_Value_Dict=data, v_Value_Dict=v_Value_Dict)
        data.update(y_Value_Dict2)

        y_Value_Dict3 = self.Simulator3(u_Value_Dict=data, v_Value_Dict=v_Value_Dict)
        data.update(y_Value_Dict3)
        data.update(self.efft_calculation())
        # 更新看门狗信息
        data.update({'__MPCWATCHDOG': self.get_task_count()})


    def efft_calculation(self):
        data = self.get_data()
        A_NOX_IN = max(1,data['A_NOX_IN'])
        B_NOX_IN = max(1,data['B_NOX_IN']) 
        A_NOX_OUT = max(1,data['A_NOX_OUT'])
        B_NOX_OUT = max(1,data['B_NOX_OUT']) 
        A_EFFT = 100*(A_NOX_IN - A_NOX_OUT)/A_NOX_IN
        B_EFFT = 100*(B_NOX_IN - B_NOX_OUT)/B_NOX_IN
        AB_EFFT_BIAS = A_EFFT - B_EFFT
        return {'A_EFFT': A_EFFT, 
                'B_EFFT': B_EFFT,
                'TAIL_NOX_COPY': data['TAIL_NOX'],
                'AB_EFFT_BIAS': AB_EFFT_BIAS}

SCRModel = SCRSim()
#####################################################################################
# 如果直接运行此程序，则在OPCgate中创建该被控对象的仿真器（OPC通讯配置）
if __name__ == '__main__':
    from tjdcs import SimulinkOPCGateTask
    task = SimulinkOPCGateTask('127.0.0.1', 9999, SCRModel, group_tag='SCR2')
    task.run()


