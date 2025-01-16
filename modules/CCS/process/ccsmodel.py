# import sys
# sys.path.insert(0, r'D:\PythonFiles\tjdcs')  # 填写tjdcs目录
# import tjdcs
# print(f'{tjdcs.__version__ = }')

import numpy as np
from tjdcs import MIMOSim, Simulink, RateLimit, sFilter

#### 协调被控对象模型  #####
# MVM-BD: , CVMW: , Grade: A, Reduction Error PCT:   1.10

Gain = 1.6227E+000 
B = 2.3393E+001 
A2 = 1.6493E+004 
A1 = 3.7437E+002
Delay = 88.000

Bs_MW_BD = np.convolve([Gain],[B, 1])
As_MW_BD = [A2, A1, 1]
Delay_MW_BD = int(Delay)

# MVM-BD: , CVMSPS: , Grade: A, Reduction Error PCT:   0.88
Gain = 1.0908E-001
B = 1.5461E+001
A2 = 9.3896E+003
A1 = 2.4259E+002
Delay = 62.000

Bs_PR_BD = np.convolve([Gain],[B, 1])
As_PR_BD = [A2, A1, 1]
Delay_PR_BD = int(Delay)

# MVM-TD: , CVMW: , Grade: A, Reduction Error PCT:   0.28
Gain = 1.5848E-001
B2 = 5.8181E+004
B1 = 6.0043E+003
A3 = 5.0717E+003
A2 = 4.1963E+003
A1 = 2.6089E+002
Delay =  0.000

Bs_MW_TD = np.convolve([Gain],[B2, B1, 1])
As_MW_TD = [A3, A2, A1, 1]
Delay_MW_TD = int(Delay)

# MVM-TD: , CVMSPS: , Grade: A, Reduction Error PCT:   0.76
Gain =-4.4325E-001
B = 6.0887E+001
A2 = 3.6276E+003
A1 = 1.9385E+002
Delay =  1.000

Bs_PR_TD = np.convolve([Gain],[B, 1])
As_PR_TD = [A2, A1, 1]
Delay_PR_TD = int(Delay)

ccs_plant = {'CCS.MW':{'CCS.BD': {'mode': 'tf_s', 'num_s': Bs_MW_BD, 'den_s': As_MW_BD, 'iodelay_s': Delay_MW_BD},
                       'CCS.TD': {'mode': 'tf_s', 'num_s': Bs_MW_TD, 'den_s': As_MW_TD, 'iodelay_s': Delay_MW_TD}},
             'CCS.PR':{'CCS.BD': {'mode': 'tf_s', 'num_s': Bs_PR_BD, 'den_s': As_PR_BD, 'iodelay_s': Delay_PR_BD}, 
                       'CCS.TD': {'mode': 'tf_s', 'num_s': Bs_PR_TD, 'den_s': As_PR_TD, 'iodelay_s': Delay_PR_TD}}}




# 定义仿真器初值（必须包含所有位号）
sim_ini_dict = {'CCS.MW': 175.0,   # 实发功率
                'CCS.PR': 15.0,    # 主汽压力
                'CCS.BD': 100.0,   # 锅炉主控
                'CCS.TD': 50.0,    # 汽机主控
                'CCS.MW_TARGET': 175.0,  # 负荷设定终点值
                'CCS.MW_SP': 175.0,    # 负荷设定当前值（速率后负荷指令）
                'CCS.PR_TARGET': 15.0,   # 主汽压力设定终点值
                'CCS.MW_SLOPE': 2.0,  # 变负荷速率 (MW/min)
                'CCS.PR_SP': 15.0,    # 主汽压力设定当前值（速率后主汽压力设定）
                'CCS.PR_SP_BIAS': 0.0,  # 主汽压力设定值偏置
                'CCS.MW_SP_BIAS': 0.0,  # 负荷指定设定值偏置
                }

# CCS滑压曲线
MW_PR_Fxy_x = [175, 230, 300,  350]
MW_PR_Fxy_y = [15,  20,  23.5, 24.2]

# CCS的Simulink (包含协调被控对象仿真, 设定值逻辑)
class CCSSim(Simulink):
    def __init__(self) -> None:
        super().__init__(data = sim_ini_dict)

        # 配置被控对象、滤波器、速率限制器等
        self.Simulator1 = MIMOSim(ccs_plant, sim_ini_dict, Ts = 1)
        self.rateLim1 = RateLimit(u_ini = sim_ini_dict['CCS.MW_SP'])
        self.sfilter1 = sFilter(Bs = [1], As = [60, 1], 
                                u_ini = sim_ini_dict['CCS.PR_TARGET'], 
                                y_ini = sim_ini_dict['CCS.PR_TARGET'])

    def task(self) -> dict:

        data_dict = self.get_data()
        # 调用限速率模块计算速率后负荷设定值：
        # 'CCS.MW_TARGET', 'CCS.MW_SP_BIAS', 'CCS.MW_SLOPE' -> 'CCS.MW_SP'
        MW_SP = self.rateLim1(u_end = data_dict['CCS.MW_TARGET'] + data_dict['CCS.MW_SP_BIAS'],
                              PRLimPerMin = data_dict['CCS.MW_SLOPE'])
        
        # 调用Fxy模块计算主汽压力设定终点值(滑压曲线)： 
        # 'CCS.MW_TARGET', 'CCS.MW_SP_BIAS', 'CCS.PR_SP_BIAS' -> 'CCS.PR_TARGET'
        PR_TARGET = np.interp(x = data_dict['CCS.MW_TARGET'] + data_dict['CCS.MW_SP_BIAS'],
                              xp = MW_PR_Fxy_x, fp = MW_PR_Fxy_y) + data_dict['CCS.PR_SP_BIAS']

        # 调用滤波器模块计算主汽压力设定当前值(速率后)  
        # 'CCS.MW_SP', 'CCS.PR_SP_BIAS' -> 'CCS.PR_SP'
        PR_SP0 = np.interp(x = MW_SP, xp = MW_PR_Fxy_x, fp = MW_PR_Fxy_y)  + data_dict['CCS.PR_SP_BIAS']
        PR_SP = self.sfilter1(Ut = PR_SP0)  

        data_dict.update({'CCS.MW_SP': MW_SP, 
                                   'CCS.PR_TARGET': PR_TARGET,
                                   'CCS.PR_SP': PR_SP})  

        # 调用协调被控对象仿真器  
        # 'CCS.BD','CCS.TD' -> 'CCS.MW', 'CCS.PR'
        y_Value_Dict1 = self.Simulator1(u_Value_Dict=data_dict)
        data_dict.update(y_Value_Dict1)

        # 配置写入OPCGate的字典
        opcgate_write_dict = {tag: value for tag, value in data_dict.items() 
                              if tag in ['CCS.MW_SP', 'CCS.PR_TARGET', 'CCS.PR_SP', 'CCS.MW', 'CCS.PR']}
        return opcgate_write_dict
 
ccs_process1 = CCSSim()
 
 
if __name__ == '__main__':
    from tjdcs import SimulinkOPCGateTask
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=9997, help="OPC RPC server port")
    parser.add_argument("--group", type=str, default="S1", help="OPC group tag")
    args = parser.parse_args()
    task = SimulinkOPCGateTask(
        port=args.port, group_tag=args.group, Simulator=ccs_process1)
    task.run()

