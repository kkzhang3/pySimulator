import os, sys

path2 = r"D:\TaijiControl\released_tj010\0.37.8"
sys.path.insert(0, path2)


import tj010
print(f'{tj010.__version__ = }')

import numpy as np
from matplotlib import pyplot as plt
from ccsmodel import CCSSim, ccs_plant


# sys.path.insert(0, r'D:\PythonFiles\tjdcs')  # 填写tjdcs目录
import tjdcs
print(f'{tjdcs.__version__ = }')
tjdcs.plt_model_stp(ccs_plant, Ts = 1, plotLen = 1500)

# sys.path.insert(0, r'D:\github1\tj010')  # 填写tj010目录
# import tj010
from tj010.tj007pyInterface import ControllerInterface, OutputVariable, InputVariable
import tj010.mpcplot as mpcplot
# print(f'{tj010.__version__ = }')

# 被控对象仿真器
Simulator = CCSSim()

# 定义控制器
mpc1 = ControllerInterface()

# 设置控制器模型
mpc1.SetupModelPy(ccs_plant)

# 设置仿真时长
N = 500

for k in range(0, N):

    input_dict = {}
    # 配置协调设定值
    if 0 < k < 200:
        input_dict.update({'CCS.MW_TARGET': 185,
                                        'CCS.MW_SP_BIAS': 0.0,
                                        'CCS.PR_SP_BIAS': 0.0})
    elif 200 < k < 600:
        input_dict.update({'CCS.MW_TARGET': 185,
                                        'CCS.MW_SP_BIAS': 0.0,
                                        'CCS.PR_SP_BIAS': 0.0})

    # 获取实时数据
    DataDict = Simulator.get_data()

    # 配置MPC
    mpc1.EconomicOptFlag = 0
    mpc1.DistPredSW = 0

    # 配置MPC中的CV参数
    CVx = OutputVariable()
    CVx.ControlStatus = 1
    CVx.ErrorWeight = 1.0
    CVx.IncrementWeight = 0
    CVx.ControlType = 1
    CVx.SetPoint = DataDict['CCS.MW_SP']
    CVx.DistAdaptiveSwt = 1
    CVx.Priority = 1
    # CVx.SetNp = 800
    # CVx.IntBlock = 2000
    CVx.FTSPType = 1
    CVx.FTSPRate = DataDict['CCS.MW_SLOPE']
    CVx.FTSPEnd = DataDict['CCS.MW_TARGET']
    CVx.FTSPBias = DataDict['CCS.MW_SP_BIAS']
    CVx.FTSPHzn = 800
    mpc1.ModifyOutputVariablePy('CCS.MW', CVx)

    CVx = OutputVariable()
    CVx.ControlStatus = 1
    CVx.ErrorWeight = 1.0
    CVx.IncrementWeight = 0
    CVx.ControlType = 1
    CVx.SetPoint = DataDict['CCS.PR_SP']
    CVx.DistAdaptiveSwt = 1
    CVx.Priority = 1
    CVx.FTSPType = 2
    CVx.FTSPRate = DataDict['CCS.MW_SLOPE']
    CVx.FTSPNum = DataDict['CCS.MW_SP']
    CVx.FTSPEnd = DataDict['CCS.MW_TARGET']
    CVx.FTSPTau = 1.0
    CVx.FTSPXY = '0,15#175,15#230,20#300,23.5#350,24.2#1000,24.2'
    CVx.FTSPBias = DataDict['CCS.PR_SP_BIAS']
    CVx.FTSPHzn = 800
    mpc1.ModifyOutputVariablePy('CCS.PR', CVx)

    # 配置MPC中的MV参数
    MVx = InputVariable()
    MVx.ControlStatus = 1
    MVx.IncrementWeight = 2.0
    MVx.HiLimit = 200
    MVx.LoLimit = 0
    MVx.IntBlock = 30
    mpc1.ModifyInputVariablePy('CCS.BD', MVx)

    # 更新MV参数
    MVx = InputVariable()
    MVx.ControlStatus = 1
    MVx.IncrementWeight = 5.0
    MVx.HiLimit = 100
    MVx.LoLimit = 0
    MVx.IntBlock = 50
    mpc1.ModifyInputVariablePy('CCS.TD', MVx)

    # 单步调用mpc得到MV动作
    MV_action_dict = mpc1.ControllerCalculationPy(DataDict)
    
    input_dict.update(MV_action_dict)
    
    # 更新仿真器数据，并调用单步计算(以及DV处理)
    Simulator.run(input_dict)
    # Simulator.one_sample_simulation()

# 仿真结果绘图
mpcplot.ShowCurrentParam(mpc1)
mpcplot.PlotControllerData(mpc1)

plt.show()


Simulator.plot_record_data_in_one_figure(taglist=['CCS.MW','CCS.MW_TARGET','CCS.MW_SP'])
Simulator.plot_record_data_in_one_figure(taglist=['CCS.PR','CCS.PR_TARGET','CCS.PR_SP'])
plt.show()
