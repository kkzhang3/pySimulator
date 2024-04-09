import sys, os
# sys.path.insert(0, r'D:\PythonFiles\tjdcs')  # 填写tjdcs目录
import tjdcs
print(f'{tjdcs.__version__ = }')
from tjdcs import Simulink, MIMOSim
# sys.path.insert(0, r'D:\TaijiControl_010 - 4.0.3.15\TaiJiMPC_010\bin_010')  # 填写tj010目录
sys.path.insert(0, r'D:\github1\tj010')  # 填写tj010目录
# sys.path.insert(0, r'C:\Users\KK\Desktop2\tj010-0.36.0')  # 填写tj010目录
# sys.path.insert(0, r'D:\PythonFiles\tj010_develop\MPC_released\tj010_released_v0.31.1')
import tj010
from tj010.tj007pyInterface import ControllerInterface, OutputVariable, InputVariable
import tj010.mpcplot as mpcplot
print(f'{tj010.__version__ = }')

import numpy as np
from matplotlib import gridspec
import matplotlib.pyplot as plt


FONT_SIZE = 13

from AGMill_model import AGMPower_Model, AGMSim

# 实例化MPC控制器
mpc1 = ControllerInterface()

mpc1.ControllerFlagInitialization(path = os.getcwd(), ControlInterval = 30)

# 设置MPC控制器模型(MPC执行Start Control操作)
mpc1.SetupModelPy(AGMPower_Model)
# mpc1.save_mpcmodel()

# 设置仿真时长
N = 10

for k in range(0, N):
    # 控制器总开关
    mpc1.EconomicOptFlag = 0
    mpc1.DistPredSW = 0

    # 更新CV参数: AGMPower
    CVx = OutputVariable()
    CVx.ControlStatus = 1
    CVx.ErrorWeight = 1000
    CVx.PRIWeight = 1000
    CVx.IncrementWeight = 200000
    CVx.ControlType = 1
    CVx.SetPoint = 23
    CVx.Priority = 1
    mpc1.ModifyOutputVariablePy('AGMPower', CVx)

    # 更新CV参数: AGMCurrent
    CVx = OutputVariable()
    CVx.ControlStatus = 1
    CVx.ErrorWeight = 1
    CVx.PRIWeight = 1
    CVx.IncrementWeight = 200
    CVx.ControlType = 1
    CVx.SetPoint = 3800
    CVx.Priority = 1
    mpc1.ModifyOutputVariablePy('AGMCurrent', CVx)

    # 更新MV: FreshFeedWeight参数
    MVx = InputVariable()
    MVx.ControlStatus = 1
    MVx.IncrementWeight = 1e-4
    MVx.PosSpeedLim = 1000
    MVx.NegSpeedLim = 1000
    MVx.HiLimit = 1800
    MVx.LoLimit = 1185
    # MVx.SetNm = 50
    mpc1.ModifyInputVariablePy('FreshFeedWeight', MVx)
    
    # 更新MV: AGMSpeed参数
    MVx = InputVariable()
    MVx.ControlStatus = 1
    MVx.IncrementWeight = 1000
    MVx.PosSpeedLim = 1000
    MVx.NegSpeedLim = 1000
    MVx.HiLimit = 9.7
    MVx.LoLimit = 9.2
    # MVx.SetNm = 50
    mpc1.ModifyInputVariablePy('AGMSpeed', MVx)
    
    # 更新DV参数: CRU1Weight
    DVx = InputVariable()
    DVx.MVType = 0
    DVx.ControlStatus = 1
    mpc1.ModifyInputVariablePy('CRU1Weight', DVx)
    
    # 更新DV参数: BypassWeight
    DVx = InputVariable()
    DVx.MVType = 0
    DVx.ControlStatus = 1
    mpc1.ModifyInputVariablePy('BypassWeight', DVx)

    # 更新DV参数: F80
    DVx = InputVariable()
    DVx.MVType = 0
    DVx.ControlStatus = 1
    mpc1.ModifyInputVariablePy('F80', DVx)

    # 获取实时数据，并调用单步MPC优化函数
    DataDict = AGMSim.get_data()
    MV_action_dict = mpc1.ControllerCalculationPy(DataDict)
    
    # 更新仿真器数据，并调用单步计算
    AGMSim.run(MV_action_dict)

    
# 仿真结果绘图
# mpc1._save_mpcobj()
mpcplot.ShowCurrentParam(mpc1)
mpcplot.PlotControllerData(mpc1)
mpcplot.PlotCurrentModelStp(mpc1)

plt.show()


# mpcplot.PlotCurrentStep(mpc1)
# plt.show()