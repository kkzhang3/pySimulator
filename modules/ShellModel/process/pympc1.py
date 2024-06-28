import sys, os
# sys.path.insert(0, r'D:\PythonFiles\tjdcs')  # 填写tjdcs目录
import tjdcs
print(f'{tjdcs.__version__ = }')
from tjdcs import Simulink, MIMOSim
# sys.path.insert(0, r'D:\TaijiControl_010 - 4.0.3.15\TaiJiMPC_010\bin_010')  # 填写tj010目录
# sys.path.insert(0, r'D:\github1\tj010')  # 填写tj010目录
# sys.path.insert(0, r'C:\Users\KK\Desktop2\tj010-0.36.0')  # 填写tj010目录
# sys.path.insert(0, r'D:\PythonFiles\tj010_develop\MPC_released\tj010_released_v0.31.1')

# sys.path.insert(0, r'D:\TaijiControl\PyTaiji')
sys.path.insert(0, r'D:\github2\tj010')
import tj010
print(f'{tj010.__version__ = }')

from tj010.tj007pyInterface import ControllerInterface, OutputVariable, InputVariable
import tj010.mpcplot as mpcplot

import numpy as np
from matplotlib import gridspec
import matplotlib.pyplot as plt


FONT_SIZE = 13


from shellmodel import ShellModel, plant_model

# tjdcs.plt_model_stp(plant_model, Ts=1, plotLen=300)
# plt.show()
# mpc_model_stp = tjdcs.get_model_stp(mpc_model, length=200, Ts=1)
# model_stp = mpc_model_stp['CV1']['MV1']
# model_ipr = np.diff(mpc_model_stp['CV1']['MV1'], prepend=0).tolist()

# subfigNum = 3
# plt.figure(figsize=(5, 5*subfigNum), facecolor = 'w', tight_layout = True)
# gs = gridspec.GridSpec(subfigNum, 1)
# ax = plt.subplot(gs[0])
# ax.plot(process_model_stp['CV1']['MV1'], '-', label=f"true_process_stp")
# ax.legend(loc='best', fontsize=FONT_SIZE)
# plt.yticks(fontsize=FONT_SIZE)
# plt.xticks(fontsize=FONT_SIZE)
# ax.grid(True)

# ax = plt.subplot(gs[1])
# ax.plot(model_stp, '-', label=f"mpc_model_stp")
# ax.legend(loc='best', fontsize=FONT_SIZE)
# plt.yticks(fontsize=FONT_SIZE)
# plt.xticks(fontsize=FONT_SIZE)
# ax.grid(True)

# ax = plt.subplot(gs[2])
# ax.plot(model_ipr, '-', label=f"mpc_model_ipr")
# ax.legend(loc='best', fontsize=FONT_SIZE)
# plt.yticks(fontsize=FONT_SIZE)
# plt.xticks(fontsize=FONT_SIZE)
# ax.grid(True)
# plt.show()

# mpc_model2 = { 'CV1': {'MV1': {'mode': 'stp', 'stp': model_ipr, 'IntegralMark': 1}}}


# 实例化MPC控制器
mpc1 = ControllerInterface()

mpc1.ControllerFlagInitialization(path = os.getcwd(), ControlInterval = 1)

# 设置MPC控制器模型(MPC执行Start Control操作)
mpc1.SetupModelPy(plant_model)
# mpc1.save_mpcmodel()

# 设置仿真时长
N = 50

for k in range(0, N):
    # 控制器总开关
    mpc1.EconomicOptFlag = 1
    mpc1.DistPredSW = 1

    # 更新CV1的参数
    CVx = OutputVariable()
    CVx.ControlStatus = 1
    CVx.Priority = 1
    CVx.PRIWeight = 0.5
    CVx.ErrorWeight = 1.0
    CVx.ControlType = 1
    CVx.IncrementWeight = 100.0
    CVx.HiRange = 43
    CVx.SetPoint = 40
    CVx.LoRange = 15
    CVx.DistAdaptiveSwt = 1
    mpc1.ModifyOutputVariablePy('CV1', CVx)

    # 更新CV2的参数
    CVx = OutputVariable()
    CVx.ControlStatus = 1
    CVx.Priority = 1
    CVx.PRIWeight = 0.5
    CVx.ErrorWeight = 1.0
    CVx.IncrementWeight = 100.0
    CVx.ControlType = 3
    CVx.HiRange = 33
    CVx.SetPoint = 30
    CVx.LoRange = 0
    CVx.DistAdaptiveSwt = 1
    # CVx.ErrWtRngFactor = 0.02
    # CVx.IncWtRngFactor = 0.02
    mpc1.ModifyOutputVariablePy('CV2', CVx)

    # 更新CV3的参数
    CVx = OutputVariable()
    CVx.ControlStatus = 1
    CVx.Priority = 1
    CVx.PRIWeight = 0.5
    CVx.ErrorWeight = 1.0
    CVx.IncrementWeight = 100.0
    CVx.ControlType = 3
    CVx.HiRange = 50
    CVx.SetPoint = 35
    CVx.LoRange = 15
    CVx.DistAdaptiveSwt = 1
    # CVx.ErrWtRngFactor = 0.02
    # CVx.IncWtRngFactor = 0.02
    mpc1.ModifyOutputVariablePy('CV3', CVx)

    # 更新CV4的参数
    CVx = OutputVariable()
    CVx.ControlStatus = 1
    CVx.Priority = 1
    CVx.PRIWeight = 0.5
    CVx.ErrorWeight = 1.0
    CVx.IncrementWeight = 100.0
    CVx.ControlType = 1
    CVx.HiRange = 43
    CVx.SetPoint = 35
    CVx.LoRange = 15
    CVx.DistAdaptiveSwt = 0
    mpc1.ModifyOutputVariablePy('CV4', CVx)

    # 更新CV5的参数
    CVx = OutputVariable()
    CVx.ControlStatus = 1
    CVx.Priority = 5
    CVx.PRIWeight = 0.5
    CVx.ErrorWeight = 1.0
    CVx.IncrementWeight = 100.0
    CVx.ControlType = 1
    CVx.HiRange = 43
    CVx.SetPoint = 35
    CVx.LoRange = 15
    CVx.DistAdaptiveSwt = 1
    mpc1.ModifyOutputVariablePy('CV5', CVx)

    # 更新CV6的参数
    CVx = OutputVariable()
    CVx.ControlStatus = 1
    CVx.Priority = 6
    CVx.PRIWeight = 0.5
    CVx.ErrorWeight = 1.0
    CVx.IncrementWeight = 100.0
    CVx.ControlType = 1
    CVx.HiRange = 43
    CVx.SetPoint = 35
    CVx.LoRange = 15
    CVx.DistAdaptiveSwt = 1
    mpc1.ModifyOutputVariablePy('CV6', CVx)

    # 更新CV7的参数
    CVx = OutputVariable()
    CVx.ControlStatus = 1
    CVx.Priority = 7
    CVx.PRIWeight = 0.5
    CVx.ErrorWeight = 1.0
    CVx.IncrementWeight = 100.0
    CVx.ControlType = 1
    CVx.HiRange = 43
    CVx.SetPoint = 35
    CVx.LoRange = 15
    CVx.DistAdaptiveSwt = 1
    mpc1.ModifyOutputVariablePy('CV7', CVx)

    # 更新MV1参数
    MVx = InputVariable()
    MVx.ControlStatus = 1
    MVx.IncrementWeight = 1.0
    MVx.IncrementConstraint = 6.0
    MVx.HiLimit = 50
    MVx.LoLimit = -50
    # MVx.intBlock = 30
    # MVx.NegSpeedLim = 0
    mpc1.ModifyInputVariablePy('MV1', MVx)

    # 更新MV2参数
    MVx = InputVariable()
    MVx.ControlStatus = 1
    MVx.IncrementWeight = 1.0
    MVx.IncrementConstraint = 4.5
    MVx.HiLimit = 100
    MVx.LoLimit = -50
    # MVx.intBlock = 20
    mpc1.ModifyInputVariablePy('MV2', MVx)

    # 更新MV3参数
    MVx = InputVariable()
    MVx.ControlStatus = 1
    MVx.IncrementWeight = 1.0
    MVx.IncrementConstraint = 4.0
    MVx.HiLimit = 60
    MVx.LoLimit = 0
    # MVx.intBlock = 15
    mpc1.ModifyInputVariablePy('MV3', MVx)

    # 更新DV1参数
    DVx = InputVariable()
    DVx.ControlStatus = 1
    DVx.MVType = 0  
    mpc1.ModifyInputVariablePy('DV1', DVx)

    # 更新DV2参数
    DVx = InputVariable()
    DVx.ControlStatus = 1
    DVx.MVType = 0  
    mpc1.ModifyInputVariablePy('DV2', DVx)


    # 获取实时数据，并调用单步MPC优化函数
    DataDict = ShellModel.get_data()
    MV_action_dict = mpc1.ControllerCalculationPy(DataDict)
    
    # 更新仿真器数据，并调用单步计算
    ShellModel.run(MV_action_dict)

    
# 仿真结果绘图
# mpc1._save_mpcobj()
mpcplot.ShowCurrentParam(mpc1)
mpcplot.PlotControllerData(mpc1)
mpcplot.PlotCurrentModelStp(mpc1)

plt.show()


# mpcplot.PlotCurrentStep(mpc1)
# plt.show()