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

plant_model = { 'CV1': {'MV1': {'mode': 'tf_s', 'num_s': [0, 3.5], 'den_s': [1000, 1], 'iodelay_s': 60, 'IntegralMark': 0}}}


# process_model_stp = tjdcs.get_model_stp(plant_model, length=1500, Ts=1)
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

from AGMill_model import AGM_Power_Model, AGMSim

# 实例化MPC控制器
mpc1 = ControllerInterface()

mpc1.ControllerFlagInitialization(path = os.getcwd(), ControlInterval = 30)

# 设置MPC控制器模型(MPC执行Start Control操作)
mpc1.SetupModelPy(AGM_Power_Model)
# mpc1.save_mpcmodel()

# 设置仿真时长
N = 500

for k in range(0, N):
    # 控制器总开关
    mpc1.EconomicOptFlag = 0
    mpc1.DistPredSW = 0

    # 更新CV参数
    CVx = OutputVariable()
    CVx.ControlStatus = 1
    CVx.ErrorWeight = 1
    CVx.IncrementWeight = 0.0
    CVx.ControlType = 1
    # CVx.SetPoint = 31 if k < 500 else 35
    CVx.SetPoint = 24
    
    # CVx.HiRange = 55
    # CVx.LoRange = 36
    # CVx.ErrWtRngFactor = 0.01
    # CVx.IncWtRngFactor = 0.01
    # CVx.DistAdaptiveSwt = 1
    CVx.Priority = 1
    # CVx.SetNp = 200
    mpc1.ModifyOutputVariablePy('AGMPower', CVx)

    # 更新MV参数
    MVx = InputVariable()
    MVx.ControlStatus = 1
    MVx.IncrementWeight = 0.001
    MVx.PosSpeedLim = 1000
    MVx.NegSpeedLim = 1000
    MVx.HiLimit = 1800
    MVx.LoLimit = 0
    # MVx.SetNm = 50
    mpc1.ModifyInputVariablePy('FreshFeedWeight', MVx)

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