import sys
# sys.path.append('C:\\Huawei Drive\\PythonFiles2\\tj010_develop')
sys.path.insert(0, r'D:\github1\tj010')
import tj010
from tj010.tj007pyInterface import ControllerInterface, OutputVariable, InputVariable
import tj010.mpcplot as mpcplot
import matplotlib.pyplot as plt

# 加载系统模型文件
import os, sys
from sim import plant_model_APC as model_dict
from sim import TrainingSISO as Simulator1


# 打印tj010版本信息
print(tj010.__version__)



# 定义控制器
mpc1 = ControllerInterface()

# 设置控制器采样时间，单位：秒
mpc1.ControlInterval = 1  

# 设置控制器模型
mpc1.SetupModelPy(model_dict)
# mpc1._export_mpcmodel()

# 调用StartControl
mpc1.ProcessVariableInitializationPy()

# 设置仿真时长
N = 300

for k in range(0, N):

    # if k == 151:
    #     print(k)

    # if k == 15:
    #     model2 = { 'S1.CV1': {'S1.MV1': {'mode': 'tf_z', 'num_z': [0, 1.0, 0.5], 'den_z': [1.0, -1.5, 0.7], 'iodelay_z': 0}}}
    #     mpc1.SetupModelPy(model2)


    mpc1.EconomicOptFlag = 1
    mpc1.DistPredSW = 0

    # 更新CV参数
    CVx = OutputVariable()
    CVx.ControlStatus = 1
    CVx.ErrorWeight = 1.0
    CVx.IncrementWeight = 100.0
    CVx.ControlType = 2
    # CVx.SetPoint = 40
    CVx.HiRange = 55
    CVx.LoRange = 32
    # CVx.ErrWtRngFactor = 0.01
    # CVx.IncWtRngFactor = 0.01
    CVx.DistAdaptiveSwt = 0
    CVx.Priority = 1
    
    CVx.IRVExpect = 41.5
    # CVx.QuadraticWeight = 2.0
    CVx.LinearWeight = -1 if k >150 else 0
    # CVx.SetNp  = 2000
    # CVx.IntBlock = 2000
    mpc1.ModifyOutputVariablePy('APC.CV1', CVx)

    # 更新MV参数
    MVx = InputVariable()
    MVx.ControlStatus = 1
    MVx.IncrementWeight = 1.0
    MVx.HiLimit = 100
    MVx.LoLimit = 0
    # MVx.LinearWeight = -1
    # MVx.PosSpeedLim = 0.0
    # MVx.NegSpeedLim = 0.0
    # MVx.SetNm = 2000
    # MVx.IntBlock = 30
    mpc1.ModifyInputVariablePy('APC.MV1', MVx)

    # 获取实时数据
    DataDict = Simulator1.get_data()

    # 将新数据按MV/CV标签输入MPC
    # cv_tag_list, mv_tag_list = mpc1.get_tag_list()
    # MVCurrentDataList = [DataDict[mvtag] for mvtag in mv_tag_list]
    # CVCurrentDataList = [DataDict[cvtag] for cvtag in cv_tag_list]
    # tmp = mpc1.PlantRecursion(MVCurrentDataList,CVCurrentDataList)
    # # print(tmp)

    # 调用单步MPC优化函数
    MV_action_dict = mpc1.ControllerCalculationPy(DataDict)
    # 更新仿真器数据，并调用单步计算
    new_data = {}
    new_data.update(MV_action_dict)
    # new_data.update({'S1.__Random_CV_ONOFF': })
    Simulator1.run(new_data)

    


# 仿真结果绘图
# mpc1._export_mpcdata()
print(mpc1.mpcobj.info)
# Simulator1.export_csvdata(['S1.MV1','S1.CV1'], filename = f'R{R}.csv')
# mpcplot.ShowSimPlot(mpc1.mpcobj)
mpcplot.PlotControllerData(mpc1)
mpcplot.PlotCurrentModelStp(mpc1)


# 利用仿真器绘图
# Simulator1.plot_record_data(taglist=['S1.MV1','S1.CV1'])
plt.show()


