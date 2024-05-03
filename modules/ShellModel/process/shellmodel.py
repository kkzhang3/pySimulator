import numpy as np
from scipy import signal
from collections import deque

# import sys
# sys.path.insert(0, r'D:\PythonFiles\tjdcs')  # 填写tjdcs目录,tjdcs版本>=0.9.23
from tjdcs import Simulink, MIMOSim

############################################################################################################
# 1. 本文件定义了一个壳牌精馏塔模型（shell process）
# 2. 文件中的类：ShellSim 定义了模型的响应，可以被实例化后直接使用
# 3. 若直接运行此python文件，则会在TaiJiOPCGate中创建ShellModel仿真器（一般用于常规仿真）
#   3.1 TaiJiOPCGate中的'SET_CV_STD'与'SET_DV_RANDOM' 位号
#       可以控制ShellModel中的随机干扰
############################################################################################################

# 配置被控对象模型
plant_model = {
    "CV1": {
        "MV1": {"mode": "tf_s", "num_s": [0, 4.05], "den_s": [50, 1], "iodelay_s": 25},
        "MV2": {"mode": "tf_s", "num_s": [0, 1.77], "den_s": [60, 1], "iodelay_s": 27},
        "MV3": {"mode": "tf_s", "num_s": [0, 5.88], "den_s": [50, 1], "iodelay_s": 25},
        "DV1": {"mode": "tf_s", "num_s": [0, 1.20], "den_s": [45, 1], "iodelay_s": 25},
        "DV2": {"mode": "tf_s", "num_s": [0, 1.44], "den_s": [40, 1], "iodelay_s": 25},
    },
    "CV2": {
        "MV1": {"mode": "tf_s", "num_s": [0, 5.39], "den_s": [50, 1], "iodelay_s": 17},
        "MV2": {"mode": "tf_s", "num_s": [0, 5.72], "den_s": [60, 1], "iodelay_s": 17},
        "MV3": {"mode": "tf_s", "num_s": [0, 6.90], "den_s": [40, 1], "iodelay_s": 15},
        "DV1": {"mode": "tf_s", "num_s": [0, 1.52], "den_s": [25, 1], "iodelay_s": 15},
        "DV2": {"mode": "tf_s", "num_s": [0, 1.83], "den_s": [20, 1], "iodelay_s": 15},
    },
    "CV3": {
        "MV1": {"mode": "tf_s", "num_s": [0, 3.66], "den_s": [9, 1], "iodelay_s": 4},
        "MV2": {"mode": "tf_s", "num_s": [0, 1.65], "den_s": [30, 1], "iodelay_s": 4},
        "MV3": {"mode": "tf_s", "num_s": [0, 5.53], "den_s": [40, 1], "iodelay_s": 6},
        "DV1": {"mode": "tf_s", "num_s": [0, 1.16], "den_s": [11, 1], "iodelay_s": 4},
        "DV2": {"mode": "tf_s", "num_s": [0, 1.27], "den_s": [6, 1], "iodelay_s": 4},
    },
    "CV4": {
        "MV1": {"mode": "tf_s", "num_s": [0, 5.92], "den_s": [12, 1], "iodelay_s": 8},
        "MV2": {"mode": "tf_s", "num_s": [0, 2.54], "den_s": [27, 1], "iodelay_s": 9},
        "MV3": {"mode": "tf_s", "num_s": [0, 8.10], "den_s": [20, 1], "iodelay_s": 9},
        "DV1": {"mode": "tf_s", "num_s": [0, 1.73], "den_s": [5, 1], "iodelay_s": 8},
        "DV2": {"mode": "tf_s", "num_s": [0, 1.79], "den_s": [19, 1], "iodelay_s": 9},
    },
    "CV5": {
        "MV1": {"mode": "tf_s", "num_s": [0, 4.13], "den_s": [8, 1], "iodelay_s": 5},
        "MV2": {"mode": "tf_s", "num_s": [0, 2.38], "den_s": [19, 1], "iodelay_s": 7},
        "MV3": {"mode": "tf_s", "num_s": [0, 6.23], "den_s": [10, 1], "iodelay_s": 5},
        "DV1": {"mode": "tf_s", "num_s": [0, 1.31], "den_s": [2, 1], "iodelay_s": 5},
        "DV2": {"mode": "tf_s", "num_s": [0, 1.26], "den_s": [22, 1], "iodelay_s": 5},
    },
    "CV6": {
        "MV1": {"mode": "tf_s", "num_s": [0, 4.06], "den_s": [13, 1], "iodelay_s": 7},
        "MV2": {"mode": "tf_s", "num_s": [0, 4.18], "den_s": [33, 1], "iodelay_s": 7},
        "MV3": {"mode": "tf_s", "num_s": [0, 6.53], "den_s": [9, 1], "iodelay_s": 5},
        "DV1": {"mode": "tf_s", "num_s": [0, 1.19], "den_s": [19, 1], "iodelay_s": 5},
        "DV2": {"mode": "tf_s", "num_s": [0, 1.17], "den_s": [24, 1], "iodelay_s": 5},
    },
    "CV7": {
        "MV1": {"mode": "tf_s", "num_s": [0, 4.38], "den_s": [33, 1], "iodelay_s": 15},
        "MV2": {"mode": "tf_s", "num_s": [0, 4.42], "den_s": [44, 1], "iodelay_s": 15},
        "MV3": {"mode": "tf_s", "num_s": [0, 7.20], "den_s": [19, 1], "iodelay_s": 15},
        "DV1": {"mode": "tf_s", "num_s": [0, 1.14], "den_s": [27, 1], "iodelay_s": 16},
        "DV2": {"mode": "tf_s", "num_s": [0, 1.26], "den_s": [32, 1], "iodelay_s": 17},
    },
}

plant_model_OPEN = eval(
    str(plant_model).replace("CV", "OPEN_CV").replace("MV", "OPEN_MV")
)

# 过程初值
process_ini_dict = {
    "MV1": 15,
    "MV2": 20,
    "MV3": 35,
    "DV1": 10,
    "DV2": 10,
    "CV1": 31,
    "CV2": 32,
    "CV3": 33,
    "CV4": 34,
    "CV5": 35,
    "CV6": 36,
    "CV7": 37,
    "SET_CV_STD": 1,
    "SET_DV_RANDOM": 1,
}

OPEN_ini_dict = {
    k.replace("MV", "OPEN_MV"): v
    for k, v in process_ini_dict.items()
    if k.startswith("MV")
}

OPEN_ini_dict.update(
    {
        k.replace("CV", "OPEN_CV"): v
        for k, v in process_ini_dict.items()
        if k.startswith("CV")
    }
)


sim_ini_dict = {}
sim_ini_dict.update(process_ini_dict)
sim_ini_dict.update(OPEN_ini_dict)
sim_ini_dict.update({f"CV{i}std_10min": 0.0 for i in range(1, 8)})
sim_ini_dict.update({f"OPEN_CV{i}std_10min": 0.0 for i in range(1, 8)})


# 定义仿真流程
class ShellSim(Simulink):
    def __init__(self) -> None:
        super().__init__(data=sim_ini_dict)
        self.plant = MIMOSim(plant_model, sim_ini_dict, Ts=1)
        self.plant_open = MIMOSim(plant_model_OPEN, sim_ini_dict, Ts=1)

        # 配置CV干扰
        self.N = 100000
        np.random.seed(1)

        v1 = signal.lfilter(
            [1, -0.5, 0.7], np.convolve([1, -0.95], [1, -0.99]), np.random.randn(self.N)
        )
        v2 = signal.lfilter(
            [1, -0.5, 0.7], np.convolve([1, -0.95], [1, -0.99]), np.random.randn(self.N)
        )
        v3 = signal.lfilter(
            [1, -0.5, 0.7], np.convolve([1, -0.95], [1, -0.98]), np.random.randn(self.N)
        )
        v4 = signal.lfilter(
            [1, -0.5, 0.7], np.convolve([1, -0.95], [1, -0.98]), np.random.randn(self.N)
        )
        v5 = signal.lfilter(
            [1, -0.5, 0.7], np.convolve([1, -0.95], [1, -0.97]), np.random.randn(self.N)
        )
        v6 = signal.lfilter(
            [1, -0.5, 0.7], np.convolve([1, -0.95], [1, -0.98]), np.random.randn(self.N)
        )
        v7 = signal.lfilter(
            [1, -0.5, 0.7], np.convolve([1, -0.95], [1, -0.98]), np.random.randn(self.N)
        )
        self.v = {f"CV{i}": eval(f"v{i}") for i in range(1, 8)}
        for value in self.v.values():
            value -= np.mean(value)
            value /= np.std(value)

        # 配置DV
        dv1 = signal.lfilter(
            [1, 0.7], np.convolve([1, -0.95], [1, -0.95]), np.random.randn(self.N)
        )
        dv1 += 5.5 * signal.lfilter(
            [1, 0.7], np.convolve([1, -0.15], [1, -0.19]), np.random.randn(self.N)
        )
        dv1 -= np.mean(dv1)
        dv1 = 1.3 * dv1 / np.std(dv1) + sim_ini_dict["DV1"]
        dv2 = signal.lfilter(
            [1, 0.7], np.convolve([1, -0.95], [1, -0.95]), np.random.randn(self.N)
        )
        dv2 += 9.5 * signal.lfilter(
            [1, 0.7], np.convolve([1, -0.15], [1, -0.19]), np.random.randn(self.N)
        )
        dv2 -= np.mean(dv2)
        dv2 = 0.9 * dv2 / np.std(dv2) + sim_ini_dict["DV2"]
        self.dv = {f"DV{i}": eval(f"dv{i}") for i in range(1, 3)}

        self.cv_array_dict = {
            f"CV{i}array_10min": deque(maxlen=600) for i in range(1, 8)
        }
        self.open_cv_array_dict = {
            f"OPEN_CV{i}array_10min": deque(maxlen=600) for i in range(1, 8)
        }

    def task(self) -> None:
        data = self.get_data()
        count = self.get_task_count() % self.N

        # 获取CV的不可测干扰
        Vstd = max(0, data["SET_CV_STD"])
        v_dict = {f"CV{i}": Vstd * self.v[f"CV{i}"][count] for i in range(1, 8)}
        v_dict.update(
            {f"OPEN_CV{i}": Vstd * self.v[f"CV{i}"][count] for i in range(1, 8)}
        )

        # 获取DV
        if data["SET_DV_RANDOM"] > 0.5:
            dv_dict = {f"DV{i}": self.dv[f"DV{i}"][count] for i in range(1, 3)}
            data.update(dv_dict)

        # 计算CV
        cv_dict = self.plant.run(u_Value_Dict=data, v_Value_Dict=v_dict)
        data.update(cv_dict)
        open_cv_dict = self.plant_open.run(u_Value_Dict=data, v_Value_Dict=v_dict)
        data.update(open_cv_dict)

        # 计算标准差
        for i in range(1, 8):
            self.cv_array_dict[f"CV{i}array_10min"].append(data[f"CV{i}"])
            self.open_cv_array_dict[f"OPEN_CV{i}array_10min"].append(
                data[f"OPEN_CV{i}"]
            )
            data[f"CV{i}std_10min"] = np.std(self.cv_array_dict[f"CV{i}array_10min"])
            data[f"OPEN_CV{i}std_10min"] = np.std(
                self.open_cv_array_dict[f"OPEN_CV{i}array_10min"]
            )


ShellModel = ShellSim()

if __name__ == "__main__":
    from tjdcs import SimulinkOPCGateTask

    task = SimulinkOPCGateTask(Simulator=ShellModel, group_tag="S1")
    task.run()
