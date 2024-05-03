import numpy as np
from scipy import signal
from collections import ChainMap
from tjdcs import Simulink, MIMOSim
import copy
from collections import deque

# 配置被控对象模型
plant_model = {
    "CV1": {
        "MV1": {"mode": "tf_s", "num_s": [0, 4.05], "den_s": [20, 1], "iodelay_s": 5},
        "MV2": {"mode": "tf_s", "num_s": [0, 1.77], "den_s": [60, 1], "iodelay_s": 8},
        "DV1": {"mode": "tf_s", "num_s": [0, 3.3], "den_s": [20, 1], "iodelay_s": 4},
    },
    "CV2": {
        "MV1": {"mode": "tf_s", "num_s": [0, -1.39], "den_s": [50, 1], "iodelay_s": 7},
        "MV2": {"mode": "tf_s", "num_s": [0, 5.72], "den_s": [15, 1], "iodelay_s": 7},
        "DV1": {"mode": "tf_s", "num_s": [0, 1.2], "den_s": [25, 1], "iodelay_s": 4},
    },
    "CV3": {
        "MV1": {"mode": "tf_s", "num_s": [0, 1.2], "den_s": [40, 1], "iodelay_s": 5},
        "MV2": {"mode": "tf_s", "num_s": [0, 1.5], "den_s": [35, 1], "iodelay_s": 6},
        "DV1": {"mode": "tf_s", "num_s": [0, 2.0], "den_s": [25, 1], "iodelay_s": 5},
    },
}

plant_model_OPEN = eval(
    str(plant_model).replace("CV", "OPEN_CV").replace("MV", "OPEN_MV")
)


# 过程初值
process_ini_dict = {
    "MV1": 35,
    "MV2": 40,
    "DV1": 33,
    "CV1": 41,
    "CV2": 42,
    "CV3": 43,
    "SET_CV_STD": 0.0,
    "SET_DV_RANDOM": 0.0,
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
sim_ini_dict.update({f"CV{i}std_10min": 0.0 for i in range(1, 4)})
sim_ini_dict.update({f"OPEN_CV{i}std_10min": 0.0 for i in range(1, 4)})


# 定义仿真流程
class Sim(Simulink):
    def __init__(self) -> None:
        super().__init__(data=sim_ini_dict)
        self.plant = MIMOSim(plant_model, sim_ini_dict, Ts=1)
        self.plant_open = MIMOSim(plant_model_OPEN, sim_ini_dict, Ts=1)

        # 配置输出噪声序列
        self.N = 100000
        np.random.seed(1)
        v1 = signal.lfilter(
            [1, -0.5, 0.3], np.convolve([1, -0.95], [1, -0.99]), np.random.randn(self.N)
        )
        v2 = signal.lfilter(
            [1, -0.5, 0.3], np.convolve([1, -0.95], [1, -0.99]), np.random.randn(self.N)
        )
        v3 = signal.lfilter(
            [1, -0.5, 0.3], np.convolve([1, -0.95], [1, -0.99]), np.random.randn(self.N)
        )
        self.v = {f"CV{i}": eval(f"v{i}") for i in range(1, 4)}
        for value in self.v.values():
            value -= np.mean(value)
            value /= np.std(value)

        dv1 = signal.lfilter(
            [1, -0.5, 0.7], np.convolve([1, -0.97], [1, -0.97]), np.random.randn(self.N)
        )
        dv1 += 10 * signal.lfilter(
            [1, -0.5, 0.7], np.convolve([1, -0.15], [1, -0.19]), np.random.randn(self.N)
        )
        self.dv1 = 1.0 * dv1 / np.std(dv1) + sim_ini_dict["DV1"]

        self.cv_array_dict = {
            f"CV{i}array_10min": deque(maxlen=600) for i in range(1, 4)
        }
        self.open_cv_array_dict = {
            f"OPEN_CV{i}array_10min": deque(maxlen=600) for i in range(1, 4)
        }

    def task(self) -> None:
        data = self.get_data()
        count = self.get_task_count() % self.N

        # 获取CV的不可测干扰
        Vstd = max(0, data["SET_CV_STD"])
        v_dict = {f"CV{i}": Vstd * self.v[f"CV{i}"][count] for i in range(1, 4)}
        v_dict.update(
            {f"OPEN_CV{i}": Vstd * self.v[f"CV{i}"][count] for i in range(1, 4)}
        )

        # 获取DV
        if data["SET_DV_RANDOM"] > 0.5:
            data.update({"DV1": self.dv1[count]})

        # 计算CV
        cv_dict = self.plant.run(u_Value_Dict=data, v_Value_Dict=v_dict)
        data.update(cv_dict)
        open_cv_dict = self.plant_open.run(u_Value_Dict=data, v_Value_Dict=v_dict)
        data.update(open_cv_dict)

        # 计算标准差
        for i in range(1, 4):
            self.cv_array_dict[f"CV{i}array_10min"].append(data[f"CV{i}"])
            self.open_cv_array_dict[f"OPEN_CV{i}array_10min"].append(
                data[f"OPEN_CV{i}"]
            )
            data[f"CV{i}std_10min"] = np.std(self.cv_array_dict[f"CV{i}array_10min"])
            data[f"OPEN_CV{i}std_10min"] = np.std(
                self.open_cv_array_dict[f"OPEN_CV{i}array_10min"]
            )


# 实例化仿真对象
MIMOModel_2x3 = Sim()

if __name__ == "__main__":
    from tjdcs import SimulinkOPCGateTask
    task = SimulinkOPCGateTask(Simulator=MIMOModel_2x3, group_tag="T3")
    task.run()
