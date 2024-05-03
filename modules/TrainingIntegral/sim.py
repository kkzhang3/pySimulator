from tjdcs import MIMOSim, Simulink, PID
import numpy as np
from scipy import signal
from collections import deque


# 被控对象传递函数
plant_model = {
    "CV1": {
        "MV1": {
            "mode": "tf_s",
            "num_s": [0, 0.08],
            "den_s": [30, 1],
            "iodelay_s": 17,
            "IntegralMark": 1,
        },
        "DV1": {
            "mode": "tf_s",
            "num_s": [0, 0.06],
            "den_s": [20, 1],
            "iodelay_s": 4,
            "IntegralMark": 1,
        },
    }
}

plant_model_OPEN = eval(
    str(plant_model).replace("CV", "OPEN_CV").replace("MV", "OPEN_MV")
)

plant_model_PID = eval(str(plant_model).replace("CV", "PID_CV").replace("MV", "PID_MV"))


# 定义仿真初值（必须包含所有位号）
process_ini_dict = {
    "MV1": 29,
    "CV1": 51,
    "DV1": 33,
    "SP1": 51,
    "OP_HI": 100.0,
    "OP_LO": 0.0,
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

PID_ini_dict = {
    k.replace("MV", "PID_MV"): v
    for k, v in process_ini_dict.items()
    if k.startswith("MV")
}

PID_ini_dict.update(
    {
        k.replace("CV", "PID_CV"): v
        for k, v in process_ini_dict.items()
        if k.startswith("CV")
    }
)

PID_ini_dict.update(
    {
        "PID_KP": 0.2,
        "PID_TI": 150,
        "PID_TD": 20.0,
        "PID_N": 100.0,
    }
)

sim_ini_dict = {}
sim_ini_dict.update(process_ini_dict)
sim_ini_dict.update(OPEN_ini_dict)
sim_ini_dict.update(PID_ini_dict)

sim_ini_dict.update(
    {"CV1std_10min": 0.0, "OPEN_CV1std_10min": 0.0, "PID_CV1std_10min": 0.0}
)


class AntiWindupLimter:
    def __init__(self):
        self._u0 = None
        self._y0 = None

    def __call__(self, u: float, HI=100, LO=0) -> float:
        if self._u0 is None:
            self._u0 = u

        if self._y0 is None:
            self._y0 = min(max(u, LO), HI)

        delta_u = u - self._u0
        y = min(max(self._y0 + delta_u, LO), HI)
        self._u0 = u
        self._y0 = y
        return y

    def run(self, u, HI=100, LO=0) -> float:
        return self.__call__(u, HI, LO)


# 定义仿真流程
class SISOSim(Simulink):
    def __init__(self) -> None:
        super().__init__(data=sim_ini_dict)
        self.plant_open = MIMOSim(
            plant_model_OPEN, sim_ini_dict, Ts=self.get_sampling_time()
        )
        self.plant = MIMOSim(plant_model, sim_ini_dict, Ts=self.get_sampling_time())
        self.plant_pid = MIMOSim(
            plant_model_PID, sim_ini_dict, Ts=self.get_sampling_time()
        )
        self.limiter1 = AntiWindupLimter()
        self.limiter2 = AntiWindupLimter()
        self.limiter3 = AntiWindupLimter()
        self.pid = PID(TS=self.get_sampling_time())

        # 配置输出噪声序列
        self.N = 100000
        np.random.seed(2)
        v1 = signal.lfilter(
            [1, -0.5, 0.3], np.convolve([1, -0.99], [1, -0.99]), np.random.randn(self.N)
        )
        v1 -= np.mean(v1)
        self.v1 = 1.0 * v1 / np.std(v1)

        dv1 = signal.lfilter(
            [1, 0.7, -0.1], np.convolve([1, -0.95], [1, -0.92]), np.random.randn(self.N)
        )
        dv1 += 10 * signal.lfilter(
            [1, 0.7, -0.1], np.convolve([1, -0.15], [1, -0.19]), np.random.randn(self.N)
        )
        dv1 -= np.mean(dv1)
        self.dv1 = 1.0 * dv1 / np.std(dv1) + sim_ini_dict["DV1"]

        self.OPEN_CV1std_10min = deque(maxlen=600)
        self.CV1std_10min = deque(maxlen=600)
        self.PID_CV1std_10min = deque(maxlen=600)

    def task(self):
        data = self.get_data()
        count = self.get_task_count() % self.N

        # 获取CV的不可测干扰
        Vstd = max(0, data["SET_CV_STD"])
        v_dict = {
            "CV1": Vstd * self.v1[count],
            "OPEN_CV1": Vstd * self.v1[count],
            "PID_CV1": Vstd * self.v1[count],
        }

        # 获取DV
        if data["SET_DV_RANDOM"] > 0.5:
            data.update({"DV1": self.dv1[count]})

        # PID控制器
        self.pid.setPIDParam(
            KP=data["PID_KP"], TI=data["PID_TI"], TD=data["PID_TD"], N=data["PID_N"]
        )
        self.pid.setOPHighLowLimit(OP_HI=data["OP_HI"], OP_LO=data["OP_LO"])
        data["PID_MV1"] = self.pid.run(
            MODE=1, SP=data["SP1"], PV=data["PID_CV1"], TV=data["PID_MV1"]
        )

        # 计算CV
        open_loop_out = self.plant_open.run(u_Value_Dict=data, v_Value_Dict=v_dict)
        open_cv1 = self.limiter1.run(open_loop_out["OPEN_CV1"], 100, 0)
        data.update({"OPEN_CV1": open_cv1})

        pid_loop_out = self.plant_pid.run(u_Value_Dict=data, v_Value_Dict=v_dict)
        pid_cv1 = self.limiter2.run(pid_loop_out["PID_CV1"], 100, 0)
        data.update({"PID_CV1": pid_cv1})

        apc_loop_out = self.plant.run(u_Value_Dict=data, v_Value_Dict=v_dict)
        apc_cv1 = self.limiter3.run(apc_loop_out["CV1"], 100, 0)
        data.update({"CV1": apc_cv1})

        # 计算标准差
        self.CV1std_10min.append(data["CV1"])
        data["CV1std_10min"] = np.std(self.CV1std_10min)
        self.OPEN_CV1std_10min.append(data["OPEN_CV1"])
        data["OPEN_CV1std_10min"] = np.std(self.OPEN_CV1std_10min)
        self.PID_CV1std_10min.append(data["PID_CV1"])
        data["PID_CV1std_10min"] = np.std(self.PID_CV1std_10min)


TrainingSISO = SISOSim()

if __name__ == "__main__":
    from tjdcs import SimulinkOPCGateTask

    task = SimulinkOPCGateTask(Simulator=TrainingSISO, group_tag="T2")
    task.run()
