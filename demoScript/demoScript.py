# 示例脚本：使用OPCGatePy库，实现一个简单的计算任务。
from OPCGatePy.opc_calc_task import OPCCalcTask   
import time,os,sys,inspect    
import numpy as np    
import tjdcs

class Task_SimpleCalc(OPCCalcTask):
    def __init__(self, ip, port):
        super().__init__(ip, port)
        # 初始化时执行
        self.set_sampling_time(1)  # 设置脚本的执行周期为1秒
        print(f"{tjdcs.__version__ = }")    # 打印tjdcs的版本号
        print("Hello World!")    # 打印Hello World!
        
    def done(self):
        # 每个采样时刻执行
        val1 = self.read_value('Demo.Random1')  
        val1_calc = val1 + 1000
        self.write({'Demo.Tag1': val1_calc})  
        
        val2, val3 = self.read_value(['Demo.Random2', 'Demo.Random3'])
        self.write({'Demo.Tag2': val2,
                    'Demo.Tag3': val3,
                    })  
        
        # 打印信息到控制台   
        print(f"{time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())} val1={val1}, val2={val2}, val2={val3}") 
        


task = Task_SimpleCalc('127.0.0.1',9999)
task.run()