import os, sys
from PyQt5 import QtCore, QtGui, QtWidgets
from QtGUI import TaiJiSimulatorWidget

if __name__ == "__main__":
    QtWidgets.QApplication.setAttribute(QtCore.Qt.HighDpiScaleFactorRoundingPolicy.PassThrough)
    current_file_path = os.path.abspath(__file__)         # 获取当前文件的绝对路径
    current_directory = os.path.dirname(current_file_path) # 获取当前文件的目录
    app = QtWidgets.QApplication(sys.argv)
    app.setWindowIcon(QtGui.QIcon(os.path.join(current_directory, 'pet.svg')))
    win = TaiJiSimulatorWidget()
    win.show()
    sys.exit(app.exec())