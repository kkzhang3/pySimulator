# -*- coding: utf-8 -*-
import os
import sys
import importlib
import yaml
import subprocess, psutil, time
from typing import List, Tuple, Dict 
from PyQt5 import QtCore, QtGui, QtWidgets
from OPCGatePy.opc_calc_task import OPCCalcTask
from tjdcs import SimulinkOPCGateTask, Simulink

original_stdout = sys.stdout

def find_file_in_folder(directory, target_folder, target_file):
    found_files = []
    for dirpath, dirnames, filenames in os.walk(directory):
        if os.path.basename(dirpath) == target_folder:
            if target_file in filenames:
                found_files.append(os.path.join(dirpath, target_file))
    return found_files

class Stream(QtCore.QObject):
    newText = QtCore.pyqtSignal(str)
    def write(self, text):
        self.newText.emit(str(text))

class WorkerThread(QtCore.QThread):
    def __init__(self, parent, task):
        super().__init__(parent)
        self.task = task

    def run(self):
        print("task started.")
        self.task.run()
        print("task stopped.")

class LogTextEdit(QtWidgets.QTextEdit):
    def __init__(self, max_lines=300, *args, **kwargs):
        super(LogTextEdit, self).__init__(*args, **kwargs)
        self.max_lines = max_lines

    def append_log(self, text:str):
        lines = self.toPlainText().splitlines()
        while len(lines) >= self.max_lines:
            lines.pop(0)
        lines.append(text.rstrip('\n'))
        self.setPlainText('\n'.join(lines))
        self.moveCursor(self.textCursor().End)

def dynamic_import_object(directory, module_name, object_name):
    if directory not in sys.path:
        sys.path.insert(0, directory)
    module = importlib.import_module(module_name)
    importlib.reload(module)
    desired_object = getattr(module, object_name)
    return desired_object

class SelectItemDialog(QtWidgets.QDialog):
    def __init__(self, parent=None):
        super(SelectItemDialog, self).__init__(parent)
        self.resize(500, 400)
       
        self.label = QtWidgets.QLabel("Please select a process to start", self)
        self.tableWidget = QtWidgets.QTableWidget(self)
        self.tableWidget.setEditTriggers(QtWidgets.QTableWidget.NoEditTriggers)
        self.tableWidget.setWordWrap(True)
        self.tableWidget.setColumnCount(4)
        self.tableWidget.setHorizontalHeaderLabels(['Directory', 'Module', 'Object', 'Description'])
        self.tableWidget.setSelectionBehavior(QtWidgets.QTableWidget.SelectRows)

        self.load_process()
        self.tableWidget.setRowCount(len(self.process_list))

        for i, d in enumerate(self.process_list):
            self.tableWidget.setItem(i, 0, QtWidgets.QTableWidgetItem(d.get("import_dir","")))
            self.tableWidget.setItem(i, 1, QtWidgets.QTableWidgetItem(d.get("module_name","")))
            self.tableWidget.setItem(i, 2, QtWidgets.QTableWidgetItem(d.get("object_name","")))
            description_item = QtWidgets.QTableWidgetItem(d.get("description",""))
            description_item.setFlags(description_item.flags() | QtCore.Qt.TextWordWrap)
            self.tableWidget.setItem(i, 3, description_item)
            self.tableWidget.resizeRowToContents(i)

        self.tableWidget.resizeColumnsToContents()
        header = self.tableWidget.horizontalHeader()
        header.setSectionResizeMode(2, QtWidgets.QHeaderView.Stretch)

        self.buttonBox = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel, self)
        self.buttonBox.accepted.connect(self.accept)
        self.buttonBox.rejected.connect(self.reject)

        self.layout = QtWidgets.QVBoxLayout()
        self.layout.addWidget(self.label)
        self.layout.addWidget(self.tableWidget)
        self.layout.addWidget(self.buttonBox)
        self.setLayout(self.layout)

    def load_process(self):
        current_file_path = os.path.abspath(__file__)         
        current_directory = os.path.dirname(current_file_path) 
        parent_directory = os.path.dirname(current_directory)  
        directory = parent_directory
        target_folder = "process"
        target_file = "info.yaml"

        yaml_file_results = find_file_in_folder(directory, target_folder, target_file)
        self.process_list = []
        for yaml_file in yaml_file_results:
            with open(yaml_file, 'r') as file:
                yaml_data = yaml.safe_load(file)
            d = {
                'yaml_file': yaml_file,
                'import_dir': os.path.dirname(yaml_file),
                'module_name': yaml_data.get("file", "").rstrip('.py'),
                'object_name': yaml_data.get("object", ""),
                'default_group': yaml_data.get("defaultGroup", ""),
                'description': yaml_data.get("description", "")
            }
            self.process_list.append(d)

    def get_selected_item(self) -> dict:
        currentRow = self.tableWidget.currentRow()
        if currentRow != -1 and self.process_list:  # -1 means no row is selected
            return self.process_list[currentRow]
        else:
            return {}

class TaiJiSimulatorWidget(QtWidgets.QMainWindow):
    worker_thread = QtCore.QThread()

    def __init__(self, parent=None):
        super(TaiJiSimulatorWidget, self).__init__(parent)
        self.setWindowTitle("Tai-Ji Simulator")
        current_file_path = os.path.abspath(__file__)         
        current_directory = os.path.dirname(current_file_path) 
        self.setWindowIcon(QtGui.QIcon(os.path.join(current_directory, 'pet.svg')))
        self.resize(600, 800)

        # 添加菜单栏
        menu_bar = self.menuBar()
        file_menu = menu_bar.addMenu("File")

        # 添加动作到文件菜单
        self.load_action = QtWidgets.QAction("Load process", self)
        self.load_action.triggered.connect(self.show_dialog)
        file_menu.addAction(self.load_action)
        
        exit_action = QtWidgets.QAction("Exit", self)
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)

        tool_menu = menu_bar.addMenu("Tool")
        valid_opc_action = QtWidgets.QAction("Valid TaiJiOPCSim Connect...", self)
        valid_opc_action.triggered.connect(self.valid_opc_connect)
        tool_menu.addAction(valid_opc_action)

        # groupBox_1
        groupBox_1 = QtWidgets.QGroupBox(self)
        groupBox_1.setTitle(f"Process INFO")
        layout_B1 = QtWidgets.QVBoxLayout()
        groupBox_1.setLayout(layout_B1)

        self.lineEdit_moduleDir = QtWidgets.QLineEdit()
        self.lineEdit_moduleDir.setDisabled(True)
        self.lineEdit_moduleName = QtWidgets.QLineEdit()
        self.lineEdit_moduleName.setDisabled(True)
        self.lineEdit_groupName = QtWidgets.QLineEdit()
        self.lineEdit_objectName = QtWidgets.QLineEdit()
        self.lineEdit_objectName.setDisabled(True)
        self.lineEdit_processDetail = QtWidgets.QLineEdit()
        self.lineEdit_processDetail.setDisabled(True)

        layout_1 = QtWidgets.QHBoxLayout()
        layout_1.addWidget(QtWidgets.QLabel("Module Name"))
        layout_1.addWidget(self.lineEdit_moduleName)
        layout_1.addWidget(QtWidgets.QLabel("Object Name"))
        layout_1.addWidget(self.lineEdit_objectName)
        layout_1.addWidget(QtWidgets.QLabel("OPC Tag Group"))
        layout_1.addWidget(self.lineEdit_groupName)

        form_layout = QtWidgets.QFormLayout()
        form_layout.addRow(QtWidgets.QLabel("Detail"), self.lineEdit_processDetail)
        form_layout.addRow(QtWidgets.QLabel("Directory"), self.lineEdit_moduleDir)

        self.start_btn = QtWidgets.QPushButton(str("START"))
        self.stop_btn = QtWidgets.QPushButton(str("STOP"))
        self.start_btn.setDisabled(True)
        self.stop_btn.setDisabled(True)
        self.start_btn.clicked.connect(self.start_process_btn_clicked)
        self.stop_btn.clicked.connect(self.stop_process_btn_clicked)

        layout_3 = QtWidgets.QHBoxLayout()
        layout_3.addWidget(self.start_btn, stretch=1)
        layout_3.addWidget(self.stop_btn, stretch=1)

        layout_B1.addLayout(layout_1)
        layout_B1.addLayout(form_layout)
        layout_B1.addLayout(layout_3)

        # groupBox_3
        groupBox_3 = QtWidgets.QGroupBox(self)
        layout_B3 = QtWidgets.QVBoxLayout()
        groupBox_3.setTitle(f"Running LOG")
        groupBox_3.setLayout(layout_B3)

        self.log_text_editor = LogTextEdit()
        sys.stdout = Stream(newText=self.log_text_editor.append_log)
        layout_B3.addWidget(self.log_text_editor)

        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(groupBox_1)
        layout.addWidget(groupBox_3)

        central_widget = QtWidgets.QWidget(self)
        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)

    def show_dialog(self):
        dialog = SelectItemDialog(self)
        result = dialog.exec_()
        if result == QtWidgets.QDialog.Accepted:
            selected_process = dialog.get_selected_item()
            if selected_process:
                self.start_btn.setDisabled(False)
                self.lineEdit_moduleName.setText(selected_process["module_name"])
                self.lineEdit_objectName.setText(selected_process["object_name"])
                self.lineEdit_groupName.setText(selected_process["default_group"])
                self.lineEdit_processDetail.setText(selected_process["description"])
                self.lineEdit_moduleDir.setText(selected_process["import_dir"])
            else:
                self.start_btn.setDisabled(True)

    def start_process(self):
        directory_path = self.lineEdit_moduleDir.text()
        module_name = self.lineEdit_moduleName.text()
        object_name = self.lineEdit_objectName.text()
        group_tag = self.lineEdit_groupName.text()
        ojb = dynamic_import_object(directory_path, module_name, object_name)
        if isinstance(ojb, Simulink):
            task = SimulinkOPCGateTask('127.0.0.1', 9997, ojb, group_tag)
        elif isinstance(ojb, SimulinkOPCGateTask):
            task = ojb
        elif isinstance(ojb, OPCCalcTask):
            task = ojb
        else:
            print("Error")
            assert False
        self.worker_thread = WorkerThread(self, task)
        self.worker_thread.start()
        if self.worker_thread.isRunning:
            self.start_btn.setDisabled(True)
            self.load_action.setDisabled(True)
            self.lineEdit_groupName.setDisabled(True)
            self.stop_btn.setDisabled(False)

    def close_process(self):
        if self.worker_thread.isRunning():
            self.worker_thread.task.stop()
            self.worker_thread.wait()
            self.start_btn.setDisabled(False)
            self.stop_btn.setDisabled(True)

    def closeEvent(self, event):
        if not self.worker_thread.isRunning():
            sys.stdout = original_stdout
            event.accept()
            self.close_opc_sim()
            return
        reply = QtWidgets.QMessageBox.question(self,
                    'Message',
                    "Process is running. Are you sure you want to quit?", 
                    QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, 
                    QtWidgets.QMessageBox.No)
        if reply == QtWidgets.QMessageBox.Yes:
            self.close_process()
            self.close_opc_sim()
            sys.stdout = original_stdout
            event.accept()  
        else:
            event.ignore()  

    def start_process_btn_clicked(self):
        self.valid_opc_connect()
        self.start_process()

    def stop_process_btn_clicked(self):
        self.close_process()

    def valid_opc_connect(self):
        for proc in psutil.process_iter(['name']):
            if proc.info['name'] == 'taiji-opcsim-server.exe':
                print('TaiJiOPCSim is Launched')
                return
        filePath = '..\\bin\\taiji-opcsim-server.exe'
        try:
            self.sim_process = subprocess.Popen([filePath,'-s'], creationflags=subprocess.CREATE_NO_WINDOW)
        except FileNotFoundError:
            print('Not Found TaiJiOPCSim File:' + filePath)
            return
        except Exception as ex:
            print('Launch TaiJiOPCSim Failed ' + ex)
            return
        time.sleep(1)
        for proc in psutil.process_iter(['name']):
            if proc.info['name'] == 'taiji-opcsim-server.exe':
                print('Launch TaiJiOPCSim Successful')
                return
        self.sim_process = None
        print('Launch TaiJiOPCSim Failed')

    def close_opc_sim(self):
        if self.sim_process:
            self.sim_process.terminate()
            self.sim_process.wait()

