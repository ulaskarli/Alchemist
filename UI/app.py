# importing required libraries
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtPrintSupport import *
from UI.rviz_widget.myviz import MyViz
import os, time
import sys
import rospy
from std_msgs.msg import String 
from UI.codeEditor.higlight import PythonHighlighter
from PyQt5.QtWidgets import QWidget
from threading import Thread
os.environ["QT_API"] = "pyqt5"
from pyqtconsole.console import PythonConsole
import subprocess

def run_gpt_code():
	print("Running GPT Code")

	python_version = sys.version[0]
	if python_version == "3":
		result = subprocess.run(["python","./LLM/gpt_code.py"], capture_output=True, text=True, check=False)

		print(result.stdout)


class startUpWindow(QMainWindow):
	def __init__(self):
		super().__init__()
		layout = QVBoxLayout()
		self.combobox1 = QComboBox()
		self.combobox1.addItems(['Panda', 'UR5', 'TIAGo'])
		label1 = QLabel("Select Robot Platform")
		self.combobox2 = QComboBox()
		self.combobox2.addItems(['Low', 'Medium', 'High'])
		label2 = QLabel("Select Abstraction Level")
		layout.addWidget(label1)
		layout.addWidget(self.combobox1)
		layout.addWidget(label2)
		layout.addWidget(self.combobox2)
		button = QPushButton("Start")
		button.clicked.connect(self.showMainWindow)
		layout.addWidget(button)
		container = QWidget()

		# setting layout to the container
		container.setLayout(layout)

		# Center Screen
		qtRectangle = self.frameGeometry()
		centerPoint = QDesktopWidget().availableGeometry().center()
		qtRectangle.moveCenter(centerPoint)
		self.move(qtRectangle.topLeft())

		# making container as central widget
		self.setCentralWidget(container)
		
	def showMainWindow(self):
		global window, robot, abstractionLevel
		robot = self.combobox1.currentText()
		abstractionLevel = self.combobox2.currentText()
		rospy.loginfo("Selected Robot: "+robot+" and abstraction level: "+abstractionLevel)
		window = MainWindow()
		window.timer.start(100)
		window.hide_show()

class MainWindow(QMainWindow):

	def __init__(self,*args, **kwargs):
		super(MainWindow, self).__init__(*args, **kwargs)

		#GUI node
		rospy.init_node('GUI')
		rospy.Subscriber("/llm_response", String ,self.store_llm_response)
		rospy.Subscriber("/run_gpt_code", String, self.store_run_gpt_code)
		self.chatPublisher=rospy.Publisher("/llm_propmt", String,queue_size=10)
		self.robot_name_pub=rospy.Publisher("/start_llm",String,queue_size=1)


		self.llm_responses=[]
		self.last_displayed_response=0
		self.updated=False
		self.timer = QTimer()
		self.timer.timeout.connect(self.display_llm_response)
		self.timer.start(1000)

		#self.gpt=GPT()
		# Center Screen
		qtRectangle = self.frameGeometry()
		centerPoint = QDesktopWidget().availableGeometry().center()
		qtRectangle.moveCenter(centerPoint)
		self.move(qtRectangle.topLeft())

		layout1 = QVBoxLayout()
		layout2 = QHBoxLayout()
		self.editor = QPlainTextEdit()
		fixedfont = QFontDatabase.systemFont(QFontDatabase.FixedFont)
		fixedfont.setPointSize(12)
		self.editor.setFont(fixedfont)

		# self.path holds the path of the currently open file.
		# If none, we haven't got a file open yet (or creating new).
		self.path = "./LLM/gpt_code.py"

		self.text_area = QTextEdit()
		self.text_area.setFocusPolicy(Qt.NoFocus)
		self.message = QLineEdit()
		chatBox = QVBoxLayout()
		chatBox.addWidget(self.text_area)
		chatBox.addWidget(self.message)
        
		self.message.returnPressed.connect(self.send_message)

		self.redColor = QColor(255, 0, 0)
		self.blackColor = QColor(0, 0, 0)
		self.greenColor = QColor(0, 255, 0)

		# adding editor, chatbox and rviz to the layout
		self.__highlighter = PythonHighlighter(self.editor.document())
		layout2.addWidget(self.editor)

		self.rviz=MyViz(robot)

		self.console = PythonConsole()
		self.console.push_local_ns('run', run_gpt_code)

		lower_box = QHBoxLayout()
		lower_box.addLayout(chatBox)
		lower_box.addWidget(self.console)

		layout2.addWidget(self.rviz)
		layout1.addLayout(layout2)
		#layout1.addLayout(chatBox)
		#layout1.addWidget(self.console)
		layout1.addLayout(lower_box)

		# creating a QWidget layout
		container = QWidget()

		# setting layout to the container
		container.setLayout(layout1)

		# making container as central widget
		self.setCentralWidget(container)

		# creating a status bar object
		self.status = QStatusBar()

		# setting stats bar to the window
		self.setStatusBar(self.status)

		# creating a file tool bar
		file_toolbar = QToolBar("File")

		# adding file tool bar to the window
		self.addToolBar(file_toolbar)

		# creating a file menu
		file_menu = self.menuBar().addMenu("&File")

		# creating actions to add in the file menu
		# creating a open file action
		open_file_action = QAction("Open file", self)

		# setting status tip
		open_file_action.setStatusTip("Open file")

		# adding action to the open file
		open_file_action.triggered.connect(self.file_open)

		# adding this to file menu
		file_menu.addAction(open_file_action)

		# adding this to tool bar
		file_toolbar.addAction(open_file_action)

		# similarly creating a save action
		save_file_action = QAction("Save", self)
		save_file_action.setStatusTip("Save current page")
		save_file_action.triggered.connect(self.file_save)
		file_menu.addAction(save_file_action)
		file_toolbar.addAction(save_file_action)

		# similarly creating save action
		saveas_file_action = QAction("Save As", self)
		saveas_file_action.setStatusTip("Save current page to specified file")
		saveas_file_action.triggered.connect(self.file_saveas)
		file_menu.addAction(saveas_file_action)
		file_toolbar.addAction(saveas_file_action)

		# for print action
		print_action = QAction("Print", self)
		print_action.setStatusTip("Print current page")
		print_action.triggered.connect(self.file_print)
		file_menu.addAction(print_action)
		file_toolbar.addAction(print_action)

		# creating another tool bar for editing text
		edit_toolbar = QToolBar("Edit")

		# adding this tool bar to the main window
		self.addToolBar(edit_toolbar)

		# creating a edit menu bar
		edit_menu = self.menuBar().addMenu("&Edit")

		# adding actions to the tool bar and menu bar

		# undo action
		undo_action = QAction("Undo", self)
		# adding status tip
		undo_action.setStatusTip("Undo last change")

		# when triggered undo the editor
		undo_action.triggered.connect(self.editor.undo)

		# adding this to tool and menu bar
		edit_toolbar.addAction(undo_action)
		edit_menu.addAction(undo_action)

		# redo action
		redo_action = QAction("Redo", self)
		redo_action.setStatusTip("Redo last change")

		# when triggered redo the editor
		redo_action.triggered.connect(self.editor.redo)

		# adding this to menu and tool bar
		edit_toolbar.addAction(redo_action)
		edit_menu.addAction(redo_action)

		# cut action
		cut_action = QAction("Cut", self)
		cut_action.setStatusTip("Cut selected text")

		# when triggered cut the editor text
		cut_action.triggered.connect(self.editor.cut)

		# adding this to menu and tool bar
		edit_toolbar.addAction(cut_action)
		edit_menu.addAction(cut_action)

		# copy action
		copy_action = QAction("Copy", self)
		copy_action.setStatusTip("Copy selected text")

		# when triggered copy the editor text
		copy_action.triggered.connect(self.editor.copy)

		# adding this to menu and tool bar
		edit_toolbar.addAction(copy_action)
		edit_menu.addAction(copy_action)

		# paste action
		paste_action = QAction("Paste", self)
		paste_action.setStatusTip("Paste from clipboard")

		# when triggered paste the copied text
		paste_action.triggered.connect(self.editor.paste)

		# adding this to menu and tool bar
		edit_toolbar.addAction(paste_action)
		edit_menu.addAction(paste_action)

		# select all action
		select_action = QAction("Select all", self)
		select_action.setStatusTip("Select all text")

		# when this triggered select the whole text
		select_action.triggered.connect(self.editor.selectAll)

		# adding this to menu and tool bar
		edit_toolbar.addAction(select_action)
		edit_menu.addAction(select_action)


		# wrap action
		wrap_action = QAction("Wrap text to window", self)
		wrap_action.setStatusTip("Check to wrap text to window")

		# making it checkable
		wrap_action.setCheckable(True)

		# making it checked
		wrap_action.setChecked(True)

		# adding action
		wrap_action.triggered.connect(self.edit_toggle_wrap)

		# adding it to edit menu not to the tool bar
		edit_menu.addAction(wrap_action)

		# creating another tool bar for editing text
		view_toolbar = QToolBar("View")

		# adding this tool bar to the main window
		self.addToolBar(view_toolbar)

		# creating a edit menu bar
		view_menu = self.menuBar().addMenu("&View")

		# for toggling the editor
		toggle_action = QAction("Hide/Show Editor", self)
		toggle_action.setStatusTip("Toggle editor window to hide or show")
		toggle_action.triggered.connect(self.hide_show)
		view_menu.addAction(toggle_action)
		view_toolbar.addAction(toggle_action)

		# calling update title method
		self.update_title()

		# showing all the components
		self.show()

		self.console.eval_in_thread()
		#self.console.eval_queued()
		self.run_flag=False
		self.robot_name_pub.publish(robot)

		# The status of the editor (not hidden)
		self.hidden = False


	# creating dialog critical method
	# to show errors
	def dialog_critical(self, s):

		# creating a QMessageBox object
		dlg = QMessageBox(self)

		# setting text to the dlg
		dlg.setText(s)

		# setting icon to it
		dlg.setIcon(QMessageBox.Critical)

		# showing it
		dlg.show()

	# action called by file open action
	def file_open(self):

		# getting path and bool value
		path, _ = QFileDialog.getOpenFileName(self, "Open file", "",
							"Text documents (*.txt);All files (*.*)")

		# if path is true
		if path:
			# try opening path
			try:
				with open(path, 'rU') as f:
					# read the file
					text = f.read()

			# if some error occurred
			except Exception as e:

				# show error using critical method
				self.dialog_critical(str(e))
			# else
			else:
				# update path value
				self.path = path

				# update the text
				self.editor.setPlainText(text)

				# update the title
				self.update_title()

	# action called by file save action
	def file_save(self):

		# if there is no save path
		if self.path is None:

			# call save as method
			return self.file_saveas()

		# else call save to path method
		self._save_to_path(self.path)

	# action called by save as action
	def file_saveas(self):

		# opening path
		path, _ = QFileDialog.getSaveFileName(self, "Save file", "",
							"Text documents (*.txt);All files (*.*)")

		# if dialog is cancelled i.e no path is selected
		if not path:
			# return this method
			# i.e no action performed
			return

		# else call save to path method
		self._save_to_path(path)

	# save to path method
	def _save_to_path(self, path):

		# get the text
		text = self.editor.toPlainText()

		# try catch block
		try:

			# opening file to write
			with open(path, 'w') as f:

				# write text in the file
				f.write(text)

		# if error occurs
		except Exception as e:

			# show error using critical
			self.dialog_critical(str(e))

		# else do this
		else:
			# change path
			self.path = path
			# update the title
			self.update_title()

	# action called by print
	def file_print(self):

		# creating a QPrintDialog
		dlg = QPrintDialog()

		# if executed
		if dlg.exec_():

			# print the text
			self.editor.print_(dlg.printer())

	def hide_show(self):
		if self.hidden:
			self.editor.show()
			self.hidden = False
		else:
			self.editor.hide()
			self.hidden = True

	# update title method
	def update_title(self):

		# setting window title with prefix as file name
		# suffix as PyQt5 Notepad
		self.setWindowTitle("%s - PyQt5 Notepad" %(os.path.basename(self.path)
												if self.path else "Untitled"))

	# action called by edit toggle
	def edit_toggle_wrap(self):

		# chaining line wrap mode
		self.editor.setLineWrapMode(1 if self.editor.lineWrapMode() == 0 else 0 )

	# chatbox send message to llm 
	def send_message(self):
		prompt=self.message.text()
		rospy.loginfo(prompt)
		self.chatPublisher.publish(prompt)
		self.message.clear()
		self.text_area.setTextColor(self.redColor)
		self.text_area.append("User> ")
		self.text_area.setTextColor(self.blackColor)
		self.text_area.insertPlainText(prompt)
		#self.gpt.get_gpt_response(prompt)

	def store_llm_response(self,data):
		new_response=data.data
		self.llm_responses.append(new_response)
		self.updated=True

	def store_run_gpt_code(self,data):
		self.run_flag=data.data=="True"
		#print("did this")

	# chatbox recieve and display text from llm
	def display_llm_response(self):
		if self.updated or len(self.llm_responses)>self.last_displayed_response:
			new_response=self.llm_responses[self.last_displayed_response]
			self.text_area.setTextColor(self.greenColor)
			self.text_area.append(robot+"> ")
			self.text_area.setTextColor(self.blackColor)
			self.text_area.insertPlainText(new_response)
			self.show_llm_code()
			self.last_displayed_response+=1
			self.updated=False
			if self.run_flag:
				self.console.insert_input_text('run()')
				#self.console.enterEvent(QEvent(QEvent.KeyPress)).emit()
				cursor = self.console._textCursor()
				cursor.movePosition(QTextCursor.End)
				self.console._setTextCursor(cursor)
				buffer = self.console.input_buffer()
				self.console._hide_cursor()
				self.console.insert_input_text('\n', show_ps=False)
				self.console.process_input(buffer)
				#self.console._handle_enter_key(QEvent.KeyPress)
				self.run_flag=False
				#print('made false')	

	def show_llm_code(self):
		try:
			with open(self.path, 'rU') as f:
				# read the file
				text = f.read()

		# if some error occurred
		except Exception as e:

			# show error using critical method
			self.dialog_critical(str(e))
		# else
		else:
			# update the text
			self.editor.setPlainText(text)

			# update the title
			self.update_title()

def closing_llm():
	p=rospy.Publisher("/llm_propmt", String,queue_size=10)
	rospy.loginfo("shutting down the llm node...")
	time.sleep(2)
	p.publish("!quit")
	#event.accept()

# drivers code
#if __name__ == '__main__':

def Main():

	global window

	# creating PyQt5 application
	app = QApplication(sys.argv)

	# setting application name
	app.setApplicationName("Natural Robot")
	app.aboutToQuit.connect(closing_llm)

	# creating a main window object
	#window = MainWindow()
	window = startUpWindow()
	window.show()

	# loop
	sys.exit(app.exec_())