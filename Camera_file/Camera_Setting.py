from PyQt5 import QtCore, QtGui,QtWidgets
from Setting_UI import Ui_MainWindow
import configparser
import numpy as np
import sys
import cv2
import os
import errno
from pathlib import Path

"""GST_STR = 'nvarguscamerasrc \
		! video/x-raw(memory:NVMM), width=1280, height=720, format=(string)NV12, framerate=(fraction)120/1 \
		! nvvidconv ! video/x-raw, width=(int)1280, height=(int)720, format=(string)BGRx \
		!videobalance   \
        ! videoconvert \
		! appsink max-buffers=1 drop=True' """
# gainrange="8 20" gainrange="3 8"
#GST_STR = 'nvarguscamerasrc saturation=1.1\
GST_STR = 'nvarguscamerasrc saturation=1.3\
           tnr-strength=-0.5 tnr-mode=1\
		! video/x-raw(memory:NVMM), width=1280, height=720, format=(string)NV12, framerate=(fraction)60/1 \
		! nvvidconv ! video/x-raw, width=(int)1280, height=(int)720, format=(string)BGRx \
		!videobalance  \
        ! videoconvert \
		! appsink max-buffers=1 drop=True'
# hue=0 contrast=1. saturation=1. brightness=-0.25
class Movie(QtWidgets.QMainWindow):
    msec = 20 # ms
    def __init__(self,parent=None):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        self.config_ini = configparser.ConfigParser()
        self.config_ini_path = str(Path(__file__).resolve().parent.parent) + '/config.ini'
        print(self.config_ini_path)

        self.capture = cv2.VideoCapture(GST_STR, cv2.CAP_GSTREAMER)
        if self.capture.isOpened() is False:
            raise("IO Error")

        if not os.path.exists(self.config_ini_path):
            raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), self.config_ini_path)
        self.config_ini.read(self.config_ini_path, encoding='utf-8')
            
        self.hsv_min= [0] * 3
        self.hsv_max= [0] * 3

        self.settingData = {'Orange': [[0]*3]*3, 'Blue': [[0]*3]*3, 'Yellow': [[0]*3]*3}
        self.setting_changed = False
        
        self.scene_cor = QtWidgets.QGraphicsScene()
        self.scene_mask = QtWidgets.QGraphicsScene()
        self.change_object()
        self.set()

        timer = QtCore.QTimer(self)
        timer.timeout.connect(self.set)
        timer.start(self.msec)

    def set(self):
        ret, cv_img = self.capture.read()
        if ret == False:
            return
        cv_img = cv2.resize(cv_img[:, 280:1000], (480, 480))
        hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV_FULL)
        cv_img = cv2.cvtColor(cv_img,cv2.COLOR_BGR2RGB)
        height, width, dim = cv_img.shape

        self.hsv_min[0] = self.ui.Hue_min.value()
        self.hsv_max[0] = self.ui.Hue_max.value()
        self.hsv_min[1] = self.ui.Satuation_min.value()
        self.hsv_max[1] = self.ui.Satuation_max.value()
        self.hsv_min[2] = self.ui.Value_min.value()
        self.hsv_max[2] = self.ui.Value_max.value()
        self.ui.Hue_min_value.setText(str(self.hsv_min[0]))
        self.ui.Hue_max_value.setText(str(self.hsv_max[0]))
        self.ui.Sat_min_value.setText(str(self.hsv_min[1]))
        self.ui.Sat_max_value.setText(str(self.hsv_max[1]))
        self.ui.Val_min_value.setText(str(self.hsv_min[2]))
        self.ui.Val_max_value.setText(str(self.hsv_max[2]))
        #print(self.hsv_min, self.hsv_max)
        #self.ui.system_info.clear()
        text = "Orange: \n" + self.config_ini.get('Camera',"Orange_min")+" to "+ self.config_ini.get('Camera', "orange_max")\
            + "\nBlue: \n" + self.config_ini.get('Camera', "Blue_min")+" to "+ self.config_ini.get('Camera', "Blue_max")+\
                "\nYellow: \n" + self.config_ini.get('Camera', "Yellow_min") + " to "+ self.config_ini.get('Camera', "Yellow_min")
        self.ui.system_info.setText(text)
        if self.setting_changed:
            self.ui.system_info.setText(str(self.ui.system_info.toPlainText()) + "\n        **Setting is changed !**")

        if self.hsv_max < self.hsv_min:
            mask_image1 = cv2.inRange(hsv, (0, self.hsv_min[1], self.hsv_min[2]), tuple(self.hsv_max))
            mask_image2 = cv2.inRange(hsv, tuple(self.hsv_min), (255, self.hsv_max[1], self.hsv_max[2]))
            mask_image = mask_image1 + mask_image2
        else :
            mask_image = cv2.inRange(hsv, tuple(self.hsv_min), tuple(self.hsv_max))

        contours,_ =cv2.findContours(mask_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        rects = []
        for contour in contours:
            approx = cv2.convexHull(contour)
            rect = cv2.boundingRect(approx)
            rects.append(np.array(rect))
        
        if len(rects) >0:
            rect = max(rects, key=(lambda x: x[2] * x[3]))
            cv2.rectangle(cv_img, tuple(rect[0:2]), tuple(rect[0:2] + rect[2:4]), (255, 255, 255), thickness=2)

        self.image_mask = QtGui.QImage(mask_image, width, height, QtGui.QImage.Format_Grayscale8)
        self.image_cor = QtGui.QImage(cv_img.data, width, height, QtGui.QImage.Format_RGB888)
        self.item_mask = QtWidgets.QGraphicsPixmapItem(QtGui.QPixmap.fromImage(self.image_mask))
        self.item_cor = QtWidgets.QGraphicsPixmapItem(QtGui.QPixmap.fromImage(self.image_cor))
        self.scene_cor.clear()
        self.scene_mask.clear()
        self.scene_cor.addItem(self.item_cor)
        self.scene_mask.addItem(self.item_mask)
        self.ui.graphicsView.setScene(self.scene_mask)
        self.ui.graphicsView_2.setScene(self.scene_cor)

    def change_value(self):
        self.settingData[self.ui.comboBox.currentText()] = [self.hsv_min, self.hsv_max]
        self.setting_changed = True
        

    def change_object(self):
        data_min = eval(self.config_ini.get('Camera', self.ui.comboBox.currentText() + "_min"))
        data_max = eval(self.config_ini.get('Camera', self.ui.comboBox.currentText() + "_max"))
        self.ui.Hue_min.setValue(data_min[0])
        self.ui.Hue_max.setValue(data_max[0])
        self.ui.Satuation_min.setValue(data_min[1])
        self.ui.Satuation_max.setValue(data_max[1])
        self.ui.Value_min.setValue(data_min[2])
        self.ui.Value_max.setValue(data_max[2])

    def write_file(self):
        self.config_ini.set("Camera", self.ui.comboBox.currentText()+"_min", str(self.hsv_min))
        self.config_ini.set("Camera", self.ui.comboBox.currentText()+"_max", str(self.hsv_max))
        with open(self.config_ini_path, "w", encoding="utf-8") as configfile:
            self.config_ini.write(configfile, True)
        self.setting_changed = False
    
    def __del__(self):
        self.capture.release()   

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)

    window = Movie()
    window.show()
    sys.exit(app.exec_())
