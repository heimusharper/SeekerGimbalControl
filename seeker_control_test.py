import sys
import serial
import numpy as np

from PyQt5 import QtCore, QtWidgets, Qt

import view


class RunApp(QtWidgets.QMainWindow, view.Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)

        self.pushButtonConnect.clicked.connect(self.connectSerial)

        self.pushButtonZoomIn.clicked.connect(self.zoomIn)
        self.pushButtonZoomOut.clicked.connect(self.zoomOut)
        self.pushButtonStop.clicked.connect(self.zoomStop)

        self.pushButtonFIn.clicked.connect(self.focusIn)
        self.pushButtonFOut.clicked.connect(self.focusOut)
        self.pushButtonFStop.clicked.connect(self.focusStop)

        self.pushButtonPhoto.clicked.connect(self.takePhoto)
        self.pushButtonRecStart.clicked.connect(self.videoRecStart)
        self.pushButtonRecStop.clicked.connect(self.videoRecStop)

        self.horizontalSliderYaw.valueChanged.connect(self.updateAngle)
        self.horizontalSliderRoll.valueChanged.connect(self.updateAngle)
        self.horizontalSliderPitch.valueChanged.connect(self.updateAngle)

        self.dialSpeed.valueChanged.connect(self.updateGlobalSpeed)


        self.yawSpeed = 0
        self.rollSpeed = 0
        self.pitchSpeed = 0

        self.updateGlobalSpeed()
        self.connectSerial()

    # action to serial connection
    def connectSerial(self):
        try:
            self.ser = serial.Serial(self.lineEditSerial.text(), 115200, timeout=1)
        except serial.serialutil.SerialException as e:
            self.statusBar.showMessage(str(e))

    # gimbal speed control
    def doGimbalSpeed(self, roll, pitch, yaw):
        HEADER = [0xFF, 0x01, 0x0F, 0x10]
        MODE = [0x01, 0x01, 0x01]
        rsl, rsh = self.speed2data(roll)
        psl, psh = self.speed2data(pitch)
        ysl, ysh = self.speed2data(yaw)
        ROLL = [0x00, 0x00, rsl, rsh]
        PITCH = [0x00, 0x00, psl, psh]
        YAW = [0x00, 0x00, ysl, ysh]
        CRC = self.getCRC(MODE + ROLL + PITCH + YAW)

        self.writeCommand(HEADER + MODE + ROLL + PITCH + YAW + CRC)

    # gimbal angle control
    def doGimbalAngles(self, roll, pitch, yaw):
        HEADER = [0xFF, 0x01, 0x0F, 0x10]
        MODE = [0x05, 0x05, 0x05]
        rsl, rsh = self.angle2data(roll)
        psl, psh = self.angle2data(pitch)
        ysl, ysh = self.angle2data(yaw)
        ROLL = [rsl, rsh, 0x00, 0x00]
        PITCH = [psl, psh, 0x00, 0x00]
        YAW = [ysl, ysh, 0x00, 0x00]
        CRC = self.getCRC(MODE + ROLL + PITCH + YAW)

        self.writeCommand(HEADER + MODE + ROLL + PITCH + YAW + CRC)


    def getCRC(self, data):
        checksum = sum(data)
        checksum = hex(checksum % 256)
        checksum_m = 0
        if len(checksum) == 3:
            checksum_m = "0" + checksum[2]
        elif len(checksum) == 4:
            checksum_m = checksum[2:4]
        checksum_m = int(checksum_m, 16)
        checksum_m = np.uint16(checksum_m)
        return [checksum_m]



    def dec2hex(self, dec):
        if dec >= 0:
            hexa = hex(int(dec))
        else:
            absDec = abs(int(dec))
            bina = bin(absDec)
            bina = bina[2:].zfill(16)

            inv_bina = ''
            for i in bina:  
                if i == '0':
                    inv_bina += '1'
                else:
                    inv_bina += '0'
            int_sum = int(inv_bina, 2) + int(1)
            hexa = hex(int_sum)

        return hexa


    def angle2data(self, angle):
        angle /= 0.02197265625
        hexAngle = self.dec2hex(angle)
        if len(hexAngle) == 6:
            low = hexAngle[4:6]
            high = hexAngle[2:4]
        elif len(hexAngle) == 5:
            low = hexAngle[3:5]
            high = "0" + hexAngle[2]
        elif len(hexAngle) == 4:
            low = hexAngle[2:4]
            high = "00"
        elif len(hexAngle) == 3:
            low = "0" + hexAngle[2]
            high = "00"
        else:
            self.statusBar.showMessage("ERROR: Introduce a correct angle!")
        low = int(low, 16)
        high = int(high, 16)

        low = np.uint8(low)
        high = np.uint8(high)
        return low, high

    def speed2data(self, speed):
        
        speed /= 0.1220740379 # 1 unit: 0.1220740379 degree/sec
        hexSpeed = self.dec2hex(speed)

        if len(hexSpeed) == 6:
            low = hexSpeed[4:6]
            high = hexSpeed[2:4]
        elif len(hexSpeed) == 5:
            low = hexSpeed[3:5]
            high = "0" + hexSpeed[2]
        elif len(hexSpeed) == 4:
            low = hexSpeed[2:4]
            high = "00"
        elif len(hexSpeed) == 3:
            low = "0" + hexSpeed[2]
            high = "00"
        else:
            self.statusBar.showMessage("ERROR: Introduce a correct speed!")
        
        low = int(low, 16)
        high = int(high, 16)

        low = np.uint8(low)
        high = np.uint8(high)
        return low, high

    def keyPressEvent(self, event):
        if event.isAutoRepeat():
            return
        # print(str(event.key()))
        if event.key() == 87: #Qt.Key_W:
            self.pitchSpeed = self.globalSpeed
            self.updateSpeed()
        if event.key() == 83: #Qt.Key_S:
            self.pitchSpeed = -self.globalSpeed
            self.updateSpeed()
        if event.key() == 65: #Qt.Key_A:
            self.yawSpeed = -self.globalSpeed
            self.updateSpeed()
        if event.key() == 68: #Qt.Key_D:
            self.yawSpeed = self.globalSpeed
            self.updateSpeed()
        if event.key() == 82: #Qt.Key_R:
            self.zoomIn()
        if event.key() == 70: #Qt.Key_F:
            self.zoomOut()
        if event.key() == 84: #Qt.Key_T:
            self.focusIn()
        if event.key() == 71: #Qt.Key_G:
            self.focusOut()
            

    def keyReleaseEvent(self, event):
        if event.isAutoRepeat():
            return
        # print("release")
        if event.key() == 87: #Qt.Key_W:
            self.pitchSpeed = 0
            self.updateSpeed()
        if event.key() == 83:# Qt.Key_S:
            self.pitchSpeed = 0
            self.updateSpeed()
        if event.key() == 65: #Qt.Key_A:
            self.yawSpeed = 0
            self.updateSpeed()
        if event.key() == 68: #Qt.Key_D:
            self.yawSpeed = 0
            self.updateSpeed()
        if event.key() == 67: #Qt.Key_C:
            self.takePhoto()
        if event.key() == 82: #Qt.Key_R:
            self.zoomStop()
        if event.key() == 70: #Qt.Key_F:
            self.zoomStop()
        if event.key() == 84: #Qt.Key_T:
            self.focusStop()
        if event.key() == 71: #Qt.Key_G:
            self.focusStop()

    # angle control
    def updateAngle(self):
        YAW = self.horizontalSliderYaw.value()
        ROLL = self.horizontalSliderRoll.value()
        PITCH = self.horizontalSliderPitch.value()

        self.doGimbalAngles(ROLL, PITCH, YAW)

    # angle control
    def updateSpeed(self):
        YAW = self.yawSpeed
        ROLL = self.rollSpeed
        PITCH = self.pitchSpeed
        print("angles " + str(YAW) + " " + str(PITCH))
        
        self.doGimbalSpeed(ROLL, PITCH, YAW)

    # speed update
    def updateGlobalSpeed(self):
        self.globalSpeed = self.dialSpeed.value()

    # zoom in
    def zoomIn(self):
        self.writeCommand([0x81, 0x01, 0x04, 0x07, 0x27, 0xFF])
    # zoom out
    def zoomOut(self):
        self.writeCommand([0x81, 0x01, 0x04, 0x07, 0x37, 0xFF])
    # zoom stop
    def zoomStop(self):
        self.writeCommand([0x81, 0x01, 0x04, 0x07, 0x00, 0xFF])
    # focus in
    def focusIn(self):
        self.writeCommand([0x81, 0x01, 0x04, 0x08, 0x27, 0xFF])
    # focus out
    def focusOut(self):
        self.writeCommand([0x81, 0x01, 0x04, 0x08, 0x37, 0xFF])
    # focus stop
    def focusStop(self):
        self.writeCommand([0x81, 0x01, 0x04, 0x08, 0x00, 0xFF])

    # take photo
    def takePhoto(self):
        self.writeCommand([0x81, 0x01, 0x04, 0x68, 0x01, 0xFF])
    # start video record
    def videoRecStart(self):
        self.writeCommand([0x81, 0x01, 0x04, 0x68, 0x02, 0xFF])
    # stop video record
    def videoRecStop(self):
        self.writeCommand([0x81, 0x01, 0x04, 0x68, 0x03, 0xFF])

    # serial control
    def writeCommand(self, command):
        try:
            self.ser.write(command)
        except serial.serialutil.SerialException as e:
            self.statusBar.showMessage(str(e))

    

def main():
    app = QtWidgets.QApplication(sys.argv)
    window = RunApp()
    window.show()
    app.exec_()

if __name__ == '__main__':
    main()
