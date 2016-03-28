"""
Open Source Dobot GUI Application: open-dobot-gui
Contains main function for entire application, GUI initilization and functions
First Author: Mike Ferguson www.mikeahferguson.com 3/26/2016
Additional Authors (Add your name below):
1.
License: MIT

Requires PyQt5 to be installed.

Anything Qt specific (functions, classes, etc.. starts with the letter Q)

The GUI template is created by a program called QtDesigner, which spits out a .ui file, which is basically just an XML
file that describes the GUI elements. In the designer, the elements are given object names. The first step of the code
below is to load the .ui file and get a reference to it.
"""




import serial, time, struct
import sys, os, threading
from threading import Thread
from PyQt5.QtWidgets import QApplication, QMainWindow, QListWidget, QListWidgetItem, QFileDialog, QMessageBox
from PyQt5 import uic, QtCore, QtGui
import DobotInverseKinematics
import serial.tools.list_ports

# This loads the GUI from the .ui file that is created by QtDesigner. The .ui file should be in the same folder as this
# python file (or specify different path).
Ui_MainWindow, QtBaseClass = uic.loadUiType('DobotMainUi.ui')


# Here, a class is defined to represent the entire GUI. It is derived from a Qt class named QMainWindow, which
# corresponds to the GUI type specified in QtDesigner. All of the functional aspects (as opposed to design aspects) of
# the GUI are defined in this class. For example, what happens when a user presses a button.
class DobotGUIApp(QMainWindow):
    # class initialization function (initialize the GUI)
    def __init__(self, parent=None):
        # I'm a python noob, but I'm guessing this means initialize the parent class. I imagine all the super classes
        # have to be explicitly initialized.
        super(DobotGUIApp, self).__init__(parent)
        # This sets up the ui variable of this class to refer to the loaded .ui file.
        self.ui = Ui_MainWindow()
        # This call is required. Does whatever set up is required, probably gets references to the elements and so on.
        self.ui.setupUi(self)

        # Anything named after self.ui. (e.g. self.ui.x) means you are referring to an object name x that corresponds
        # to an element in the gui. Qt uses a signals and slots framework to define what happens in response to UI
        # events. You'll have to look that up if you're interested in it.


        ###
        # Connect gui elements in the .ui file to event handling functions.
        ###

        # connect serial ports list refresh button clicked event to the update serial port list function
        self.ui.pushButtonRefreshSerialPortsList.clicked.connect(self.update_serial_port_list)
        # connect move coordinates button clicked event to function to move to the coordinate specified
        self.ui.pushButtonMoveToCoordinate.clicked.connect(self.pushButtonMoveToCoordinate_clicked)
        # connect move to angles button clicked event to function to move to the angles specified
        self.ui.pushButtonMoveToAngles.clicked.connect(self.pushButtonMoveToAngles_clicked)


        ###
        # Define application class variables.
        ###

        # connect to the arduino serial port

        try:
            # variable for arduino serial. I'm connecting to the serial named COM8 here (I'm using windows 10)
            # the second argument is the baud rate, or how fast the serial connection is. needs to be the same in firmware
            # This will reset the Arduino, and make the LED flash once.
            self.arduinoSerial = serial.Serial('COM8', 115200)
            # Must give Arduino time to reset
            #  "Any time less than this does not seem to work..." quoted from someone else on a blog
            time.sleep(1.5)
            # I think this clears the input stream?
            # I think I may not need this, but I'm including it anyways, against the advice of someone on stackexchange.
            # I don't think it does anything bad, just one more line of unnecessary code.
            self.arduinoSerial.flushInput()
        except Exception as e:
            self.show_a_warning_message_box('Unknown error connecting to the arduino serial port. Code error shown below:',
                                            repr(e),
                                            'Arduino Serial Connection Error')


        ###
        # General initialization code
        ###

        # populate the serial ports list widget
        self.update_serial_port_list()







    def pushButtonMoveToCoordinate_clicked(self):

        # get moveTo coordinate text values from lineedits
        moveToX = self.ui.lineEditMoveToX.text()
        moveToY = self.ui.lineEditMoveToY.text()
        moveToZ = self.ui.lineEditMoveToZ.text()

        # check that the values were not empty
        if (moveToX == '' or moveToY == '' or moveToZ == ''):
            self.show_a_warning_message_box('Missing a coordinate value.',
                                            'Check that you entered a value for each dimension.',
                                            'Invalid coordinate for move to command')
            return

        # convert values from string to float and ensure that the values entered were actually numbers
        try:
            moveToXFloat = float(moveToX)
            moveToYFloat = float(moveToY)
            moveToZFloat = float(moveToZ)
        except Exception as e:
            self.show_a_warning_message_box('Check that your coordinate values are numbers and not letters. The code '
                                            + 'error is shown below:',
                                                repr(e),
                                                'Coordinate value conversion to float error')
            return




        # call inverse kinematics function to convert from cartesian coordinates to angles for Dobot arm
        # moveToAngles is a list of angles (type float) with the following order: [base angle, upper arm angle, lower arm angle]
        # catch any errors (likely due to coordinates out of range being input) NEED TO ADDRESS THIS AT SOME POINT
        try:
            moveToAngles = DobotInverseKinematics.convert_cartesian_coordinate_to_arm_angles(moveToXFloat,moveToYFloat,moveToZFloat,
            DobotInverseKinematics.lengthUpperArm, DobotInverseKinematics.lengthLowerArm, DobotInverseKinematics.heightFromBase)
        except Exception as e:
            self.show_a_warning_message_box('Unknown inverse kinematics error. Check that your coordinate values are within the robot\'s range. '
                                            + 'The error is shown below:',
                                                repr(e),
                                                'Inverse Kinematics Error')
            return


        # check that inverse kinematics did not run into a range error. If it does, it should return -999 for all angles, so check that.
        if(moveToAngles[0] == -999):
            self.show_a_warning_message_box('Desired coordinate is outside of the robot\'s range.',
                                                'It is impossible for the robot arm to reach the coordinate you specified. Build longer arms if this range is desired.'
                                                + 'You will probably need higher torque stepper motors as well.',
                                                'Inverse Kinematics Range Error')
            return

        print('ik base angle')
        print(moveToAngles[0])
        print('ik upper angle')
        print(moveToAngles[1])
        print('ik lower angle')
        print(moveToAngles[2])





        moveToUpperArmAngleFloat = moveToAngles[1]
        moveToLowerArmAngleFloat = moveToAngles[2]

        transformedUpperArmAngle = (90 - moveToUpperArmAngleFloat)
        #-90 different from c++ code, accounts for fact that arm starts at the c++ simulation's 90
        # note that this line is different from the similar line in the move angles function. Has to do with the inverse kinematics function
        # and the fact that the lower arm angle is calculated relative to the upper arm angle.
        transformedLowerArmAngle = 360 + (transformedUpperArmAngle - moveToLowerArmAngleFloat) - 90
        print('transformed upper angle:')
        print(transformedUpperArmAngle)
        print('transformed lower angle:')
        print(transformedLowerArmAngle)


        #check that the final angles are mechanically valid. note that this check only considers final angles, and not angles while the arm is moving
        # need to pass in real world angles
        # real world base and upper arm angles are those returned by the ik function.
        # real world lower arm angle is -1 * transformedLowerArmAngle
        if(self.check_for_angle_limits_is_valid(moveToAngles[0], moveToAngles[1], -1 * transformedLowerArmAngle)):
            # continue on to execute the arduino code
            pass
        else:
            # exit, don't move. the check function takes care of the warning message
            return



        # INSERT CODE HERE TO SEND MOVEMENT COMMANDS TO ARDUINO
        # I'm simply writing three floats to the arduino. See the following two stack exchange posts for more details on this:
        # http://arduino.stackexchange.com/questions/5090/sending-a-floating-point-number-from-python-to-arduino
        # ttps://arduino.stackexchange.com/questions/3753/how-to-send-numbers-to-arduino-uno-via-python-3-and-the-module-serial
        self.arduinoSerial.write( struct.pack('f',moveToAngles[0]) )
        self.arduinoSerial.write( struct.pack('f',transformedUpperArmAngle) )
        self.arduinoSerial.write( struct.pack('f',transformedLowerArmAngle) )

        # if movement was successful, update the current position
        # note that float values are rounded to 3 decimal places for display and converted to strings
        self.ui.labelBaseAngleValue.setText(str(round(moveToAngles[0],3)))
        self.ui.labelUpperArmAngleValue.setText(str(round(moveToAngles[1],3)))
        self.ui.labelLowerArmAngleValue.setText(str(round(moveToAngles[2],3)))

        self.ui.labelCurrentXValue.setText(str(round(moveToXFloat,3)))
        self.ui.labelCurrentYValue.setText(str(round(moveToYFloat,3)))
        self.ui.labelCurrentZValue.setText(str(round(moveToZFloat,3)))

        # code for debugging purposes. the firmware I am using (at time of writing this) is set up to print the 3 angles it read to the serial
        # this reads the 3 angles that the arduino printed from the serial. There is certainly a better way to do this.
        # this was quick and dirty and is prone to fatal errors (fatal for this program that is).
        for i in range(0,15 ):
            print ( self.arduinoSerial.readline() )





    def pushButtonMoveToAngles_clicked(self):
        # get moveTo angle text values from lineedits
        moveToBaseAngle = self.ui.lineEditMoveToBaseAngle.text()
        moveToUpperArmAngle = self.ui.lineEditMoveToUpperArmAngle.text()
        moveToLowerArmAngle = self.ui.lineEditMoveToLowerArmAngle.text()

        # check that the values were not empty
        if (moveToBaseAngle == '' or moveToUpperArmAngle == '' or moveToLowerArmAngle == ''):
            self.show_a_warning_message_box('Missing a angle value.',
                                            'Check that you entered a value for each angle.',
                                            'Invalid angles for move to command')
            return

        # convert values from string to float and ensure that the values entered were actually numbers
        try:
            moveToBaseAngleFloat = float(moveToBaseAngle)
            moveToUpperArmAngleFloat = float(moveToUpperArmAngle)
            moveToLowerArmAngleFloat = float(moveToLowerArmAngle)
        except Exception as e:
            self.show_a_warning_message_box('Check that your angle values are numbers and not letters. The code '
                                            + 'error is shown below:',
                                                repr(e),
                                                'Angle value conversion to float error')
            return


        #check that the final angles are mechanically valid. note that this check only considers final angles, and not angles while the arm is moving
        # need to pass in real world angles
        # real world base, upper, and lower arm angles are those entered in the text box.
        if(self.check_for_angle_limits_is_valid(moveToBaseAngleFloat, moveToUpperArmAngleFloat, moveToLowerArmAngleFloat)):
            # continue on to execute the arduino code
            pass
        else:
            # exit, don't move. the check function takes care of the warning message
            return


        transformedUpperArmAngle = (90 - moveToUpperArmAngleFloat)
        transformedLowerArmAngle = moveToLowerArmAngleFloat*-1
        print('transformed upper angle:')
        print(transformedUpperArmAngle)
        print('transformed lower angle:')
        print(transformedLowerArmAngle)

        # INSERT CODE HERE TO SEND MOVEMENT COMMANDS TO ARDUINO
        # I'm simply writing three floats to the arduino. See the following two stack exchange posts for more details on this:
        # http://arduino.stackexchange.com/questions/5090/sending-a-floating-point-number-from-python-to-arduino
        # ttps://arduino.stackexchange.com/questions/3753/how-to-send-numbers-to-arduino-uno-via-python-3-and-the-module-serial
        self.arduinoSerial.write( struct.pack('f',moveToBaseAngleFloat) )
        self.arduinoSerial.write( struct.pack('f',transformedUpperArmAngle) )
        self.arduinoSerial.write( struct.pack('f',transformedLowerArmAngle) )

        # if movement was successful, update the current position
        # note that float values are rounded to 3 decimal places for display and converted to strings
        self.ui.labelBaseAngleValue.setText(str(round(moveToBaseAngleFloat,3)))
        self.ui.labelUpperArmAngleValue.setText(str(round(moveToUpperArmAngleFloat,3)))
        self.ui.labelLowerArmAngleValue.setText(str(round(moveToLowerArmAngleFloat,3)))

        """
        # need to implement forward kinematics
        self.ui.labelCurrentXValue.setText(str(round(moveToXFloat,3)))
        self.ui.labelCurrentYValue.setText(str(round(moveToYFloat,3)))
        self.ui.labelCurrentZValue.setText(str(round(moveToZFloat,3)))
        """

        # code for debugging purposes. the firmware I am using (at time of writing this) is set up to print the 3 angles it read to the serial
        # this reads the 3 angles that the arduino printed from the serial. There is certainly a better way to do this.
        # this was quick and dirty and is prone to fatal errors (fatal for this program that is).
        for i in range(0,15 ):
            print ( self.arduinoSerial.readline() )


    # angles passed as arguments here should be real world angles (horizontal = 0, below is negative, above is positive)
    # i.e. they should be set up the same way as the unit circle is
    def check_for_angle_limits_is_valid(self, baseAngle, upperArmAngle, lowerArmAngle):

        returnBool = True
        # implementing limit switches and IMUs will make this function more accurate and allow the user to calibrate the limits
        # necessary for this function.
        # Not currently checking the base angle

        # check the upperArmAngle
        # max empirically determined to be around 107 - 108 degrees. Using 105.
        # min empirically determined to be around -23/24 degrees. Using -20.
        if (-20 <= upperArmAngle <= 105):
            # do nothing, return value already initialized true
            pass
        else:
            self.show_a_warning_message_box('Upper arm angle out of range.',
                                            'Upper arm must have an angle between -20 and 105 degrees. It is mechanically constrained.',
                                            'Upper Arm Range Error')
            returnBool = False

        # check the lowerArmAngle
        # the valid Lower Arm angle is dependent on the upper arm angle. The real world angle of the lower arm (0 degrees = horizontal) needs to be evaluated.
        # min empirically determined to be around -105 degrees. Using -102.
        # max empirically determined to be around 21 degrees. Using 18.


        if (-102 <= lowerArmAngle <= 18):
            # do nothing, already initialized true
            pass
        else:
            self.show_a_warning_message_box('Lower arm angle out of range.',
                                            'Lower arm must have a real world angle between -102 and 18 degrees. It is mechanically constrained.',
                                            'Lower Arm Range Error')
            returnBool = False


        return returnBool




    def update_serial_port_list(self):
        """
        Updates the serial ports qtablewidget in the configuration tab with qlistwidgetitems containing text values
        that are the names of the serial port. Calls function to get a list of ports. This function works for
        Windows, Mac, and Linux
        """
        # must clear the listwidget of any previous items
        self.ui.listWidgetSerialPorts.clear()
        # get a list of all serial ports (all, not just open ones)
        listOfSerialPorts = serial.tools.list_ports.comports()
        # add each serial port name (string) to the list as a qlistwidgetitem with the string value
        for portName in listOfSerialPorts:
            item = QListWidgetItem(portName[0])
            self.ui.listWidgetSerialPorts.addItem(item)


    def show_a_warning_message_box(self, text, infoText, windowTitle):
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Warning)
            msg.setText(text)
            msg.setInformativeText(infoText)
            msg.setWindowTitle(windowTitle)
            msg.exec()




# main function
if __name__ == '__main__':
    # These first three lines initialize the Qt application/GUI.
    app = QApplication(sys.argv)
    window = DobotGUIApp()
    # displays the GUI
    window.show()

    # write whatever set up or logic code you want to here.


    # Says to exit the whole code when the Qt application is closed. app.exec returns some value when qt app quits
    sys.exit(app.exec_())




