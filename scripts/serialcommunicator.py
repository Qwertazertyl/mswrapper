#!/usr/bin/python
import serial
import time
import re

"""
This class contains functions to communicate over serial with the controller

Currently contains 5 demo functions:
    - motor_on
    - motor_off
    - motor_jog
    - get_input_1
    - send_command (Any text command)
"""

class Serialcommunicator:
    def __init__(self):
        self.ser = serial.Serial('/dev/ttyS0', 9600, timeout=.1)
        print("initialized asciicommunicator")

    #Send any ascii command to the controller via serial.
    def send_command(self, command):
        msg = (command + '\r\n')
        self.ser.write(msg)

        #Don't wait more than 1/10 of a second to recieve a response
        #This allows for reading the error numbers that don't end in a ">"
        endtime = time.time() + .1

        #Reads one character at a time, checking if the last character is a '>' or if time is up
        rsp = ""
        while rsp == "" or rsp[-1] != '>':
            if time.time() >= endtime:
                return rsp or False

            #read one character over serial
            rsp = rsp + self.ser.read(1)

        return rsp

    def motor_on(self):
        self.send_command('a1 mtr on')

    def motor_off(self):
        self.send_command('a1 mtr off')

    def motor_jog(self, speed):
        self.send_command('a1 jog ' + str(speed))

    def get_input_1(self):
        response = self.send_command('inb 1')
        if not response:
            return False
        #This try/except thing is a hack. If two messages are recieved
        #at the same time, the response is invalid. If this invalid
        #response can not be converted to a float, then it throws a ValueError.
        #I except it to prevent the program from crashing.
        try:
            return bool(float(response[2:-2]))
        except ValueError:
            return False

if __name__ == '__main__':
    print("Hello! Running main!")
    cm = Serialcommunicator()
    cm.motor_on()
    print(cm.get_input_1())
    cm.motor_jog(4)
    time.sleep(3)
    cm.motor_off()


