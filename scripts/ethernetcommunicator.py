#!/usr/bin/python
import socket
import time
import re
import errno

"""
This class contains functions to communicate over ethernet with the controller

Currently contains 5 demo functions:
    - motor_on
    - motor_off
    - motor_jog
    - get_input_1
    - send_command (any text command)
"""

class Ethernetcommunicator:
    # Executed upon creation of an instance of the class.
    # Sets up the socket and initializes the class 
    def __init__(self):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect(('10.1.10.11', 23))
        self.s.settimeout(False)
        print("initialized ethernetcommunicator")


    # Sends any ascii command to the controller
    def send_command(self, command, response_handler=None):
        msg = (command + '\r\n')
        self.s.sendall(msg)

        #Don't wait longer than a 1/10 of a second to recieve a response.
        #This allows for reading of the error numbers that don't end in a ">" 
        endtime = time.time() + .1

        #Reads one character at a time, checking if the last character is a '>' or if time is up
        rsp = ""
        while rsp == "" or rsp[-1] != '>':
            # Check if time is up
            if time.time() > endtime:
                break
            # Read one character over Ethernet
            try:
                rsp = rsp + self.s.recv(1)
            # An error is thrown if there is no character to read.
            except socket.error, e:
                err = e.args[0]
                if err == errno.EAGAIN or err == errno.EWOULDBLOCK:
                    # No character to read yet. Try again.
                    continue
                else:
                    # a "real" error occurred
                    print(e)
                    sys.exit(1)

        return rsp

    def motor_on(self):
        response = self.send_command('a1 mtr on')
        return response

    def motor_off(self):
        response = self.send_command('a1 mtr off')
        return response

    def motor_jog(self, speed):
        response = self.send_command('a1 jog ' + str(speed))
        return response

    def get_input_1(self):
        response = self.send_command('inb 1')
        if not response:
            return False
        #This try/except thing is a hack. If two messages are recieved
        #at the same time, the response is invalid. If this invalid
        #response can not be converted to a float, then it throws a Value Error.
        #I except it to prevent the program from crashing.
        try:
            return bool(float(response[2:-2]))
        except ValueError:
            return False

#This is true if this program is directly executed (not imported)
if __name__ == '__main__':
    print("Hello! Running main!")
    cm = Ethernetcommunicator()
    print(cm.motor_on())
    print(cm.get_input_1())
    cm.motor_jog(4)
    time.sleep(3)
    print(cm.motor_off())


