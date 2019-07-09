#!/usr/bin/python

import rospy
from mswrapper.srv import *
from std_msgs.msg import String
import Tkinter

"""
I don't know how half of this gui stuff works, I just copy/pasted much of it from examples
that I found on the internet and made the changes that I needed to make. But it seems to work.

Ths graphical library I used was Tkinter.
"""

send_command = None

class GUIApp(Tkinter.Frame):
    def run_command(self):
        response = str(send_command(String(self.entry.get())))
        print(response)
        self.response.delete('1.0', Tkinter.END)
        self.response.insert(Tkinter.END, response)

    def createWidgets(self):
        self.entry = Tkinter.Entry(self)

        self.entry.grid(row=1,column=1)

        self.hi_there = Tkinter.Button(self)
        self.hi_there["text"] = "Send",
        self.hi_there["command"] = self.run_command

        self.hi_there.grid(row=1,column=2)

        self.QUIT = Tkinter.Button(self)
        self.QUIT["text"] = "QUIT"
        self.QUIT["fg"]   = "red"
        self.QUIT["command"] =  self.quit

        self.QUIT.grid(row=1,column=3)

        self.response = Tkinter.Text(self, height=4, width=40)

        self.response.grid(row=2,column=1,columnspan=3)

    def __init__(self, master=None):
        Tkinter.Frame.__init__(self, master)
        self.pack()
        self.createWidgets()

def controller():
    #Initialize and name node 
    # In ROS, nodes are uniquely named. If two nodes with the same          
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('gui_controller', anonymous=False)

    global send_command
    send_command = rospy.ServiceProxy('mswrapper/send_command', SendCommand)

    root = Tkinter.Tk()
    app = GUIApp(master=root)
    app.mainloop()
    try:
        root.destroy()
    except Tkinter.TclError:
        sys.exit()

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass
