#!/usr/bin/env python

from Tkinter import *
import tkMessageBox
import Tkinter as tk
import time
import rospy
from std_msgs.msg import String, Int16, Int64, Bool
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry


rospy.init_node('GUI')

class Page(tk.Frame):
    
    def __init__(self, *args, **kwargs):
        tk.Frame.__init__(self, *args, **kwargs)
        
    def show(self):
        self.lift()

class Page1(Page): #Localization - Position 
        
   def __init__(self, *args, **kwargs):        
        Page.__init__(self, *args, **kwargs)
        label = tk.Label(self, text="LOCALIZATION",font ="Times 20 bold")
        label.pack(side="top", fill="both", expand=True)

        # rospy.Subscriber(.....)
        rospy.Subscriber("/odometry/filtered", Odometry, self.Pos)
        
    
   def Pos(self, msg):
        label = tk.Label(self, text= Odometry(), anchor=W) 
        label.pack(side="right", fill="both", expand=False)
        

class Page2(Page): #USB hub - IMU 
    
   def __init__(self, *args, **kwargs):
       Page.__init__(self, *args, **kwargs)
       label = tk.Label(self, text="IMU DATA",font ="Times 20 bold")
       label.pack(side="top", fill="both", expand=True)
       rospy.Subscriber("/imu/data", Imu, self.USB)
       
   def USB(self, *args, **kwargs):
        label = tk.Label(self, text=Imu()) 
        label.pack(side="right", fill="both", expand=True)

class Page3(Page): #Arduino - Wheels 
   def __init__(self, *args, **kwargs):
       Page.__init__(self, *args, **kwargs)
       label = tk.Label(self, text="ARDUINO COMMUNICATION",font ="Times 20 bold")
       label.pack(side="top", fill="both", expand=True)
       rospy.Subscriber("/rwheel", Int16, self.wheels)

   def wheels(self, *args, **kwargs):
        label = tk.Label(self, text= Int16()) 
        label.pack(side="right", fill="both", expand=True)


class Page4(Page): #Lidar - laserscan 
   def __init__(self, *args, **kwargs):
       Page.__init__(self, *args, **kwargs)
       label = tk.Label(self, text="LIDAR DATA",font ="Times 20 bold")
       label.pack(side="top", fill="both", expand=True)
       rospy.Subscriber("/scan", LaserScan, self.Lidar)
       
   def Lidar(self, *args, **kwargs):
       label = tk.Label(self, text= LaserScan()) 
       label.pack(side="right", fill="both", expand=True)
        
class Page5(Page):
   def __init__(self, *args, **kwargs):
       Page.__init__(self, *args, **kwargs)
       label = tk.Label(self, text= "" )
       label.pack(side="bottom", fill="both", expand=True)
       
       self.C = tk.Canvas(self, height=500, width=800, borderwidth=0, highlightthickness=0,)
       
       
       self.C.create_oval(20,10,260,250, fill='red')
       self.C.create_text(140,130,fill="navy",text="POSITION",font ="Times 30 bold")

       self.C.create_oval(300,10,540,250, fill='red')
       self.C.create_text(420,130,fill="navy",text="USB",font ="Times 30 bold")
       
       self.C.create_oval(20,258,260,498, fill='red')
       self.C.create_text(140,378,fill="navy",text="WHEELS",font ="Times 30 bold")
       
       self.C.create_oval(300,258,540,498, fill='red')
       self.C.create_text(420,378,fill="navy",text="LIDAR",font ="Times 30 bold")

       self.C.pack(side="left", fill = "both",expand =True)
       
       rospy.Subscriber("/odometry/filtered", Odometry, self.Localization)
       rospy.Subscriber("/imu/data", Imu, self.USB)
       rospy.Subscriber("/rwheel", Int16, self.wheels) #/rhweel is the topic name
       rospy.Subscriber("/scan", LaserScan, self.Lidar)
       

   def Localization(self, msg):
       print "position"
       self.C.create_oval(20,10,260,250, fill='green2')
       self.C.create_text(140,130,fill="navy",text="POSITION",font ="Times 30 bold")

   def USB(self, msg):
       #msg.data
       print "We got data!"
       self.C.create_oval(300,10,540,250, fill='green2')
       self.C.create_text(420,130,fill="navy",text="USB", font ="Times 30 bold")

   def wheels(self, msg):
       print "wheels!"
       self.C.create_oval(20,258,260,498, fill='green2')
       self.C.create_text(140,378,fill="navy",text="WHEELS",font ="Times 30 bold")
     
   def Lidar(self, msg):
       print "lidar!"
       self.C.create_oval(300,258,540,498, fill='green2')
       self.C.create_text(420,378,fill="navy",text="LIDAR",font ="Times 30 bold")

   
class Page6(Page):
   def __init__(self, *args, **kwargs):
       Page.__init__(self, *args, **kwargs)
       label = tk.Label(self, text="Clear")
       label.pack(side="top", fill="both", expand=True)
       
       
class Page7(Page): #Blink when we got the can -> How to check when the robot grips the can?
    
    def __init__(self, *args, **kwargs):
       Page.__init__(self, *args, **kwargs)
       label = tk.Label(self, text="STATE",font ="Times 20 bold")
       label.pack(side="top", fill="both", expand=False)
       rospy.Subscriber("/states", String, self.states)
       rospy.Subscriber("/we_got_the_can", Bool, self.gotthecan)

    def states(self, msg):
       label = tk.Label(self, text= String())
       label.pack(side="top", fill="both", expand=False)
       
    def gotthecan(self, parent):

       #tk.Frame.__init__(self, parent)
       self.label = tk.Label(self, text="WE GOT THE CAN!",font ="Times 80 bold", background="red", foreground="green2")
       self.label.pack(side="top", fill="both", expand=True, anchor=NW)
       self.flash()
       
            
    def flash(self):
        bg = self.label.cget("background")
        fg = self.label.cget("foreground")
        self.label.configure(background=fg, foreground=bg)
        self.after(700, self.flash)
        
        #i need a delay (5 secs) before clearing it #1000=1s
        self.label.after(5000, self.clear_label)

    def clear_label(self):
        self.label.pack_forget()
           
class MainView(tk.Frame):
    def __init__(self, *args, **kwargs):
        tk.Frame.__init__(self, *args, **kwargs)
        p1 = Page1(self)
        p2 = Page2(self)
        p3 = Page3(self)
        p4 = Page4(self)
        p5 = Page5(self)
        p6 = Page6(self)
        p7 = Page7(self)

        buttonframe = tk.Frame(self)
        container = tk.Frame(self)
        buttonframe.pack(side="top", fill="x", expand=False)
        container.pack(side="top", fill="both", expand=True)

        p1.place(in_=container, x=0, y=0, relwidth=1, relheight=1) #relheight is to change the height of the information that will be placed on the screen; while relwidth is shifting to the right
        p2.place(in_=container, x=0, y=0, relwidth=1, relheight=1)
        p3.place(in_=container, x=0, y=0, relwidth=1, relheight=1)
        p4.place(in_=container, x=0, y=0, relwidth=1, relheight=1)
        p5.place(in_=container, x=0, y=0, relwidth=1.5, relheight=1.75)
        p6.place(in_=container, x=0, y=0, relwidth=1.5, relheight=1.75)
        p7.place(in_=container, x=0, y=0, relwidth=1, relheight=1)

#buttons
        b1 = tk.Button(buttonframe, text="LOCALIZATION", command=p1.lift)
        b2 = tk.Button(buttonframe, text="USB HUB", command=p2.lift)
        b3 = tk.Button(buttonframe, text="ARDUINO", command=p3.lift)
        b4 = tk.Button(buttonframe, text="LIDAR", command=p4.lift)
        b5 = tk.Button(buttonframe, text="STATUS CHECK", command=p5.lift)
        b6 = tk.Button(buttonframe, text="CLEAR", command=p6.lift)
        b7 = tk.Button(buttonframe, text="STATES", command=p7.lift)

        b1.pack(side=LEFT)
        b2.pack(side=LEFT)
        b3.pack(side=LEFT)
        b4.pack(side=LEFT)
        b5.pack(side=LEFT)
        b6.pack(side=LEFT)
        b7.pack(side=LEFT)

        p7.show()
        


        
if __name__ == "__main__":
    root = tk.Tk()
    main = MainView(root)
    main.pack(side="top", fill="both", expand=True)
    root.wm_geometry("1100x600")
    root.mainloop()


