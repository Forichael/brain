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
        self.newlabel = tk.Label(self,text="")
        self.CV = tk.Canvas(self, height=500, width=700, borderwidth=0, highlightthickness=0)
        # rospy.Subscriber(.....)
        self.trajectory = []
        rospy.Subscriber("/odometry/filtered/local", Odometry, self.Pos)

        #drawgrid
        for a in range(15):
            self.CV.create_line(50*a, 0, 50*a,500  ) #vertical line
            self.CV.create_line(0, 50*a, 700, 50*a) #horizontal line
            self.CV.pack(side="left", fill = "both",expand =True)
            a=a+1
    
   def Pos(self, msg): 
        self.newlabel.pack_forget()
        self.newlabel = tk.Label(self, text= str(msg.pose.pose), anchor=W)
        self.newlabel.pack(side="right", fill="both", expand=True)
        i1 = msg.pose.pose.position.x
        j1 = msg.pose.pose.position.y

        self.trajectory.append((i1,j1))
        
        if len(self.trajectory) > 1:
            
            # DRAW ALL THE OLD TRAJECTORY
            for src,dst in zip(self.trajectory[:-1], self.trajectory[1:]):
                srci, srcj = src
                dsti, dstj = dst
                self.CV.create_line(srci, srcj, dsti, dstj, fill="red", width=2)
                
            # OVERDRAW NEWEST WITH DIFFERENT COLOR
            srci, srcj = self.trajectory[-2]
            dsti, dstj = self.trajectory[-1]
            self.CV.create_line(srci, srcj, dsti, dstj, fill="green2", width=2)
     
##                    
##           
##        #draw direction
##        i=350
##        j=0
##        iold=0
##        jold=0
##        ishift=0
##        jshift=0
##        new = 0
##        old = 0
##
##        prv_i = 0
##        prv_j = 0
##        
##        while(i>0, i<700, j>0, j<500):
##            
##            ishiftold=ishift
##            jshiftold=jshift
##            print (iold,jold,i+ishiftold,j+jshiftold)            
##            
##            time.sleep(7)
##            i2 =msg.pose.pose.position.x
##            j2 = msg.pose.pose.position.y                       
##            print (iold,jold,i+ishiftold,j+jshiftold, i1,i2, j1,j2)
##            
##            if (new>old):               
##                    self.CV.create_line(iold, jold, iold+ishiftold, jold+jshiftold, fill="red", width=2)
##                    
##                    #produce new starting point
##                    i=i+ishift
##                    j=j+jshift
##                    print(old, i, j)
##                        
##            iold=i
##            jold=j
##            #ishift = input ('ishift=')#
##            ishift = i2-i1
##            #jshift = input('jshift=')#
##            jshift = j2-j1
##            new=new+1
##            self.CV.create_line(i, j, i+ishift, j+jshift, fill="green2", width=2)
##            print(i , j, i+ishift, j+jshift)
##            time.sleep(2)
##            
        
class Page2(Page): #USB hub - IMU 
    
   def __init__(self, *args, **kwargs):
       Page.__init__(self, *args, **kwargs)
       label = tk.Label(self, text="IMU DATA",font ="Times 20 bold")
       label.pack(side="top", fill="both", expand=True)
       self.newlabel = tk.Label(self,text="")
       rospy.Subscriber("/imu/data", Imu, self.USB)
       
   def USB(self, msg):
        self.newlabel.pack_forget()
        self.newlabel = tk.Label(self, text= str(msg))
        self.newlabel.pack(side="right", fill="both", expand=True)

class Page3(Page): #Arduino - Wheels 
   def __init__(self, *args, **kwargs):
       Page.__init__(self, *args, **kwargs)
       label = tk.Label(self, text="ARDUINO COMMUNICATION",font ="Times 20 bold")
       label.pack(side="top", fill="both", expand=True)
       self.newlabel = tk.Label(self,text="")

       rospy.Subscriber("/rwheel", Int16, self.wheels)

   def wheels(self, msg): 
        self.newlabel.pack_forget()
        self.newlabel = tk.Label(self, text="Data:" + str(msg.data), font ="Times 25 bold") 
        self.newlabel.pack(side="right", fill="both", expand=True)


class Page4(Page): #Lidar - laserscan 
   def __init__(self, *args, **kwargs):
       Page.__init__(self, *args, **kwargs)
       label = tk.Label(self, text="LIDAR DATA",font ="Times 20 bold")
       label.pack(side="top", fill="both", expand=True)
       self.newlabel = tk.Label(self,text="")
       rospy.Subscriber("/scan", LaserScan, self.Lidar)
       
   def Lidar(self, msg):
       self.newlabel.pack_forget()
       self.newlabel = tk.Label(self, text= str(msg)) 
       self.newlabel.pack(side="right", fill="both", expand=True)
        
class Page5(Page):
   def __init__(self, *args, **kwargs):
       Page.__init__(self, *args, **kwargs)
       label = tk.Label(self, text= "" )
       label.pack(side="bottom", fill="both", expand=True)
       
       self.C = tk.Canvas(self, height=500, width=800, borderwidth=0, highlightthickness=0)
       
       
       self.C.create_oval(20,10,260,250, fill='red')
       self.C.create_text(140,130,fill="navy",text="STATE",font ="Times 30 bold")

       self.C.create_oval(300,10,540,250, fill='red')
       self.C.create_text(420,130,fill="navy",text="USB",font ="Times 30 bold")
       
       self.C.create_oval(20,258,260,498, fill='red')
       self.C.create_text(140,378,fill="navy",text="WHEELS",font ="Times 30 bold")
       
       self.C.create_oval(300,258,540,498, fill='red')
       self.C.create_text(420,378,fill="navy",text="LIDAR",font ="Times 30 bold")

       self.C.pack(side="left", fill = "both",expand =True)
       
       rospy.Subscriber("/states", String, self.states)
       rospy.Subscriber("/imu/data", Imu, self.USB)
       rospy.Subscriber("/rwheel", Int16, self.wheels) #/rhweel is the topic name
       rospy.Subscriber("/scan", LaserScan, self.Lidar)
       

   def states(self, msg):
       print "STATE"
       self.C.create_oval(20,10,260,250, fill='green2')
       self.C.create_text(140,130,fill="navy",text="STATE",font ="Times 30 bold")

   def USB(self, msg):
       
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
       label = tk.Label(self, text="STATE",font ="Times 20 bold")
       label.pack(side="top", fill="both", expand=False)
       self.state = tk.Label(self,text="")
       self.MSG = tk.Label(self,text="")
       self.MC = tk.Label(self,text="")
       
       rospy.Subscriber("/we_got_the_can", Bool, self.gotthecan)
       rospy.Subscriber("/mission_complete", Bool, self.mission_complete)
       rospy.Subscriber("/states", String, self.states)

    def states(self, msg):
       self.state.pack_forget()
       self.state = tk.Label(self, text= "Current state: " +str(msg.data),font ="Times 15 bold")
       self.state.pack(side="top", fill="both", expand=False)

    def gotthecan(self, msg):        
        if ( msg.data == 1 ):
            self.MSG = tk.Label(self, text="WE GOT THE CAN!",font ="Times 80 bold", background="green2", foreground="red")
            self.MSG.pack(side="top", fill="both", expand=True, anchor=NW)
            self.delay()

    def mission_complete(self, msg):        
        if ( msg.data == 1 ):
            self.MC = tk.Label(self, text="MISSION COMPLETE!",font ="Times 75 bold", background="black", foreground="white")
            self.MC.pack(side="top", fill="both", expand=True, anchor=NW)
            self.delay()    
            
            
    def delay(self):
        self.MSG.after(5000, self.clear_label)
        self.MC.after(5000, self.clear_label)
        
    def clear_label(self):
        self.MSG.pack_forget()
        self.MC.pack_forget()

class Page7(Page):
   def __init__(self, *args, **kwargs):
       Page.__init__(self, *args, **kwargs)
       label = tk.Label(self, text="")
       label.pack(side="top", fill="both", expand=True)
      
       
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
        p6.place(in_=container, x=0, y=0, relwidth=1, relheight=1)
        p7.place(in_=container, x=0, y=0, relwidth=1.5, relheight=1.75)

#buttons
        b1 = tk.Button(buttonframe, text="LOCALIZATION", font ="Times 10 bold", command=p1.lift)
        b2 = tk.Button(buttonframe, text="IMU DATA",font ="Times 10 bold", command=p2.lift)
        b3 = tk.Button(buttonframe, text="ARDUINO", font ="Times 10 bold", command=p3.lift)
        b4 = tk.Button(buttonframe, text="LIDAR", font ="Times 10 bold", command=p4.lift)
        b5 = tk.Button(buttonframe, text="STATUS CHECK", font ="Times 10 bold", command=p5.lift)
        b6 = tk.Button(buttonframe, text="STATE", font ="Times 10 bold", command=p6.lift)
        b7 = tk.Button(buttonframe, text="CLEAR", font ="Times 10 bold", command=p7.lift)

        b1.pack(side=LEFT)
        b2.pack(side=LEFT)
        b3.pack(side=LEFT)
        b4.pack(side=LEFT)
        b5.pack(side=LEFT)
        b6.pack(side=LEFT)
        b7.pack(side=LEFT)

        p1.show()
        


        
if __name__ == "__main__":
    root = tk.Tk()
    main = MainView(root)
    main.pack(side="top", fill="both", expand=True)
    root.wm_geometry("1100x600")
    root.mainloop()


