#!/usr/bin/env python

from Tkinter import *
import tkMessageBox
import Tkinter as tk
import time
import rospy
from std_msgs.msg import String

rospy.init_node('GUI')

class Page(tk.Frame):
    
    def __init__(self, *args, **kwargs):
        tk.Frame.__init__(self, *args, **kwargs)
    def show(self):
        self.lift()

class Page1(Page):
        
    def __init__(self, *args, **kwargs):
        Page.__init__(self, *args, **kwargs)
        label = tk.Label(self, text=time.time()) #How do I call function and print here? 
        label.pack(side="top", fill="both", expand=True)

class Page2(Page):
   def __init__(self, *args, **kwargs):
       Page.__init__(self, *args, **kwargs)
       label = tk.Label(self, text="Insert Navigations for Directions")
       label.pack(side="top", fill="both", expand=True)

class Page3(Page):
   def __init__(self, *args, **kwargs):
       Page.__init__(self, *args, **kwargs)
       label = tk.Label(self, text="Insert Navigations for Locations")
       label.pack(side="top", fill="both", expand=True)

class Page4(Page):
   def __init__(self, *args, **kwargs):
       Page.__init__(self, *args, **kwargs)
       label = tk.Label(self, text="Insert Navigations for Camera")
       label.pack(side="top", fill="both", expand=True)
       

class Page5(Page):
   def __init__(self, *args, **kwargs):
       Page.__init__(self, *args, **kwargs)
       label = tk.Label(self, text= "hello" )
       label.pack(side="bottom", fill="both", expand=True)
       
       self.C = tk.Canvas(self, height=500, width=800, borderwidth=0, highlightthickness=0,)
       
       #C.create_oval(10,10,110,110, fill='green2')
       #C.create_text(60,60,fill="navy",text="Status1",font ="Times 20 bold")
       self.C.create_oval(10,10,110,110, fill='red')
       self.C.create_text(60,60,fill="navy",text="Status1",font ="Times 20 bold")

       self.C.create_oval(10,120,110,220, fill='green2')
       self.C.create_text(60,170,fill="navy",text="Status2",font ="Times 20 bold")
       #C.create_oval(10,120,110,220, fill='red')
       #C.create_text(60,170,fill="navy",text="Status2",font ="Times 20 bold")
       
       self.C.create_oval(10,230,110,330, fill='green2')
       self.C.create_text(60,280,fill="navy",text="Status3",font ="Times 20 bold")
       #C.create_oval(10,230,110,330, fill='red')
       #C.create_text(60,280,fill="navy",text="Status3",font ="Times 20 bold")
       
       self.C.create_oval(10,340,110,440, fill='green2')
       self.C.create_text(60,390,fill="navy",text="Status4",font ="Times 20 bold")
       #C.create_oval(10,340,110,440, fill='red')
       #C.create_text(60,390,fill="navy",text="Status4",font ="Times 20 bold")

       self.C.pack(side="left", fill = "both",expand =True)
       rospy.Subscriber("test", String, self.USB)
       

   def USB(self, msg):
       print "We got data!"
       self.C.create_oval(10,10,110,110, fill='green2')
       self.C.create_text(60,60,fill="navy",text="Status1",font ="Times 20 bold")
       

class Page6(Page):
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

        buttonframe = tk.Frame(self)
        container = tk.Frame(self)
        buttonframe.pack(side="top", fill="x", expand=False)
        container.pack(side="top", fill="both", expand=True)

        p1.place(in_=container, x=0, y=0, relwidth=0.5, relheight=0.5) #relheight is to change the height of the information that will be placed on the screen; while relwidth is shifting to the right
        p2.place(in_=container, x=400, y=0, relwidth=0.5, relheight=0.5)
        p3.place(in_=container, x=0, y=200, relwidth=0.5, relheight=0.5)
        p4.place(in_=container, x=400, y=200, relwidth=0.5, relheight=0.5)
        p5.place(in_=container, x=0, y=0, relwidth=1.5, relheight=1.5)
        p6.place(in_=container, x=0, y=0, relwidth=1.5, relheight=1.5)

        b1 = tk.Button(buttonframe, text="Time", command=p1.lift)
        b2 = tk.Button(buttonframe, text="Directions", command=p2.lift)
        b3 = tk.Button(buttonframe, text="Locations", command=p3.lift)
        b4 = tk.Button(buttonframe, text="Camera", command=p4.lift)
        b5 = tk.Button(buttonframe, text="status", command=p5.lift)
        b6 = tk.Button(buttonframe, text="Clear", command=p6.lift)

        b1.pack(side=LEFT)
        b2.pack(side=LEFT)
        b3.pack(side=LEFT)
        b4.pack(side=LEFT)
        b5.pack(side=LEFT)
        b6.pack(side=LEFT)

        p1.show()

if __name__ == "__main__":
    root = tk.Tk()
    main = MainView(root)
    main.pack(side="top", fill="both", expand=True)
    root.wm_geometry("800x500")
    root.mainloop()


