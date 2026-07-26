Water Controller rewritten using an intergrated ESP32-C6-LCD-1.9-TOUCH from Waveshare.  
This coupled with a custom PCB allows me to plug in the radio, the button controllers, and
the relay with almost no wires.  Note we are using the ESP32-C6 for wifi so didn't have to add a wifi device.

Case will be custom build 


  iiiiiiiiiiiiiiiiiiii
  ii  TEXT          ii
  iiiiiiiiiiiiiiiiiiii

       ^       v
     Raise   Lower

       X       O 
    Freeze   Reset


       (--Outoupt-)
        ||  |  ||
        ||  |  ||
   Power||     ||Relay
