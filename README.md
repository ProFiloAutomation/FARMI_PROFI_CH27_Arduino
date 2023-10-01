# FARMI_PROFI_CH27_Arduino
This repository contains the Arduino Code I have implemented to replace the original machine controller (Farmi own product PLC)

SUMMER 2023
The original Farmi Profi CH27 wood chipper machine controller didn't work anymore. 
No proper control on the feeding direction fo the ACC crane feeder. It was going only backward, and it wasn't responding anymore to the commands.
Neither radio-commands, nor manual-switch command.

I wasn't able to restore the original software/behaviour, even if I tried to follow the software-reprogramming manual.

Farmi gave me a reply, tleling us that they were able to sell us a new controller for almost 700€, way too much for us.

After a bit of reverse engineering, I decided to create my own controller using a Arduino/Arduino Pro mini, in the second version of the project.

What are the needs:
-  2x digital outputs --> 2x 12V electrovalves for forward/backward direction of the feeder chains.
-  2x digital inputs --> radio control commands, forward and backward direction
-  1x analog input --> reading the rotation speed of the machine. This is implemented via a 12V inductor sensor (active low) to measure the rotation speed.

The final control system has to implement:
-  forwarding/backwarding direction of the feeding chains according to the radio command
-  no-stress system: depending on the rotation speed of the machine, the material feeding has to be properly contrlled to avoid mechanical stresses on the tractor engine.

Additional:
-   I have used an oscilloscope to properly evaluate the speed signal (square wave, active low) coming from the inductor, together with a digital pin toggling (HIGH/LOW)
    to evaluate the processing duration of the Arduino/Arduino Pro Mini board (and compare it to the signal duration).

