This is a gadget which improves the temperature control of a Wolf electric oven. The 
model we use it for is the DF486G, but it would likely work for other similar models. 

The problem is that with the standard control algorithm, the temperature cycles up 
and down by as much as 20 or 30 degrees Fahrenheit. This controller maintains the 
temperature to within 5 or 10 degrees Fahrenheit. 

The overall design 

The Wolf control knob for the oven mode and temperature consists of (1) an outer 
selector ring that is rotated to set the mode, (2) a left-right toggle to adjust the 
temperature, and (3) a 3-character LED display. It is connected behind the oven 
"bullnose" front plate to the "ECH" control board with a 10-conductor cable. 

Our small (2.5" x 1.5") control board snaps onto the back of the control knob in 
place of that cable. The cable is then plugged into our board instead. That give us 
access to the four signals that indicate the currently selected mode, the two signals 
that transmit the left-right toggle information, and the two signals that control the 
LED display. 

We also need to read the real RTD temperature sensor in the oven, and we need to 
control the oven heating element by "faking" the temperature that the Wolf oven 
controller sees. To do that we cut the wires from the temperature sensor to the ECH 
board, and run all four ends to a four-conductor cable connected to our board. 

To allow software updates without removing the bullnose front plate (which is hard) 
to get access to the back of the knob, a small board with a programming connector is 
mounted to one of the vents on the front plate, and connected to our board with a 
6-conductor cable. 

In many modes we do nothing: the temperature sensor, up-down toggle, the mode 
selection, and the display are all "passed through" to the Wolf ECH control board. 

In the modes where we control the temperature (currently "bake", "convection bake", 
"roast", and "convection roast"), we do the following: 
 - read changes to up-down toggle switch, but do not transmit them to the oven
 - read the current temperature sensor, but do not transmit it to the oven
 - fake a "very low" (125F) current temperature to the oven when we want the heating on
 - fake a "very high" (620F) current temperature to the oven when we want the heating off
 - ignore the display data the oven is trying to show on the knob
 - generate our own display data for the knob
 
We implement several different temperature control algorithms. The default is that 
used by PID (proportional-integral-differential) controllers; see 
https://en.wikipedia.org/wiki/PID_controller. 

The choice of algorithm, and many other parameters, can be set by using the knob to 
enter a special configuration mode; see the user-interface document for details. The 
controller also maintains a temperature log, which can be exported as serial data 
either to the programming connector or to the serial data line of the knob's display. 

For more details, see the controller source code and the schematic.

Len Shustek, April 2021
