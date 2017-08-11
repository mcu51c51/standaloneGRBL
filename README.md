# Standalone GRBL
An open source, embedded, high performance CNC laser cutting and engraving controller written in optimized C that will run on a straight Arduino. This version only for laser cutters and engravers, and it can work without a PC.

### SCHEMATIC
![schematic](/uploads/schematic.jpg?raw=true)

### BUTTONS
**LEFT**, **RIGHT**, **UP** and **DOWN** allows you to manually move the machine, the motion will continue as long as the button is pressed. Otherwise, the axis will move exactly 0.1 mm each time the button is pressed.

**RUN/PAUSE** allows you to open a U-disk file from the SD card. Also this button pauses a program and resumes running from a pause.

**LASER** turns on/off laser.

**XY-0** moves the laser to the origin. Hold to set the origin.

**HOME** will home all axes, if your machine has home switches. Otherwise, this button does nothing.

**STOP** stops a running program. Also this button turns off the laser.

### DOWNLOADS
[standaloneGRBL.zip](https://yadi.sk/d/7bHaRurQ3LtvVW)

***
If you found this software useful, make a small donation: [PayPal](https://www.paypal.me/runnybore604) / [Yandex.Money](https://money.yandex.ru/to/410013968246480)
