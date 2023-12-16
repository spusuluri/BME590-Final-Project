.. _blinky-sample:

=================
BME 590 Final Project
=================

By: Srisatya Pusuluri

Purpose of Project
#####
The purpose of this project is to demonstrate proficiency in using nrf52833 development kit.
The project that two analogs signals into the microcontroller that control the brightness of LEDs
based on the peak to peak voltage of the analog signals. The two analog signals are read in with
timers to sample the sinusoidal signals at a certain interval calculated with the Nyquist–Shannon sampling theorem
as well as using the ADC channels on the board.The brightness of the LEDs are changed using the PWM module in the
microcontroller. Furthermore, there is a DC voltage sent into the microcontroller which is used to
calculate battery level. Timers in the microcontroller are used to cease function of the controller when
the controller is charging as well as to collect the peak to peak voltage in a 5 second interval.
Finally, the bluetooth module is used to send this data to a mobile device. Buttons are used in the program
to allow for the collection of the data as well as the bluetooth sending of the data. Please refer to the block
diagram to look at the inputs and outputs of the microcontroller in this project. Also, please refer to
the testing analysis of the project to learn more.


Project Configuration and Overlay
#####
The following packages are configured in this project: 

* `GPIO`
* `ADC`
* `LOG`
* `PWM`
* `BT`

Please refer to the `prj.conf` file for the exact syntax.

The overlay file outlines the different ADC channels and creates the PWM control of the board.
Please refer to the `nrf52833dk_nrf52833.overlay` file for the exact syntax.


How to Run Program
#####
First, make sure to the have the board and a cable to connect your computer to the board.
In addition, make sure to download the Nordic App in order to connect your phone to the board.
Second, download the project from Github onto your local computer.
Third, make sure to create a build on your computer to **flash** to the board.
Fourth, **flash** the board with the build.
Then, make sure to send the correct signals to the pin numbers as shown in the overlay file.
When you want to send information to the mobile device, make sure to press the buttons
specified in the diagram. Please note that the data is sent as a single line in hexadecimal.
You must convert those numbers to decimal to get the correct peak to peak voltages.