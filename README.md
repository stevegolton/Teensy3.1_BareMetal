# Teensy3.1_BareMetal

This project is a set of examples demonstrating how to code a whole bunch of standard embedded stuff on the Teensy 3.1 board without making use of the Arduino libraries. The Teensy 3.1 features a ~~Freescale~~ NXP MK20DX256VMC7 ARM Cortex M4 microcontroller with a lot of the useful IO broken out into a tiny breadboard compatible dev board.

## Examples
Examples included so far:
 - Blinking the on-board LED 
 - Polled UART
 - FTM PWM output
 - FTM input capture
 - FreeRTOS (basic set up, timers, tasks, etc)

TODO examples:
 - Interrupt driven UART
 - USB CDC
 - FlexRAM
 - I2C

This is a living project and I will attempt to add more examples as I get the time.

## Build Instructions
These build instructions have only been tested running under Ubutnu 14.04 for now but it shouldn't be too hard to fiddle with the paths in the makefile to make it work on Windows.

 - Install the gcc-arm-none-eabi toolchain with `# apt-get install gcc-arm-none-eabi` on Ubuntu, or download the [installer](https://launchpad.net/gcc-arm-embedded/+download) on Windows.
 - Install the [Teensy bootloader](https://www.pjrc.com/teensy/loader.html) (I used the command line version on Ubuntu).
 - Clone this project and navigate to one of the example directories and run `make`.
 - Follow the instructions on the teensy bootloader website above for programming the teensy from the command line which is usually something like this `sudo path/to/project/dir/teensy_loader_cli/teensy_loader_cli --mcu=mk20dx256 example.hex -w`.

## About

I started writing these examples as a result of a [quadcopter project](https://github.com/stevegolton/Teensy3.1_Quadcopter) I am concurrently working on which uses the Teensy3.1 dev board. I quickly got tired of using the Arduino environment and moved to the Kinetis Design Studio with Processor Expert which auto-generates a lot of code for you. However I wanted to learn more about programming for Arm microcontrollers from the bare-metal up, thus I cast aside the safety net of Processor Export and started poking around under the hood, cobbling together these basic examples as I went. All information was gained by consulting freely availible information on the internet including the Kinetis K20 user manual, Kinetis software guide, squinting at extremely verbose PE auto-generated code and poking around other people's Teensy and Kinetis examples on the net.

~~Freescale~~ NXP does provide some code examples for the Kinetis K family however they are very broad and cover a number of microcontroller targets and are littered with redundant code and build switches. Where possible my examples are designed with simplicity in mind in order to be as easy to understand as possible. As a consequence, however, they lose a lot of the flexibility of the Kinetis example code and are pretty much limited to the K20 chip, some even to the Teensy itself where specific Teensy pin names are referenced.

I took the startup code from [Karl Hunt's Bare Metal Teensy3.1 page](http://www.seanet.com/~karllunt/bareteensy31.html) which I would strongly advise reading as he explains the startup process of the K20 extremely well.

## Handy reference material
 - [PDF - Kinetis K20 User Guide](http://cache.nxp.com/files/32bit/doc/ref_manual/K20P121M100SF2RM.pdf?fpsp=1&WT_TYPE=Reference%20Manuals&WT_VENDOR=FREESCALE&WT_FILE_FORMAT=pdf&WT_ASSET=Documentation&fileExt=.pdf)
 - [PDF - Demonstration software for Kinetis peripheral modules](http://cache.nxp.com/files/32bit/doc/quick_ref_guide/KQRUG.pdf)
 - [PDF - Features of the FlexTimer Module](http://cache.nxp.com/files/32bit/doc/app_note/AN5142.pdf)
 - [Teensy schematic & pin routing](https://www.pjrc.com/teensy/schematic.html) - p233 for multiplexing and pin assignments, p70 for interrupt vectors
 - [Teensy pinouts](https://www.pjrc.com/teensy/pinout.html)
 - [Karl Hunt's Bare Metal Teensy3.1 page](http://www.seanet.com/~karllunt/bareteensy31.html)
 - [Freescale guide to I2C signalling](https://community.freescale.com/docs/DOC-1034#jive_content_id_Introduction_to_I2C_signaling)
 - [PDF - Application Note: Using the I2C device on Coldfire and Kinetis](http://cache.nxp.com/files/analog/doc/app_note/AN4342.pdf)
