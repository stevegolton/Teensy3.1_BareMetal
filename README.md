# Teensy3.1_BareMetal

This project is a set of bare metal examples demonstrating how to do standard stuff on the Teensy 3.1 board's ~~Freescale~~ NXP MK20DX256VMC7 microcontroller without using the Arduino environment. Ao far this includes:
 - Blinking the LED 
 - Polled UART
 - Running FreeRTOS
 - FTM (PWM output and input capture)

TODO
 - Interrupt driven UART
 - USB CDC

This is a living project and I will attempt to add more examples as I get the time.

## Build Instructions
These build instructions have only been tested running under Ubutnu 14.04 for now but it shouldn't be too hard to fiddle with the paths in the makefile to make it work on Windows.

 - Install the gcc-arm-none-eabi toolchain with `# apt-get install gcc-arm-none-eabi` on Ubuntu, or download the [installer](https://launchpad.net/gcc-arm-embedded/+download) on Windows.
 - Install the [Teensy bootloader](https://www.pjrc.com/teensy/loader.html) (I used the command line version on Ubuntu).
 - Clone this project and navigate to one of the example directories and run `make`.
 - Follow the instructions on the teensy bootloader website above for programming the teensy from the command line which is usually something like this `sudo path/to/project/dir/teensy_loader_cli/teensy_loader_cli --mcu=mk20dx256 example.hex -w`.

## About

I started writing these examples as a result of a [quadcopter project](https://github.com/stevegolton/Teensy3.1_Quadcopter) I am concurrently working on which uses the Teensy3.1 dev board. I quickly got tired of using the Arduino environment and moved to the Kinetis Design Studio with Processor Expert which auto-generates a lot of code for you. However I wanted to learn more about arm microcontrollers from the bare-metal up, thus I cast aside the safetly net of Processor Export and started poking around under the hood, cobbling together these basic examples as I went. All information was gained by consulting freely availible information on the internet including the Kinetis K20 user manual, Kinetis software guide, squinting at extremely verbose PE auto-generated code and poking around other people's Teensy examples on the net.

~~Freescale~~ NXP does provide some code examples for the Kinetis K family however they are very broad and cover a number of microcontroller targets and are littered with redundant code and build switches. My examples are designed without this in mind in order to be a simple as possible so they can be as easy to understand as possible. As a consequence, however, they lose a lot of the flexibility of the Kinetis example code and are pretty much limited to the K20 chip, some even to the Teensy itself where specific Teensy pin names are referenced.

I took the startup code from [Karl Hunt's Bare Metal Teensy3.1 page](http://www.seanet.com/~karllunt/bareteensy31.html) which I would strongly advise reading as he examplains the startup process of the K20 extrememely well.

## Handy reference material
 - [PDF - Kinetis K20 User Guide](http://cache.nxp.com/files/32bit/doc/ref_manual/K20P121M100SF2RM.pdf?fpsp=1&WT_TYPE=Reference%20Manuals&WT_VENDOR=FREESCALE&WT_FILE_FORMAT=pdf&WT_ASSET=Documentation&fileExt=.pdf)
 - [PDF - Demonstration software for Kinetis peripheral modules](http://cache.nxp.com/files/32bit/doc/quick_ref_guide/KQRUG.pdf)
 - [PDF - Features of the FlexTimer Module](http://cache.nxp.com/files/32bit/doc/app_note/AN5142.pdf)
