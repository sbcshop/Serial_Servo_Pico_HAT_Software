# Serial_Servo_Pico_HAT_Software

<img src= "https://cdn.shopify.com/s/files/1/1217/2104/files/Artboard_1_1.png?v=1718781840" />

The Pico Serial Servo HAT is a versatile and compact HAT for Pico/Pico W microcontroller with dual power options, two serial servo connectors, and two programmable buttons. It features a 1.14” TFT display for status updates, a power LED indicator, and supports simultaneous multi-servo control, making it ideal for projects requiring precise coordination and multiple servos with minimal wiring like Robotic Arm Control, CNC machine and Automation.

This github provides getting started instructions for Serial Servo Pico HAT.

### Features
- Pico Serial Servo compatible Hat for Pico and Pico W board
- TFT 1.14” display for user interactions.
- Two slots to connect Serial Servo Motors, and easily cascade servo to connect more motors. Allows controlling 1-253 serial servos at the same time*
- Two Programmable Buttons to add additional controls to project 
- Header 2.54” and DC jack options to connect 6-8.4V adapter with onboard regulator 
- Power status LED to indicate board power.
- Compact and space-saving design
- Compatible Servo Motors =>
    - [SB-SS023](https://shop.sb-components.co.uk/products/sb-serial-servo-sb-ss023-powerful-multi-purpose-digital-servo-motor?_pos=1&_sid=5cba75e00&_ss=r) - For Lightweight Projects
    - [SB-SS15](https://shop.sb-components.co.uk/products/sb-serial-servo-sb-ss15-powerful-multi-purpose-digital-servo-motor?_pos=2&_sid=5cba75e00&_ss=r) - For Heavier Applications
    - Servo Motor Key Features:
      - Real-Time Position, Load, Temperature, Speed, and Voltage feedback.
      - Servo/Motor Mode Switchable
      - High Precision And Large Torque
      - ID Range 1~253
      - 38400 bps ~ 1Mbps (1Mbps by default)

***NOTE:  Avoid Connecting More Than 6 Servos At A Time, Not Recommended Due To High Current Demand By Servos.**

For more details about Serial Servo Motor checkout [Manual](https://github.com/sbcshop/Serial_Servo_Breakout_Software/blob/main/Documents/SB_Servo_User_Manual.pdf).  

## Specification:
- **Microcontroller**: Supports Raspberry Pi Pico/Pico W
- **Board Supply Voltage**: 5V 
- **Operating Pin Voltage**: 3.3V 
- **Operating Servo voltage**: 6~8.4V 
- **Display Size**: 1.14” 
- **Display Resolution** : 240x320 pixels
- **Display Driver**: ST7789 
- **Display Appearance**: RGB, 65K/262K
- **Temperature Range**: -20°C ~ +70°C

## Getting Started with Serial Servo Pico HAT
### Pinout
<img src= "https://cdn.shopify.com/s/files/1/1217/2104/files/Seial_servo_Pico_Hat.png?v=1718861233" />

- (1) Power Status LED
- (2) Header 2.54” input (6~8.4V DC)
- (3) DC Jack input (6~8.4V DC)
- (4) & (5)  Programmable Buttons
- (6) & (7) Serial Servo Connectors
- (8) TFT 1.14” Display
- (9) Pico Header Support
  
### Interfacing Details

<img src= "https://github.com/sbcshop/Serial_Servo_Pico_HAT_Software/blob/main/images/Pico_Serial_Servo_HAT.png" width="376" height="285"/>

When HAT is connected with Pico/Pico W following pins interfaced with onboard hardware components,
 - _Serial Servo Bus Pins:_
   * Servo connector having +ve[6~8.4VDC], -ve[GND] and Signal pin. 
   * Serial Servo Signal pins breakout into UART RXD and TXD to connect with PICO UART pins,
     
     | Pico | Servo | Description | 
     |---|---|---|
     | TXD0/GPIO43 | Servo Bus RXD | UART communication pin |
     | RXD0/GPIO44 | Servo Bus TXD | UART communication pin |
  
- _Display interfacing with Pico_
    | Pico | Display | Function |
    |---|---|---|
    | GP10 | LCD_CLK | Clock pin of SPI interface for Display|
    | GP11 | LCD_DIN | MOSI (Master OUT Slave IN) pin of SPI interface|
    | GP09 | LCD_CS | Chip Select pin of SPI interface|
    | GP08 | LCD_DC| Data/Command (MISO) pin of SPI interface|
    | GP12 | LCD_RST | Display Reset pin |
    | GP13 | BL | Backlight of display|
  
- _Buttons Interfacing_
    | Pico | Hardware | Function |
    |---|---|---|
    | GP14 | BT1 | Programmable Button |
    | GP15 | BT2 | Programmable Button |

### 1. How to Install Boot Firmware in Pico/Pico W of HAT 

- If you already have MicroPython firmware with the inbuilt ST7789 module, then you can skip this step and jump to **step 2** for trying demo codes.
- In case, you need to add **MicroPython firmware** in Pico of HAT. First, you need to *Press and Hold* the boot button on pico W, and then, without releasing the button, connect it to PC/laptop using micro USB cable. Check below image for reference,
  
  <img src="https://github.com/sbcshop/ArdiPi_Software/blob/main/images/pico_bootmode.gif" width="340" height="228">

- Now your device is in boot mode, and you will see a new mass storage device named "RPI-RP2" as shown in the below figure.
  <img src= "https://github.com/sbcshop/PiCoder-Software/blob/main/images/RPI_folder.jpg" width="720" height="360"/>

- Download the MicroPython firmware file provided in this repo above as ["**_firmware.uf2_**"](https://github.com/sbcshop/Serial_Servo_Pico_HAT_Software/blob/main/firmware.uf2). Drag and drop this **_firmware.uf2_** file onto the RPI-RP2 volume.

  <img src= "https://github.com/sbcshop/Serial_Servo_Pico_HAT_Software/blob/main/images/firmware_install.png" width="740" height="463">
  
### 2. Running First Program
   - Download **Thonny IDE** from [Download link](https://thonny.org/) as per your OS and install it.
   - Download this github which contains various examples and open anyone of example in Thonny.

     <img src= "https://github.com/sbcshop/Serial_Servo_Pico_HAT_Software/blob/main/images/github_download.png" width="748" height="336" />

   - Now we have **Thonny IDE application** and github example codes, Connect hardware to laptop/PC. Open any example code in Thonny IDE. Then select micropython device at the bottom right with a suitable COM port, as shown in the below figure. You might get a different COM port.

     <img src="https://github.com/sbcshop/Serial_Servo_Pico_HAT_Software/blob/main/images/board_select.png">
  
   - Make sure to save _**SerialServo.py**_ library file to device to avoid any execution error.

      <img src= "https://github.com/sbcshop/Serial_Servo_Pico_HAT_Software/blob/main/images/servo_lib.png" />

   - Once everything all set, with any demo code open click on green play button to test program.

     <img src= "https://github.com/sbcshop/Serial_Servo_Pico_HAT_Software/blob/main/images/run_script.png" />

   - For standalone execution save script into Pico as main.py,

     <img src= "https://github.com/sbcshop/Serial_Servo_Pico_HAT_Software/blob/main/images/main_save.png" />

     Try out below provided reference example demo codes and modify to build your own application codes.
     

### Example Codes
   Try reference demo codes to test onboard components of HAT, make sure to save library file to run Serial Servo Motor related codes.
   - [Display Demo](https://github.com/sbcshop/Serial_Servo_Pico_HAT_Software/blob/main/examples/Demo_Display.py) : code to test display
   - [Motor Mode Demo](https://github.com/sbcshop/Serial_Servo_Pico_HAT_Software/blob/main/examples/Demo_Motor_mode.py) : code to run Serial Servo in Motor Mode, 360 degree rotation
   - [Servo Mode Demo](https://github.com/sbcshop/Serial_Servo_Pico_HAT_Software/blob/main/examples/Demo_Servo_mode.py) : code to run Servo with angular position
   - [More..](https://github.com/sbcshop/Serial_Servo_Pico_HAT_Software/tree/main/examples)

   Using this sample code as a guide, you can modify, build, and share codes!!
   
## Resources
  * [Schematic](https://github.com/sbcshop/Serial_Servo_Pico_HAT_Hardware/blob/main/Design%20Data/SCH%20Serial%20Servo%20Pico.pdf)
  * [Hardware Files](https://github.com/sbcshop/Serial_Servo_Pico_HAT_Hardware/tree/main)
  * [Step File](https://github.com/sbcshop/Serial_Servo_Pico_HAT_Hardware/blob/main/Mechanical%20Data/Serial%20Servo%20pico.step)
  * [MicroPython getting started for RPi Pico/Pico W](https://docs.micropython.org/en/latest/rp2/quickref.html)
  * [Pico W Getting Started](https://projects.raspberrypi.org/en/projects/get-started-pico-w)
  * [RP2040 Datasheet](https://datasheets.raspberrypi.com/pico/pico-datasheet.pdf)
  * [Serial Servo Manual](https://github.com/sbcshop/Serial_Servo_Breakout_Software/blob/main/Documents/SB_Servo_User_Manual.pdf)
 
## Related Products  
    
  * [Serial Servo Arduino Shield](https://shop.sb-components.co.uk/products/serial-servo-arduino-shield-1?_pos=4&_sid=1178c9361&_ss=r)

    ![Serial_Servo_Arduino_Shield](https://shop.sb-components.co.uk/cdn/shop/files/Artboard2_3.png?v=1718793718&width=150)

  * [Serial Servo ESP32](https://shop.sb-components.co.uk/products/serial-servo-based-on-esp32-1?_pos=1&_sid=c593a9981&_ss=r)

    ![Serial_Servo_ESP32](https://shop.sb-components.co.uk/cdn/shop/files/esp322.png?v=1718797495&width=150)
    
  * [Serial Servo Raspberry Pi HAT](https://shop.sb-components.co.uk/products/serial-servo-raspberry-pi-hat?_pos=2&_sid=c593a9981&_ss=r)

    ![Serial_Servo_Raspberry Pi_HAT](https://shop.sb-components.co.uk/cdn/shop/files/Artboard2_2.png?v=1718788805&width=150)

  * [Serial Servo Breakout](https://shop.sb-components.co.uk/products/serial-servo-breakout-1?_pos=3&_sid=5d47c0d83&_ss=r)

    ![Serial_Servo_Breakout](https://shop.sb-components.co.uk/cdn/shop/files/Artboard2.png?v=1718780131&width=150)
	


## Product License

This is ***open source*** product. Kindly check LICENSE.md file for more information.

Please contact support@sb-components.co.uk for technical support.
<p align="center">
  <img width="360" height="100" src="https://cdn.shopify.com/s/files/1/1217/2104/files/Logo_sb_component_3.png?v=1666086771&width=300">
</p>
