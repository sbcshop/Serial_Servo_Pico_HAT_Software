# Serial_Servo_Pico_HAT_Software

<img src= "https://cdn.shopify.com/s/files/1/1217/2104/files/Artboard_1_1.png?v=1718781840" />

The Pico Serial Servo HAT is a versatile and compact controller with dual power options, two serial servo connectors, and two programmable buttons. It features a 1.14” TFT display for status updates, a power LED indicator, and supports simultaneous multi-servo control, making it ideal for projects requiring precise coordination and multiple servos with minimal wiring like Robotic Arm Control, CNC machine and Automation.

This github provides getting started instructions for Serial Servo Pico HAT.
### Features

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

## Getting Started with Serial Servo Based on ESP32
### Pinout
<img src= "https://cdn.shopify.com/s/files/1/1217/2104/files/esp32_Pin.png?v=1718797005" />

- (1) Serial Servo connector
- (2) Header 2.5” input (6~8.4V DC) 
- (3) DC Jack Input (6~8.4V DC)
- (4),(5), (6) Programmable Buttons 
- (7) Boot Button
- (8) Type C for Programming/Power
- (9) Reset Button
- (10) TFT 1.14” Display
- (11) ESP32 S3 microcontroller module
- (12) Power Status LED
- (13) & (14) GPIOs and Power breakout

  
### Interfacing Details

 - _Serial Servo Bus Pins:_
   * Servo connector having +ve[6~8.4VDC], -ve[GND] and Signal pin. 
   * Serial Servo Signal pins breakout into UART RXD and TXD to connect with PICO UART pins,
     
     | Pico | Servo | Description | 
     |---|---|---|
     | TXD0/GPIO43 | Servo Bus RXD | UART communication pin |
     | RXD0/GPIO44 | Servo Bus TXD | UART communication pin |
  
- _Display interfacing with ESP32_
    | Pico | Display | Function |
    |---|---|---|
    | IO12 | LCD_CLK | Clock pin of SPI interface for Display|
    | IO11 | LCD_DIN | MOSI (Master OUT Slave IN) pin of SPI interface|
    | IO10 | LCD_CS | Chip Select pin of SPI interface|
    | IO13 | LCD_DC| Data/Command (MISO) pin of SPI interface|
    | IO14 | LCD_RST | Display Reset pin |
    | IO9  | BL | Backlight of display|
  
- _Buttons Interfacing_
    | Pico | Hardware | Function |
    |---|---|---|
    |IO4 | BT1 | Programmable Button |
    |IO5 | BT2 | Programmable Button |

 
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
