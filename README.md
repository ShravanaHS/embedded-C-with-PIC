# embedded-C-with-PIC

## Introduction
Hi! This repository contains a variety of **hardware interface codes for the PIC16F877A microcontroller**. All code is written in embedded C and does not rely on any external libraries—everything is **implemented from scratch**.
Development is done using **[MPLAB X IDE](https://www.microchip.com/en-us/tools-resources/develop/mplab-x-ide)** for code writing and compilation, and **[SimulIDE](https://simulide.com/p/)** for functional simulation and hardware verification.
Both **tools are free, open source**, and easy to use on most platforms.

## Project 1: Blinking LED

### Description
This is a simple project to blink an LED connected to the PIC16F877A microcontroller. The LED blinks with a delay of 500ms ON and 500ms OFF indefinitely, demonstrating basic digital output control.

### Hardware Connections
- **LED** connected to **PORTC pin RC0**
- The pin is configured as output in the firmware.
- No additional components required apart from an LED and a current limiting resistor.

### Code Overview
- The code initializes PORTC, pin RC0 as an output.
- In the main loop, the LED is turned ON, a delay is applied, then the LED is turned OFF, followed by another delay.
- The delay and LED toggling repeats infinitely.

### Source Code Snippet (main.c)
```
#include <xc.h>

#define _XTAL_FREQ 20000000 // 20MHz oscillator frequency

void main(void) {
TRISCbits.TRISC0 = 0; // Configure RC0 as output
while(1) {
PORTCbits.RC0 = 1; // LED ON
__delay_ms(500);
PORTCbits.RC0 = 0; // LED OFF
__delay_ms(500);
}
}
```

### Simulation Result
![Blinking LED Simulation](Blinking_LED/simulation.gif)

### Download  
[Download Blinking LED project ZIP](Blinking_LED/Blinking_LED.zip)


## Project 2: Push Button Interface

### Description
This project demonstrates interfacing push buttons with the PIC16F877A microcontroller. The board reads inputs from push buttons connected to PORTB pins and controls LEDs connected to PORTC pins accordingly. This project teaches basic input reading, conditional logic, and output control on the PIC.

### Hardware Connections
- **Push Buttons** connected to **PORTB pins RB7 and RB1**, configured as inputs.
- **LEDs** connected to **PORTC pins RC0 and RC3**, configured as outputs.
- When a button is pressed, the corresponding LED will light up.
- Pull-up or pull-down resistors should be used externally or internal pull-ups enabled for stable button readings.

### Theory
- PIC pins set as inputs (TRISB bits = 1) can read button states (logical HIGH or LOW).
- Pins set as output (TRISC bits = 0) can drive LEDs by setting pin HIGH (LED ON) or LOW (LED OFF).
- In code, the status of button pins is checked in an infinite loop and appropriate LEDs are turned on or off.
- Proper bitwise assignments in C (`=` for assignment, not `==` for comparison) are critical for expected behavior.

### Source Code Snippet (main.c)
```
#pragma config FOSC = EXTRC // Oscillator Selection bits (RC oscillator)
#pragma config WDTE = ON // Watchdog Timer Enable bit (WDT enabled)
#pragma config PWRTE = OFF // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = ON // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = ON // Low-Voltage Programming enabled
#pragma config CPD = OFF // Data EEPROM Code Protection off
#pragma config WRT = OFF // Flash Program Memory Write Enable off
#pragma config CP = OFF // Flash Program Memory Code Protection off

#include <xc.h>

#define _XTAL_FREQ 20000000

static void init_config(void){
TRISB = 0xFF; // PORTB all inputs (for buttons)
PORTB = 0x00; // Clear PORTB latch
TRISC = 0x00; // PORTC all outputs (for LEDs)
PORTC = 0x00; // Clear PORTC latch
}

void main(void) {
init_config();
while(1){
// Check status of push button connected to RB7
if(PORTBbits.RB7 == 1){
PORTCbits.RC0 = 1; // Turn ON LED at RC0
}
// Check status of push button connected to RB1
else if(PORTBbits.RB1 == 0){
PORTCbits.RC3 = 1; // Turn ON LED at RC3
}
else {
PORTCbits.RC3 = 0; // Turn OFF LEDs
PORTCbits.RC0 = 0;
}
}
}

```

### Simulation Result
![Push Button Simulation](Push_Button_Interface/simulation.gif)

### Download  
[Download Push Button Interface project ZIP](Push_Button_Interface/Push_Button_Interface.zip)


## Project 3: 7-Segment Display Interface

### Description
This project demonstrates interfacing a single common-cathode 7-segment display with the PIC16F877A microcontroller. The microcontroller drives PORTD pins to display digits 0 through 9 in a loop. The project illustrates how to encode digits to segment patterns and control a 7-segment display directly.

### Hardware Connections
- **7-segment display (common cathode)** connected to **PORTD pins RD0 to RD6** as follows:

| Segment | PIC PORTD Pin |
|---------|---------------|
| a       | RD0           |
| b       | RD1           |
| c       | RD2           |
| d       | RD3           |
| e       | RD4           |
| f       | RD5           |
| g       | RD6           |

- **Common cathode** pin of the 7-segment is connected to ground.
- Each segment lights up when the corresponding PORTD pin is set HIGH (1).
- Current limiting resistors (typically 330Ω) should be used in series with each segment to protect LEDs inside the display.

### Theory of Operation
- A 7-segment display has seven LEDs labeled a to g arranged to display numerals by lighting specific segments.
- The PIC outputs a 7-bit pattern on PORTD representing each digit’s segment ON/OFF pattern.
- The array `segment[]` stores the bit patterns for digits 0 to 9.
- By cycling through this array, the microcontroller displays digits 0–9 repeatedly.
- Timing is controlled with a delay loop to make the digits readable.

### Segment Bit Patterns Explanation
Each byte in `segment[]` corresponds to the segments a-g as:

| Bit | Segment |
|-----|---------|
| 0   | a       |
| 1   | b       |
| 2   | c       |
| 3   | d       |
| 4   | e       |
| 5   | f       |
| 6   | g       |
| 7   | (unused)|

For example:  
- `0x3F` (0011 1111 binary) lights up segments a,b,c,d,e,f (digit '0')  
- `0x06` (0000 0110 binary) lights up segments b,c (digit '1')

### Source Code Snippet (main.c)
```
#include <xc.h>
#pragma config WDTE = OFF

#define _XTAL_FREQ 20000000 // oscillator frequency

unsigned char segment[] = {0x3F, 0x06, 0x5B, 0x4F, 0x66,
0x6D, 0x7C, 0x07, 0x7F, 0x6F};

static void init_config(void) {
TRISD = 0x00; // Configure PORTD as output
PORTD = 0x00; // Clear PORTD
}

void main(void) {
init_config();
while(1) {
for(unsigned char i = 0; i < 10; i++) {
PORTD = segment[i]; // Output pattern for digit i
for(unsigned long j = 50000; j--; ); // Delay loop
}
}
}

```
### Simulation Result
![7-Segment Simulation](SevenSegment/simulation.gif)

### Download  
[Download 7-Segment Display project ZIP](SevenSegment/SevenSegment.zip)


## Project 4: LCD Display Interface

### Description
This project demonstrates interfacing a 16x2 character LCD (HD44780 compatible) with the PIC16F877A microcontroller in 8-bit mode. The LCD is used to display strings and characters by sending commands and data directly through PORTC and control pins on PORTD. This project covers basic LCD initialization, command writing, data writing, and simple string display.

---

### Theory

**LCD Display:**  
A 16x2 LCD has 16 columns and 2 rows, with each character cell consisting of a 5x8 dot matrix display. Characters are displayed by turning on the LEDs (dots) in the matrix pattern.

Controlling individual LEDs directly is complex, so LCD modules like the HD44780 use an internal controller that handles dot activation. The controller exposes two main registers:  
- **Instruction Register (IR)**: Receives commands controlling the LCD (e.g., clear screen, cursor movement)  
- **Data Register (DR)**: Receives data corresponding to characters to display

Each character on the 16x2 LCD corresponds to a unique **address** in the controller's DDRAM (Display Data RAM):  
- First row addresses range from `0x80` to `0x8F`  
- Second row addresses range from `0xC0` to `0xCF`

To display a character, you send the address to indicate location and then the character data.

---

### LCD Pin Functionality

| LCD Pin | Function                         |
|---------|---------------------------------|
| RS      | Register Select (0 = command, 1 = data)  |
| RW      | Read/Write (0 = write, 1 = read)         |
| EN      | Enable (latch signal)                    |
| D0-D7   | Data pins (8-bit mode)                    |

- When **RS = 0**, data sent is stored in the instruction register (commands).  
- When **RS = 1**, data sent is stored in the data register (characters).  

---

### 8-bit vs 4-bit Mode

- **8-bit mode:** Connect a full 8-bit port (e.g., PORTC) for data pins D0-D7 to LCD. Faster but uses more I/O pins.
- **4-bit mode:** Use only 4 data lines, sending commands in two nibbles, saving pin count at the cost of complexity.

---

### Key LCD Commands Used

| Command       | Function                             |
|---------------|------------------------------------|
| `0x38`        | Function set: 8-bit, 2 lines, 5x8 font |
| `0x06`        | Entry mode set: cursor auto-increment |
| `0x0C`        | Display on, cursor off              |
| `0x01`        | Clear display                      |

---

### Working Flow

- Configure ports connected to LCD data and control lines as outputs.
- Send the above commands in sequence to initialize the LCD.
- Display characters by writing data to the LCD’s data register.
- Use delays to allow the LCD time to process commands.

---
### Source code snpiiet
```
#include <xc.h>
#include "lcd.h"

#define RS PORTDbits.RD2
#define RW PORTDbits.RD1
#define EN PORTDbits.RD0

void lcd_init(void) {
TRISC = 0x00; // Configure PORTC as output (data)
TRISD = 0x00; // Configure PORTD as output (control pins)

text
lcd_command(0x38); // 8-bit, 2 line, 5x8 font
lcd_command(0x0C); // Display ON, Cursor OFF
lcd_command(0x06); // Auto-increment cursor
lcd_command(0x01); // Clear display
__delay_ms(2);     // Clearing takes >1.5ms
}

void lcd_command(unsigned char cmd) {
PORTC = cmd;
RS = 0; // Command mode
RW = 0; // Write mode
EN = 1; // Latch data
__delay_ms(2);
EN = 0;
}

void lcd_data(unsigned char data) {
PORTC = data;
RS = 1; // Data mode
RW = 0; // Write mode
EN = 1; // Latch data
__delay_ms(2);
EN = 0;
}

void lcd_puts(const char *str) {
while(*str) {
lcd_data(*str++);
}
}

void lcd_clear(void) {
lcd_command(0x01);
__delay_ms(2);
}
```

### Sample main.c usage snippet
```
#include <xc.h>
#include "lcd.h"

#define _XTAL_FREQ 20000000

void main(void) {
lcd_init();
lcd_clear();
lcd_puts("Hello, PIC!");
while(1);
}
```
### Simulation Result
![LCD Simulation](LCD/simulation.gif)

### Download  
[Download LCD Interface project ZIP](LCD/LCD.zi




