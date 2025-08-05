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
