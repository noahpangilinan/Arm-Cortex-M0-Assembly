# ARM Cortex-M0+ Assembly

Lab exercises from RIT's CMPE-250 (Assembly and Embedded Programming), written in ARM Thumb
assembly for the NXP **FRDM-KL05Z** board (MKL05Z32, Cortex-M0+). Each file is a complete Keil
MDK-ARM program built on the course's KL05 startup template, from basic arithmetic up to
interrupt-driven drivers.

## Programs

| File | What it covers |
| --- | --- |
| [`Basic-Arithmetic-Operations.s`](Basic-Arithmetic-Operations.s) | Registers, add/subtract, shifts, and the APSR condition flags |
| [`Memory-ConditionalBranching-Debugging.s`](Memory-ConditionalBranching-Debugging.s) | Loads and stores, conditional branches, stepping through the debugger |
| [`Iteration-and-Subroutines.s`](Iteration-and-Subroutines.s) | Loops, subroutines and the stack |
| [`PolledSerial-IO.s`](PolledSerial-IO.s) | UART0 setup and polled character I/O to a terminal |
| [`SecureString-IO.s`](SecureString-IO.s) | Bounded string input and output over serial |
| [`CircularFIFO-Queue-Operations.s`](CircularFIFO-Queue-Operations.s) | A circular FIFO queue: init, enqueue, dequeue, status |
| [`Serial-IO-Driver.s`](Serial-IO-Driver.s) | Interrupt-driven UART driver with transmit and receive queues |
| [`PIT-Timer-Driver.s`](PIT-Timer-Driver.s) | Periodic Interrupt Timer driver with a running count |
| [`Multiprecision-Arithmetic.s`](Multiprecision-Arithmetic.s) | Add and subtract on numbers wider than 32 bits |
| [`DA-and-AD-Conversions/`](DA-and-AD-Conversions/) | Mixed C and assembly: DAC output, ADC input and TPM PWM, with the C main in `da_c.c` |

## Running them

1. Install [Keil MDK-ARM](https://www.keil.arm.com/) with the Kinetis KL05 device pack.
2. Create a project for the MKL05Z32xxx4, add the `.s` file (plus `da_c.c` and `da_h.h` for the
   D/A and A/D exercise), and build.
3. Flash the FRDM-KL05Z over OpenSDA, or run it in the simulator. The serial exercises talk to a
   terminal on the board's virtual COM port.

The device header `MKL05Z4.h` and the course's shared subroutines aren't included here, so some
exercises need those to build.
