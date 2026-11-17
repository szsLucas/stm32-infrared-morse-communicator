# STM32 Infrared Morse Code Communicator

**A two-terminal Morse communication prototype using STM32F401RCT6 microcontrollers, an infrared optical link, and a Python graphical interface.**

Academic electronics and embedded-systems project — **Federal University of Uberlândia (UFU), Brazil, 2025**.

This repository contains the **available C firmware and Python GUI source files**, together with the original Portuguese technical report. It is a portfolio/documentation repository; a complete STM32CubeIDE project (including generated configuration and HAL support files) has not been included.

> **Important version note:** The attached technical report documents an earlier **bare-metal / TinyUSB USB CDC** configuration. The source files uploaded here implement an **STM32 HAL / USART2 UART** configuration instead. They should be treated as distinct project revisions: **do not use the report's pinout, 100 ms timing, or USB CDC build instructions to wire or compile the C source in this repository**. The source code below is the reference for the UART/HAL version.

## How it works

Each end of the link has a computer running the Python application and an STM32 board containing an infrared transmitter and receiver. The application encodes a typed message into Morse and sends it to its local board through a **115200-baud UART connection**. The board transmits timed infrared pulses; the other board detects pulse durations, reconstructs the Morse symbols, and returns them to its computer for display as text.

```text
PC 1 (Python GUI)
        | UART 115200 (USART2)
        v
STM32F401RCT6 1 -- IR LED ~~ infrared light ~~ IR receiver --> STM32F401RCT6 2
                                                             |
                                                   UART 115200 (USART2)
                                                             v
                                                      PC 2 (Python GUI)
```

Both boards use the same firmware and can transmit or receive, **one direction at a time (half-duplex)**. Local IR reception is disabled during transmission to prevent optical self-echo.

## Technology stack

| Layer | Implementation in the included source files |
| --- | --- |
| Microcontroller | STM32F401RCT6 (ARM Cortex-M4) |
| Firmware | C, **STM32 HAL**, interrupts and TIM2 |
| PC-to-board link | **USART2 UART**, 115200 baud, 8N1 |
| Optical link | IR LED and an active-high IR receiver |
| Timing | TIM2 configured for a **1 MHz** timer (1 µs resolution) |
| Morse unit | **200 ms** per dot, **600 ms** per dash |
| Desktop GUI | Python, CustomTkinter, pySerial |

## Repository contents

```text
stm32-morse-github-ready/
├── README.md
├── requirements.txt
├── firmware/
│   └── morse_stm32f401rct6.c
├── software/
│   └── morse_python.py
└── docs/
    └── technical-report.docx
```

- [`firmware/morse_stm32f401rct6.c`](firmware/morse_stm32f401rct6.c): STM32 HAL firmware with UART input/output, IR pulse generation and detection, and Morse gap detection.
- [`software/morse_python.py`](software/morse_python.py): user interface for serial-port connection, message encoding, sending, receiving, and decoding.
- [`docs/technical-report.docx`](docs/technical-report.docx): 45-page project report **for the earlier USB CDC / bare-metal configuration**, with architecture, hardware discussion, calculations, and reported test results.

## Hardware connections — source-code version

The following pin assignments are taken from the **included HAL firmware**, **not** from the report:

| STM32 signal | Pin | Purpose |
| --- | --- | --- |
| IR transmit | **PB6** | Output controlling the IR LED transmitter stage (source comments specify an NPN transistor and resistor) |
| IR receive | **PA0** | Active-high IR detector input, interrupt on both edges (EXTI0) |
| UART transmit | **PA2** | USART2 TX; PC serial adapter / board VCP as applicable |
| UART receive | **PA3** | USART2 RX; PC serial adapter / board VCP as applicable |

The C file configures PA0 with an internal pull-down. The complete wiring schematic, component values, and exact board-to-PC UART interface for **this HAL revision** have **not** been provided separately. Check the board's actual pin connections and 3.3 V logic compatibility before wiring. Do not connect UART GPIO pins directly to a computer's USB data lines.

## Firmware highlights

- A 200 ms Morse time unit: **dot = 1 unit**, **dash = 3 units**.
- **TIM2** free-running 32-bit counter configured for 1 µs ticks.
- **EXTI0 interrupts** on rising and falling edges of the received IR signal.
- Pulse validation and classification: pulses below 100 ms or above 800 ms are discarded; the 400 ms threshold separates dots and dashes.
- Gap processing distinguishes character, word, and end-of-message separators.
- **Half-duplex echo protection** disables IR receive interrupts while sending.
- **UART receive interrupt** assembles lines sent by the Python application; an output queue forwards recovered Morse symbols back to the PC.

These are implementation details found in the source, not independently measured performance claims.

## Python application

The desktop interface provides a serial-port field, connect/disconnect controls, a message input, and a log of transmitted and received text and Morse. It normalizes accented input characters and supports Latin letters, digits, and common punctuation.

Install Python dependencies:

```bash
python -m pip install -r requirements.txt
```

Run the GUI:

```bash
python software/morse_python.py
```

Enter the serial port corresponding to the connected STM32 (for example, `COM3` on Windows or `/dev/ttyUSB0` / `/dev/ttyACM0` on Linux, depending on the adapter). The PC side and firmware both use **115200 baud**.

**Note:** The GUI can launch without hardware, but actual message exchange requires two appropriately programmed and connected STM32 terminals. The STM32 firmware has not been built or hardware-tested as part of this repository packaging.

## Building the STM32 firmware

The C file uses `stm32f4xx_hal.h` and expects the STM32 HAL project environment. It is **not a standalone buildable `.c` file**. To compile it, create or restore the corresponding STM32F401RCT6 project in **STM32CubeIDE**, with the matching HAL package, startup code, linker script, and required interrupt setup. Check for duplicate `EXTI0_IRQHandler` and `USART2_IRQHandler` definitions if CubeIDE generated these handlers automatically.

The original `.ioc` configuration and complete IDE project have not been included, so reproducible firmware build steps cannot yet be guaranteed.

## Reported academic tests

The original report describes eight successful functional checks, including text/Morse conversion, serial-link enumeration, IR signaling, timing classification, separators, echo suppression, and end-to-end communication. In that report, the message **`OLA MUNDO`** was correctly reconstructed on the second computer.

**These results belong to the report's earlier USB CDC / bare-metal implementation** (100 ms Morse unit) and should **not** be presented as independent validation of the included HAL/UART source files (200 ms Morse unit). The current source files have not been independently compiled or tested here.

## Limitations and future improvements

- The optical link is half-duplex and sensitive to alignment and ambient light.
- The current source tree does not include the full STM32CubeIDE project or a verified schematic for the HAL/UART wiring.
- There is no documented maximum operating range, packet-error rate, or systematic hardware validation for the included revision.
- Potential improvements include error detection, acknowledgment/retransmission, more robust IR modulation, and experimental performance characterization.

## Academic and attribution notes

Developed as an academic project at the **Faculty of Electrical Engineering, Federal University of Uberlândia**. If the files include group work or borrowed code, review contributor credits and permissions before public release. The repository description does not imply individual authorship of every component.
