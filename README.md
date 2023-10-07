This example project shows how to use UART1 to talk to SIM7600G using Zephyr's modem library. Because Nordic is using 3.3.99, or a forked version of Zephyr, one of the library path needed to be changed.

## Hardware

```
SIM7600G             nRF53DK (UART1)
----------           ----------------
      TX | <-------> | RX (P1.05)
      RX | <-------> | TX (1.06)
     GND | <-------> | GND
```

UART1 on nRF5340 is configured on P1.06 and P1.05 because P1.00 and P1.01 is connected to the debug interface (IF), which applies some biasing on the RX pin causing it not to receive packets correctly ([source](https://devzone.nordicsemi.com/f/nordic-q-a/98697/uart-rx-pin-pulled-up?ReplyFilter=Answers&ReplySortBy=Answers&ReplySortOrder=Descending)).

SIM7600's 5V supply is connected to a separate USB port on my powered USB hub. The nRF53DK is connected to the same USB hub.

SIM7600 board: https://www.waveshare.com/sim7600g-h-4g-hat.htm

Jumper config is in the `B` position, connecting the module's TXD/RTX to the Pi's TXD/RTX.
