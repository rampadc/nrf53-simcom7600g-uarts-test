This example project shows how to use UART1 and UART0 in async mode. UART0 is connected to the nRF5340DK board's USB-to-UART bridge. UART1 is connected to an external SIMCOM7600G module.

This is for my own documentation. There are likely to be better ways to do this. I tried RTT logger to view the UART commands being received from the SIM7600G module but only received gibberish. 

In this project, UART0's RX is sent to UART1's TX, which is forwarded to SIM7600G's RX. Effectively, I'm using the nRF53DK as an expensive UART bridge that doesn't work all the time.

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
