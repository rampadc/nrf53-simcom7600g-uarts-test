⚠️ Bad code. DO NOT USE. Archive documentation for my future self only. ⚠️

👌 Update: Zephyr Project provides [example code](https://github.com/zephyrproject-rtos/zephyr/blob/main/drivers/modem/modem_iface_uart_async.c) to init and use UART in ASYNC mode with a modem. 👌

This example project shows how to use UART1 and UART0 in async mode. UART0 is connected to the nRF5340DK board's USB-to-UART bridge. UART1 is connected to an external SIMCOM7600G module.

This is for my own documentation. There are likely to be better ways to do this. I tried RTT logger to view the UART commands being received from the SIM7600G module but only received gibberish. This is the only approach so far that works for me.

In this project, UART0's RX is sent to UART1's TX, which is forwarded to SIM7600G's RX. Effectively, I'm using the nRF53DK as an expensive UART bridge that doesn't work all the time.

## Hardware

```
SIM7600G             nRF53DK (UART1)
----------           ----------------
      TX | <-------> | RX (P1.00)
      RX | <-------> | TX (1.01)
     GND | <-------> | GND
```

SIM7600's 5V supply is connected to a separate USB port on my powered USB hub. The nRF53DK is connected to the same USB hub.

SIM7600 board: https://www.waveshare.com/sim7600g-h-4g-hat.htm

Jumper config is in the `B` position, connecting the module's TXD/RTX to the Pi's TXD/RTX.

## Problems

- Unreliable UART buffers with characters being out of sequence
- Losing last `\n` in received UART messages

Example from device:

```
AT+CFUNC?<\r><\r><\n>
ERROR<\r>AT+CGPSZ<29><2>‚bŠATkoGP_?<\r><\r><\n>
+CGPS: 0,1<\r><\n>
<\r><\n>
OK<\r>AT<\r><\r><\n>
OK<\r>AT<\r><\r><\n>
OK<\r>
```

The above example shows that the messages sent between UART1<->modem is mostly correct and stable as there are responses, but when displayed on uart0, they are garbeled.