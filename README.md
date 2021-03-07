# Smart heater

Smart thermostat using an esp8266. I made this in order to make my heater "smart".

It uses a tiny i2c oled to display the current state.

## Features:

- Physical control with 3 buttons
- Control throught mqtt 
- The mqtt topics and payloads are compatible with home assistant
- Sync feature (So you can have multiple heaters in one room and control them)

## Parts:

- Solid state relay(SSR) or standard relay.
- 0,96 inch i2c oled display.
- Esp8266 microcontroller
- ds18b20 temperature sensor
- 3 push buttons
