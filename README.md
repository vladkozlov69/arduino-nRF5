# Arduino Core for Nordic Semiconductor nRF5 based boards


Program your [Nordic Semiconductor](https://www.nordicsemi.com) nRF51 or nRF52 board using the [Arduino](https://www.arduino.cc) IDE.

Does not require a custom bootloader on the device.

modification https://github.com/sandeepmistry/arduino-nRF5/



## Supported boards

### ++ nRF52840

### ++ nRF52811

### nRF52832

### nRF51




## Installing


###### Driver Setup for Segger J-Link

 1. Download [Zadig](http://zadig.akeo.ie)
 2. Plugin Segger J-Link or DK board
 3. Start ```Zadig```
 4. Select ```Options -> List All Devices```
 5. Plug and unplug your device to find what changes, and select the ```Interface 2``` from the device dropdown
 6. Click ```Replace Driver```

__NOTE__: To roll back to the original driver go to: Device Manager -> Right click on device -> Check box for "Delete the driver software for this device" and click Uninstall

###### Driver Setup for Black Magic Probe
1. Download .inf file drivers from [blacksphere github](https://github.com/blacksphere/blackmagic/tree/master/driver)
2. Plugin Black Magic Probe
3. Point the installer to the folder containing blackmagic.inf

__NOTE__: If using Windows 10 or Linux then two UART COM ports will be visible without requiring additional drivers



## Low Frequency Clock Source (LFCLKSRC)

If the selected board has an external 32 kHz crystal connected, it will be used as the source for the low frequency clock. Otherwise the internal 32 kHz RC oscillator will be used. The low frequency clock is used by the `delay(ms)` and `millis()` Arduino API's.

The Generic nRF51 and nRF52 board options have an additional menu item under `Tools -> Low Frequency Clock` that allows you to select the low frequency clock source. However, Nordic does not recommend the Synthesized clock, which also has a significant power impact.

