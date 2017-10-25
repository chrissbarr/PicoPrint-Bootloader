# PicoPrint Bootloader
This is a bootloader for the Aus3D PicoPrint 3D Printer control board. PicoPrint is a control board build around the STM32F4 family, in particular the STM32F446VET6.

### Default Behaviour
This bootloader allows for easier firmware updates via the DFU protocol. The current bootloader behaviour is:

1. On power-up, the status LED will pulse once to indicate that the bootloader has initialised.
2. The bootloader will then attempt to enter DFU mode. At this point, it will enumerate as a DFU device on the host computer. 
3. Once in DFU mode, the status LED will pulse constantly. The bootloader will behave as follows:
   1. If there is no user application loaded into flash, stay in DFU mode until one has been downloaded.
   2. If the BUTTON_PIN (internal pullup) was held low on startup, stay in DFU mode until a download has been completed. 
   3. Otherwise, stay in DFU mode for a short period (default 5s), or until a DFU transfer is completed (whichever is earlier). The 5s countdown pauses if a DFU transfer is active.
4. After leaving DFU mode, the bootloader will disconnect from the host USB, reset all peripherals, and begin the user application.

### Register Overrides
The bootloader will initialise the RTC peripheral, and check the persistant backup-registers to see if any flags have been set. 

Possible flags include **NONE**, **SKIP**, and **DFU**. On each startup, the bootloader will read the current value and reset the flag to **NONE** for the next boot. If any of the flags are read, they will cause the following behaviour:

* **NONE**: This is the default register state, and will cause no behavioural change to the bootloader.
* **SKIP**: If the SKIP flag is set, the bootloader will skip DFU setup and USB enumeration, and will boot the user application immediately. Basically, fast-boot mode.
* **DFU**: If the DFU flag is set, the firmware will enter DFU mode and stay there indefinitely, until a download has been completed. It is the software equivalent of pulling the BUTTON_PIN low, and can allow the user-level application to trigger a more user-accessible DFU upload, without the 5s timer.




