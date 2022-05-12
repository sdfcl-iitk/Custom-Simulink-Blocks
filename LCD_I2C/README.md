# MPU9250 Raspberry Pi Block

Ported the Arduino LCD I2C Library to a Raspberry Pi Simulink block https://github.com/fdebrabander/Arduino-LiquidCrystal-I2C-library. The block uses the Raspberry Pi's I2C bus to communicate with the device. This block only works with a 2x16 LCD. It accepts a 2x17 null-terminated char array. Each character in the array corresponds to a character on the screen. One extra byte is present on each row for the null-termination character. Note that it takes a lot of time to update the LCD. Thus it is best to run this system at no more than 10 Hz.

## Linking libi2c.a

The `libi2c.a` static library must be linked with the binary for it to work. This requirement is specified in the `Libraries` tab of the S-Function Builder. **This file must come from the Raspberry Pi itself.** A file has been included for demo purposes but this file might not work. To fetch the file, follow these instructions:

1. Install the `libi2c-dev` package on the Raspberry Pi. This should be automatically installed if you are using the MathWorks Raspbian image.
2. Find the location of the `libi2c.a` file using the command `dpkg -L libi2c-dev`.
3. Copy that file from the Raspberry Pi and paste it in the project folder. Replace the existing file.