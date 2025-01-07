# SSD1306 Menu Interface with MPU6500  

This repository contains the implementation of a simple menu interface using an SSD1306 OLED display and an MPU6500 accelerometer on an I2C bus using a STM32F407. The system provides navigation through the menu using two push-buttons:  

- **CHANGE (PE9):** Moves the cursor to the next menu option.  
- **ENTER (PE10):** Selects the currently highlighted menu option.  

## Features  

- **Display:** SSD1306 OLED connected via I2C.  
- **Accelerometer:** MPU6500 for additional sensing capabilities.  
- **Navigation:** Two buttons for menu interaction (CHANGE and ENTER).  
- **Hardware Setup:**  
  - I2C Pins:  
    - `PB6` - SCL (Clock).  
    - `PB9` - SDA (Data).  
  - Button Pins:  
    - `PE9` - UP button.  
    - `PE10` - ENTER button.
    - `PE11` - DOWN button.

## Hardware Requirements  

- STM32 microcontroller.  
- SSD1306 OLED display (I2C interface).  
- MPU6500 accelerometer (I2C interface).  
- BMP280 barometer (I2C interface)
- Three push-buttons for navigation.  

## Software Setup  

- IDE: STM32CubeIDE.  
- Libraries:  
  - SSD1306 display driver.  
  - HAL Library
    
## Getting Started  

1. **Hardware Wiring:**  
   - Connect the SSD1306 and MPU6500 to the I2C bus on `PB6` (SCL) and `PB9` (SDA).  
   - Connect the CHANGE button to `PE9` and the ENTER button to `PE10`.  

2. **Code Configuration:**  
   - Ensure I2C, GPIO, and necessary peripherals are configured correctly in the project.  
   - Customize the menu options in the source code as needed.  

3. **Compilation and Deployment:**  
   - Compile the code and flash it to the STM32 microcontroller using your preferred toolchain.  

4. **Operation:**  
   - Use the CHANGE button to navigate the menu.  
   - Use the ENTER button to select an option.  
