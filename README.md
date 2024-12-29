# Heart Rate Monitoring System

This project is a heart rate monitoring system using an STM32 microcontroller, MAX30102 sensor, SSD1306 OLED display, and HC-05 Bluetooth module. The system reads heart rate data from the MAX30102 sensor and displays it on the OLED screen. Additionally, it can send heart rate data via USART and Bluetooth. While the design supports interrupts, this implementation connects the INT pin to ground for simplicity. You can enable interrupts if desired.

## Table of Contents

- [✨ Features](#features)
- [🔧 Hardware Requirements](#hardware-requirements)
- [📊 Software Requirements](#software-requirements)
- [⚙️ Installation](#installation)
- [🛌 Usage](#usage)
- [🔐 Driver Implementation](#driver-implementation)
- [💼 Contributing](#contributing)
- [🔒 License](#license)

---

## ✨ Features

- ✔️ Real-time heart rate monitoring using MAX30102 sensor.
- ✔️ Display heart rate data on SSD1306 OLED screen.
- ✔️ Send heart rate data via USART.
- ✔️ Bluetooth connectivity using HC-05 for mobile data transmission.
- ✔️ Built-in plotting function for visualizing heart rate data.

---

## 🔧 Hardware Requirements

- STM32 microcontroller (e.g., STM32F4 series).
- MAX30102 heart rate sensor.
- SSD1306 OLED display.
- HC-05 Bluetooth module.
- USART interface for data transmission.

---

## 📊 Software Requirements

- STM32CubeMX.
- STM32 HAL Library.
- Keil MDK or STM32CubeIDE.
- OLED library for SSD1306.
- MAX30102 library.

---

## ⚙️ Installation

1. Clone the repository:

    ```sh
    git clone https://github.com/yourusername/heart-rate-monitoring.git
    cd heart-rate-monitoring
    ```

2. Open the project in STM32CubeMX and generate the code.
3. Open the generated project in Keil MDK or STM32CubeIDE.
4. Add the necessary libraries for SSD1306, MAX30102, and HC-05.
5. Build and flash the project to your STM32 microcontroller.

---

## Pin Configuration:

<img src="asset/Pin_config.png" alt="Heart Rate Monitoring System" style="border: 1px solid black; width: 500px;">

## 🎥 Working Demo

[![Watch the demo,Check in asset folder]](asset/Working_demo.mp4)


## 🛌 Usage

1. Connect the MAX30102 sensor and SSD1306 OLED display to the STM32 microcontroller as per the pin configuration in the code.
2. Connect the HC-05 Bluetooth module RX and TX pins to PA9 and PA10, respectively, and the INT pin of the MAX30102 sensor to ground (or configure it for interrupts if desired).
3. Power up the system.
4. Place your finger on the MAX30102 sensor.
5. View the heart rate data on the OLED screen.
6. Use the Serial Bluetooth Terminal app ([link](https://play.google.com/store/apps/details?id=de.kai_morich.serial_bluetooth_terminal&pcampaignid=web_share)) to receive heart rate data via Bluetooth.
7. If connected, the heart rate data will also be sent via USART.

---

## 🔐 Driver Implementation

### STM32CubeMX Setup

- **I2C Configuration:** Set I2C1 to "I2C" mode for MAX30102.
- **USART Configuration:** Set USART1 to "Asynchronous" mode for HC-05 and data transmission.
- **GPIO Setup:** Configure pins for I2C and USART communication.

> **Note:** Interrupts are optional. To enable, configure an external interrupt pin for the INT# pin of MAX30102. Set up an external interrupt pin in GPIO settings, use "**external interrupt mode with falling edge trigger detection**" and "**pull-up**" settings. Activate the external interrupt in NVIC settings by checking the corresponding box. Connect the INT# pin of your MAX30102 to this external interrupt pin.

---

### Driver Code

#### Include Libraries

```c
#include "max30102_for_stm32_hal.h"
#include "ssd1306.h"
#include "usart.h"
```

#### Initialization Before the Superloop

1. Declare an `max30102_t` object:
    ```c
    max30102_t max30102;
    ```

2. Initialize the sensor:
    ```c
    max30102_init(&max30102, &hi2c1);
    max30102_reset(&max30102);
    max30102_clear_fifo(&max30102);
    ```

3. Configure the sensor:
    ```c
    max30102_set_fifo_config(&max30102, max30102_smp_ave_8, 1, 7);
    max30102_set_led_pulse_width(&max30102, max30102_spo2_16_bit);
    max30102_set_adc_resolution(&max30102, max30102_spo2_adc_2048);
    max30102_set_sampling_rate(&max30102, max30102_spo2_800);
    max30102_set_led_current_1(&max30102, 6.2);
    max30102_set_led_current_2(&max30102, 6.2);
    max30102_set_mode(&max30102, max30102_spo2);
    ```

4. Initialize Bluetooth:
    ```c
    HAL_UART_Transmit(&huart1, (uint8_t*)"Bluetooth Ready\r\n", 18, HAL_MAX_DELAY);
    ```

#### In the Superloop

1. Continuously check for data:
    ```c
    while (1) {
        max30102_read_fifo(&max30102);
        int heart_rate = max30102_get_heart_rate(&max30102);

        // Display heart rate on OLED
        ssd1306_SetCursor(10, 10);
        char heart_rate_str[16];
        sprintf(heart_rate_str, "HR: %d bpm", heart_rate);
        ssd1306_WriteString(heart_rate_str, Font_7x10, White);
        ssd1306_UpdateScreen();

        // Send data via Bluetooth
        char bt_data[32];
        sprintf(bt_data, "Heart Rate: %d bpm\r\n", heart_rate);
        HAL_UART_Transmit(&huart1, (uint8_t*)bt_data, strlen(bt_data), HAL_MAX_DELAY);

        HAL_Delay(1000); // 1-second delay
    }
    ```

#### Optional: Interrupt Handling

If using interrupts, implement the following:

1. Include the library:
    ```c
    #include "max30102_for_stm32_hal.h"
    ```

2. Declare the `max30102_t` object as extern:
    ```c
    extern max30102_t max30102;
    ```

3. Call the interrupt handler in the ISR:
    ```c
    void EXTI15_10_IRQHandler(void) {
        HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_X); // Replace X with your pin number
        max30102_on_interrupt(&max30102);
    }
    ```

4. **GPIO Settings:**
    - Set up an external interrupt pin in GPIO settings, using "external interrupt mode with falling edge trigger detection" and "pull-up" settings.
    - Activate the external interrupt in NVIC settings by checking the corresponding box.
    - Connect the INT# pin of your MAX30102 to this external interrupt pin.

---

### ⚠️ Caution

This library is **NOT intended for clinical use**. Proceed at your own risk.

