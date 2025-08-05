.. zephyr:code-sample:: Motion detection
   :name: motion_detect

   Demonstrate motion-activated wake and FIFO watermark features of the ADXL367 accelerometer, combined with low-power modes.

Overview
********

This sample configures the ADXL367 accelerometer to detect motion and inactivity, and to wake the system from a low-power state when motion is detected.

The application:

#. Configures the **INT1** pin of the ADXL367 to trigger on motion or inactivity and maps it to **GPIO pin 8**  
   *(requires a physical jumper between INT1 and pin 8 on the ME30 board)*  
#. Sets up FIFO watermark interrupts for accelerometer data capture
#. Uses a dedicated thread to process accelerometer data on wake
#. Uses another thread to put the MCU into **STANDBY mode** after inactivity
#. Toggles a status LED depending on whether motion is detected

When motion is detected, the LED turns on and the FIFO is read out. When inactivity is detected, the LED turns off and the MCU re-enters standby.

Requirements
************

Your board must:

#. Be connected to an ADXL367 accelerometer over I2C  
#. Have a physical connection from **INT1** on the ADXL367 to **GPIO pin 8**
#. Have an LED configured using the `led0` devicetree alias
#. Support GPIO wake-up sources
#. Support low-power standby or sleep modes

Building and Running
********************

Build and flash the motion detection application as follows, changing ``me30_board`` for your board:

.. zephyr-app-commands::
   :zephyr-app: samples/sensor/motion_detect
   :board: me30_board
   :goals: build flash
   :compact:

After flashing:

- The device waits 5 seconds before initializing
- When motion is detected:
  - The LED turns **on**
  - FIFO watermark data is read
- When inactivity is detected:
  - The LED turns **off**
  - The system enters **STANDBY** mode until motion is detected again

Build errors
************

You will see a build error if:

- The `led0` alias is not defined in your board's devicetree
- GPIO pin 8 is not available or cannot be configured
- The ADXL367 driver or I2C interface is missing

Adding board support
********************

To add support for your board, ensure:

- The `led0` alias is defined in the devicetree
- GPIO pin 8 is configured as an input and mapped as a wake-up source

Example devicetree overlay:

.. code-block:: DTS

   / {
   	aliases {
   		led0 = &myled0;
   	};

   	leds {
   		compatible = "gpio-leds";
   		myled0: led_0 {
   			gpios = <&gpio0 13 GPIO_ACTIVE_LOW>;
   		};
   	};

   	int1: int1_pin {
   		gpios = <&gpio0 8 GPIO_ACTIVE_HIGH>;
   	};
   };

Tips:

- See the ADXL367 datasheet for details on **Linked Mode** and **Loop Mode** motion detection
- For GPIO wake-up, verify your board's SoC supports it and configure in hardware
- This example is tuned for **±2g range** at **50 Hz ODR** with FIFO streaming
