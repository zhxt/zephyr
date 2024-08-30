/*
 * Copyright (c) 2024 Zhang Xingtao <zhxt@live.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>

/*
AW9523B is a 16 multi-function LED driver and GPIO controller. Any of the 16 I/O
ports can be configured as LED drive mode or GPIO mode.

After power on, all the 16 I/O ports are configured as GPIO output as default.
*/


// Register address

#define AW9523B_INPUT_PORT_0     0x00
#define AW9523B_INPUT_PORT_1     0x01
#define AW9523B_OUTPUT_PORT_0    0x02
#define AW9523B_OUTPUT_PORT_1    0x03
#define AW9523B_CONFIG_PORT_0    0x04
#define AW9523B_CONFIG_PORT_1    0x05
#define AW9523B_INT_PORT_0       0x06
#define AW9523B_INT_PORT_1       0x07

#define AW9523B_ID               0x10
#define AW9523B_CTL              0x11

/*
  Configure P0_0~P0_7 as LED or GPIO mode
  0: LED mode, 1: GPIO mode;  default: 0xFF
*/
#define AW9523B_MODE_SWITCH_P0   0x12

/*
  Configure P1_0~P1_7 as LED or GPIO mode
  0: LED mode, 1: GPIO mode;  default: 0xFF
*/
#define AW9523B_MODE_SWITCH_P1   0x13

/* GPIO or LED driver, in LED mode, supports 256 step dimming */

#define AW9523B_GPIO_P1_0        0x20
#define AW9523B_GPIO_P1_1        0x21
#define AW9523B_GPIO_P1_2        0x22
#define AW9523B_GPIO_P1_3        0x23

#define AW9523B_GPIO_P0_0        0x24
#define AW9523B_GPIO_P0_1        0x25
#define AW9523B_GPIO_P0_2        0x26
#define AW9523B_GPIO_P0_3        0x27
#define AW9523B_GPIO_P0_4        0x28
#define AW9523B_GPIO_P0_5        0x29
#define AW9523B_GPIO_P0_6        0x2A
#define AW9523B_GPIO_P0_7        0x2B

#define AW9523B_GPIO_P1_4        0x2C
#define AW9523B_GPIO_P1_5        0x2D
#define AW9523B_GPIO_P1_6        0x2E
#define AW9523B_GPIO_P1_7        0x2F

#define AW9523B_SW_RSTN          0x7F
