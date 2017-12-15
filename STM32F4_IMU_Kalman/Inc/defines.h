/*
 * defines.h
 *
 *  Created on: 11 Sep 2017
 *      Author: Bartek
 */

#ifndef DEFINES_H_
#define DEFINES_H_

// Accel macros
#define LSM303_ACC_ADDRESS (0x19 << 1) //shift to left because LSB can be 1 or 0 depending on Read/Write operation; HAL handles it by itself
//CTRL_REG1_A = [ODR3][ODR2][ODR1][ODR0][LPen][Zen][Yen][Xen]
#define LSM303_ACC_CTRL_REG1_A 0x20
#define LSM303_ACC_POWER_DOWN 0x00
#define LSM303_ACC_1HZ 0x10
#define LSM303_ACC_10HZ 0x20
#define LSM303_ACC_25HZ 0x30
#define LSM303_ACC_50HZ 0x40
#define LSM303_ACC_100HZ 0x50
#define LSM303_ACC_200HZ 0x60
#define LSM303_ACC_400HZ 0x70
#define LSM303_ACC_1620HZ 0x80 //only Low-power mode
#define LSM303_ACC_1344HZ_NORMAL_OR_5376HZ_LOW 0x90
#define LSM303_ACC_XYZ_ENABLE 0x07
#define LSM303_ACC_X_ENABLE 0x01
#define LSM303_ACC_Y_ENABLE 0x02
#define LSM303_ACC_Z_ENABLE 0x04
#define LSM303_ACC_LOW_POWER 0x08

//CTRL_REG2_A = [HPM1][HPM0][HPCF2][HPCF1][FDS][HPCLICK][HPIS2][HPIS1]
#define LSM303_ACC_CTRL_REG2_A 0x21
//Some high pass filter options.

//CTRL_REG3_A = [I1_CLICK][I1_AOI1][[I1_AOI2][I1_DRDY1][I1_DRDY2][I1_WTM][I1_OVERRUN][--]
#define LSM303_ACC_CTRL_REG3_A 0x22
//Some INT1 and FIFO options.

//CTRL_REG4_A = [BDU][BLE][FS1][FS0][HR][0][0][SIM]
#define LSM303_ACC_CTRL_REG4_A 0x23
#define LSM303_ACC_HIGH_RES_OUT 0x08
#define LSM303_ACC_2G 0x00
#define LSM303_ACC_4G 0x10
#define LSM303_ACC_8G 0x20
#define LSM303_ACC_16G 0x30
#define LSM303_ACC_SPI_3_WIRE 0x01 //otherwise 4-wire SPI

//CTRL_REG5_A = [BOOT][FIFO_EN][--][--][LIR_INT1][D4D_INT1][LIR_INT2][D4D_INT2]
#define LSM303_ACC_CTRL_REG5_A 0x24
#define LSM303_ACC_REBOOT_MEMORY 0x80
#define LSM303_ACC_FIFO_ENABLE 0x40
//Some Latch interrput and 4D (?) enable.

//CTRL_REG6_A = [I2_CLICKen][I2_INT1][I2_INT2][BOOT_I1][P2_ACT][--][H_LACTIVE][--]
#define LSM303_ACC_CTRL_REG6_A 0x25
//Some interrupts and reboot

//REFERENCE/DATACAPTURE_A = [Ref7][Ref6][Ref5][Ref4][Ref3][Ref2][Ref1][Ref0]
#define LSM303_ACC_REFERENCE_DATACAPTURE_A 0x26

//STATUS_REG_A = [ZYXOR][ZOR][YOR][XOR][ZYXDA][ZDA][YDA][XDA]
#define LSM303_ACC_STATUS_REG_A 0x27
//Some data overrun and...
#define LSM303_ACC_ZYXDA 0x08 //X, Y, Z axis new data avaiable
#define LSM303_ACC_ZDA 0x04 //Z axis new data avaiable
#define LSM303_ACC_YDA 0x02 //Y axis new data avaiable
#define LSM303_ACC_XDA 0x01 //X axis new data avaiable

//OUT REGISTERS
#define LSM303_ACC_OUT_X_L_A 0x28
#define LSM303_ACC_OUT_X_H_A 0x29
#define LSM303_ACC_OUT_Y_L_A 0x2A
#define LSM303_ACC_OUT_Y_H_A 0x2B
#define LSM303_ACC_OUT_Z_L_A 0x2C
#define LSM303_ACC_OUT_Z_H_A 0x2D
#define LSM303_ACC_OUT_Z_MULTIREAD_A (LSM303_ACC_OUT_Z_L_A | 0x80)
#define LSM303_ACC_OUT_XYZ_MULTIREAD_A (LSM303_ACC_OUT_X_L_A | 0x80) //0x80 from manual to enable read from 2 bytes

#define LSM303_ACC_RESOLUTION_2G 2.0
#define LSM303_ACC_RESOLUTION_4G 4.0
#define LSM303_ACC_RESOLUTION_8G 8.0
#define LSM303_ACC_RESOLUTION_16G 16.0

#define LSM303_ACC_SCALE 2.0f

// Gyro macros
#define L3GD20_WHO_AM_I				         0xD4
#define L3GD20_REGISTER_WHO_AM_I             0x0F   // 11010100   r
#define L3GD20_REGISTER_CTRL_REG1            0x20   // 00000111   rw
#define L3GD20_REGISTER_CTRL_REG2            0x21   // 00000000   rw
#define L3GD20_REGISTER_CTRL_REG3            0x22   // 00000000   rw
#define L3GD20_REGISTER_CTRL_REG4            0x23   // 00000000   rw
#define L3GD20_REGISTER_CTRL_REG5            0x24   // 00000000   rw
#define L3GD20_REGISTER_REFERENCE            0x25   // 00000000   rw
#define L3GD20_REGISTER_OUT_TEMP             0x26   //            r
#define L3GD20_REGISTER_STATUS_REG           0x27   //            r
#define L3GD20_REGISTER_OUT_X_L              0x28   //            r
#define L3GD20_REGISTER_OUT_X_H              0x29   //            r
#define L3GD20_REGISTER_OUT_Y_L              0x2A   //            r
#define L3GD20_REGISTER_OUT_Y_H              0x2B   //            r
#define L3GD20_REGISTER_OUT_Z_L              0x2C   //            r
#define L3GD20_REGISTER_OUT_Z_H              0x2D   //            r
#define L3GD20_REGISTER_FIFO_CTRL_REG        0x2E   // 00000000   rw
#define L3GD20_REGISTER_FIFO_SRC_REG         0x2F   //            r
#define L3GD20_REGISTER_INT1_CFG             0x30   // 00000000   rw
#define L3GD20_REGISTER_INT1_SRC             0x31   //            r
#define L3GD20_REGISTER_TSH_XH               0x32   // 00000000   rw
#define L3GD20_REGISTER_TSH_XL               0x33   // 00000000   rw
#define L3GD20_REGISTER_TSH_YH               0x34   // 00000000   rw
#define L3GD20_REGISTER_TSH_YL               0x35   // 00000000   rw
#define L3GD20_REGISTER_TSH_ZH               0x36   // 00000000   rw
#define L3GD20_REGISTER_TSH_ZL               0x37   // 00000000   rw
#define L3GD20_REGISTER_INT1_DURATION        0x38   // 00000000   rw

#define L3GD20_SENSITIVITY_250		0.00875	/* 8.75 mdps/digit */
#define L3GD20_SENSITIVITY_500		0.0175	/* 17.5 mdps/digit */
#define L3GD20_SENSITIVITY_2000		0.070	/* 70 mdps/digit */
//#define GYRO_FS_DPS 250

#define CS_LOW HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 0)
#define CS_HIGH HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 1)

#define RAD_TO_DEG 180/M_PI
#endif /* DEFINES_H_ */
