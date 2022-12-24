/*
 * mirage_spi.h
 *
 *  Created on: Sep 13, 2022
 *      Author: Zero
 *      Stored under Mirage V1.2>Custom>Inc
 */

#ifndef STM32F1XX_HAL_DRIVER_INC_MIRAGE_SPI_H_
#define STM32F1XX_HAL_DRIVER_INC_MIRAGE_SPI_H_

//#define TLC5947_XLAT_PORT GPIOA
//#define TLC5947_BLANK_PORT GPIOA
//#define BOARD_LED_PORT GPIOC
// TODO ESTO ESTÁ EN MAIN.H
//#define TLC5947_BLANK_Pin GPIO_PIN_1
//#define TLC5947_XLAT_Pin GPIO_PIN_3
//#define BOARD_LED_PIN GPIO_PIN_13


#define TLC5947_CHANNELS 24 //Cantidad de canales a escribir por cada tlc5947
#define TLC5947_DRIVER_AMOUNT 1 //Cantidad de tlc5947 que uso
#define TOTAL_CHANNELS (TLC5947_CHANNELS*TLC5947_DRIVER_AMOUNT)
#define SPI_BYTE_AMOUNT (TLC5947_CHANNELS*12*TLC5947_DRIVER_AMOUNT/8) //Para solo una placa tengo que enviar 12 bits x 24 canales/8 bits = 36 Bytes

#define RED 0
#define GREEN 1
#define BLUE 2
#define RGB 3


#endif /* STM32F1XX_HAL_DRIVER_INC_MIRAGE_SPI_H_ */
