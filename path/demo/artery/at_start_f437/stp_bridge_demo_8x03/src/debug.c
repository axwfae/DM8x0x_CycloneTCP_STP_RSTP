/**
 * @file debug.c
 * @brief Debugging facilities
 *
 * @section License
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Copyright (C) 2010-2023 Oryx Embedded SARL. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * @author Oryx Embedded SARL (www.oryx-embedded.com)
 * @version 2.3.0
 **/

//Dependencies
#include "at32f435_437.h"
#include "debug.h"


/**
 * @brief Debug UART initialization
 * @param[in] baudrate UART baudrate
 **/

void debugInit(uint32_t baudrate)
{
   gpio_init_type gpio_init_struct;

   //Enable GPIOA clock
   crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
   //Enable USART1 clock
   crm_periph_clock_enable(CRM_USART1_PERIPH_CLOCK, TRUE);

   //Get default parameters
   gpio_default_para_init(&gpio_init_struct);

   //Configure USART1_TX (PA9)
   gpio_init_struct.gpio_pins = GPIO_PINS_9;
   gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
   gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
   gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
   gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
   gpio_init(GPIOA, &gpio_init_struct);

   //Configure USART1_RX (PA10)
   gpio_init_struct.gpio_pins = GPIO_PINS_10;
   gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
   gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
   gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
   gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
   gpio_init(GPIOA, &gpio_init_struct);

   //Remap USART1 pins
   gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE9, GPIO_MUX_7);
   gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE10, GPIO_MUX_7);

   //Configure USART1
   usart_init(USART1, baudrate, USART_DATA_8BITS, USART_STOP_1_BIT);
   usart_transmitter_enable(USART1, TRUE);
   usart_enable(USART1, TRUE);
}


/**
 * @brief Display the contents of an array
 * @param[in] stream Pointer to a FILE object that identifies an output stream
 * @param[in] prepend String to prepend to the left of each line
 * @param[in] data Pointer to the data array
 * @param[in] length Number of bytes to display
 **/

void debugDisplayArray(FILE *stream,
   const char_t *prepend, const void *data, size_t length)
{
   uint_t i;

   for(i = 0; i < length; i++)
   {
      //Beginning of a new line?
      if((i % 16) == 0)
         fprintf(stream, "%s", prepend);
      //Display current data byte
      fprintf(stream, "%02" PRIX8 " ", *((uint8_t *) data + i));
      //End of current line?
      if((i % 16) == 15 || i == (length - 1))
         fprintf(stream, "\r\n");
   }
}


/**
 * @brief Write character to stream
 * @param[in] c The character to be written
 * @param[in] stream Pointer to a FILE object that identifies an output stream
 * @return On success, the character written is returned. If a writing
 *   error occurs, EOF is returned
 **/

int_t fputc(int_t c, FILE *stream)
{
   //Standard output or error output?
   if(stream == stdout || stream == stderr)
   {
      //Wait for the transmitter to be ready
      while(usart_flag_get(USART1, USART_TDBE_FLAG) == RESET)
      {
      }

      //Send character
      usart_data_transmit(USART1, c);

      //On success, the character written is returned
      return c;
   }
   //Unknown output?
   else
   {
      //If a writing error occurs, EOF is returned
      return EOF;
   }
}
