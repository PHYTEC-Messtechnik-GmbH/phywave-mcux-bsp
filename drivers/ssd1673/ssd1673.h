/*
 * Copyright (c) 2017 PHYTEC Messtechnik GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the PHYTEC Messtechnik GmbH nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @defgroup  drivers_ssd1673 SSD1673 Display Driver
 * @ingroup   drivers_display
 * @brief     Active Matrix EPD 150x250 Display Driver with Controller
 *
 * @{
 *
 * @file
 * @brief     Interface definition for the SSD1673 display driver.
 */

#ifndef SSD1673_H
#define SSD1673_H

#include <stdint.h>
#include "fsl_dspi_freertos.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Physical resolution in the X direction */
#define SSD1673_XRES  122
/** Physical resolution in the Y direction */
#define SSD1673_YRES  250

/**
 * To be used with ssd1673_set_orientation().
 */
typedef enum {
	/** Normal orientation, displays content the same as any other
	 * components on the board. */
	SSD1673_ORIENTATION_NORMAL,
	/** Upside down orientation, displays content upside down */
	SSD1673_ORIENTATION_UPSIDE_DOWN
} ssd1673_orientation_t;

/**
 * Initialize the display with a given SPI handle.
 *
 * @param handle  the SPI handle
 */
int ssd1673_init(void);

/**
 * Initialize a full display update.
 */
void ssd1673_set_initial_update(void);

/**
 * Execute a full display update.
 *
 * This will actually update the display with the content loaded into its RAM.
 * Be sure that you have called ssd1673_set_initial_update() before.
 */
void ssd1673_do_initial_update(void);

/**
 * Write data into the display RAM.
 *
 * @param data  the raw pixel data, where 1 byte equals 8 pixels (black/white)
 * @param len  the length of data
 */
size_t ssd1673_write_ram(uint8_t *data, size_t len);

/**
 * Initialize a partial update.
 */
/* void ssd1673_set_part_update(void); */

/**
 * Execute a partial update.
 *
 * This will actually partially update the display with the content given by
 * data. Be sure that you have called ssd1673_set_part_update() before.
 *
 * @param sx  start in X direction
 * @param ex  end in X direcction
 * @param sy  start in Y direction
 * @param ey  end in Y direction
 * @param data  the raw pixel data, where 1 byte equals 8 pixels (black/white)
 */
/* void ssd1673_update_partial(size_t sx, size_t ex, size_t sy, size_t ey,
		uint8_t *data); */
/**
 * Set the orientation of the display.
 *
 * @param orientation  SSD1673_ORIENTATION_NORMAL displays content the same as
 *                     any other components on the board,
 *                     SSD1673_ORIENTATION_UPSIDE_DOWN displays content upside
 *                     down
 */
void ssd1673_set_orientation(ssd1673_orientation_t orientation);

#ifdef __cplusplus
}
#endif

#endif /* SSD1673_H */
/** @} */
