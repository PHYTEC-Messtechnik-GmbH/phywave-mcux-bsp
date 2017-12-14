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
 * @defgroup  drivers_yafg Yet Another Font Generator
 * @ingroup   drivers_display
 * @brief     Font and Framebuffer generation for monochrome displays
 *
 * @{
 *
 * @file
 * @brief     Interface definition for the YAFG.
 */

#ifndef YAFG_H
#define YAFG_H

#include <stdint.h>
#include <stdlib.h>

/**
 * A smaller picture object containing up to 64 bytes of data.
 */
struct yafg_pic {
	/** The width of a picture in pixels */
	uint8_t width;
	/** The height of a picture in pixels */
	uint8_t height;
	/** The raw data of a picture */
	uint8_t data[64];
};

/**
 * A framebuffer object containing the complete data of one frame.
 */
struct yafg_framebuffer {
	/** The width of a framebuffer in pixels */
	uint8_t width;
	/** The height of a framebuffer in pixels */
	uint8_t height;
	/** The raw data of a framebuffer */
	uint8_t data[4000];
};

/**
 * Draw a yafg_pic object to the framebuffer.
 *
 * @param object  the object data pixels to draw
 * @param x  X size of the object
 * @param y  Y size of the object
 */
int yafg_draw(const struct yafg_pic *object, size_t x, size_t y);

/**
 * Get the internal framebuffer.
 *
 * @return  a pointer to the internal framebuffer.
 */
struct yafg_framebuffer * yafg_get_framebuffer(void);

/**
 * Reverse all bytes of the internal framebuffer.
 */
void yafg_framebuffer_reverse_bytes(void);

/**
 * Invert all pixels of the internal framebuffer.
 */
void yafg_framebuffer_invert(void);

/**
 * Mirror the internal framebuffer at the X axis.
 */
void yafg_framebuffer_mirror_x(void);

/**
 * Mirror the internal framebuffer at the Y axis.
 */
void yafg_framebuffer_mirror_y(void);

/**
 * Print a string onto the framebuffer.
 *
 * The font used has 10 pixels wide and 16 pixels tall bitmap glyphs.
 *
 * @param str  the string to print
 * @param row  the row on display the string is printed to (1 row = 16 pixels)
 * @param x  the offset in X direction of the beginning of the string
 * @param spacing  the spacing in pixels between each glyph
 */
void yafg_print(char *str, size_t row, size_t x, size_t spacing);

#endif /* YAFG_H */
/** @} */
