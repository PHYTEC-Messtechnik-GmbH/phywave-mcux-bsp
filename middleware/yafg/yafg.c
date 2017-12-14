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
 * @ingroup  drivers_yafg
 * @{
 *
 * @file
 * @brief    Implemenation of the YAFG.
 *
 * @}
 */

#include "yafg.h"
#include "yafg_glyphs.h"
#include <string.h>

static struct yafg_framebuffer fb = {
	.width = 250,
	.height = 128,
	.data = {0}
};
static struct yafg_pic glyph_not_found = {
	.width = 10,
	.height = 16,
	.data = {
		0xff, 0xff, 0x07, 0xe0, 0x19, 0x98, 0x21, 0x84,
		0xc1, 0x83, 0xc1, 0x83, 0x21, 0x84, 0x19, 0x98,
		0x07, 0xe0, 0xff, 0xff
	}
};

/* local -------------------------------------------------------------------- */

static uint8_t draw_char(char c, uint8_t x, uint8_t y);
static struct yafg_pic * lookup_char(uint8_t c);

/* global ------------------------------------------------------------------- */

int yafg_draw(const struct yafg_pic *object, size_t x, size_t y)
{
	if (object == NULL) {
		return -1;
	}

	if (object->height % 8 || y % 8) {
		return -2;
	}

	for (size_t i = 0; i < object->width; i++) {
		for (size_t k = 0; k < object->height / 8; k++) {
			fb.data[(i + x) * (fb.height)/8 + y/8 + k] =
				object->data[i * (object->height / 8) + k];
		}
	}

	return 0;
}

struct yafg_framebuffer * yafg_get_framebuffer(void)
{
	return &fb;
}

void yafg_framebuffer_reverse_bytes(void)
{
	for (size_t i = 0; i < fb.width * fb.height / 8; i++) {
		fb.data[i] = (fb.data[i] & 0xf0) >> 4 | (fb.data[i] & 0x0f) << 4;
		fb.data[i] = (fb.data[i] & 0xcc) >> 2 | (fb.data[i] & 0x33) << 2;
		fb.data[i] = (fb.data[i] & 0xaa) >> 1 | (fb.data[i] & 0x55) << 1;
	}
}

void yafg_framebuffer_invert(void)
{
	for (size_t i = 0; i < fb.width * fb.height / 8; i++) {
		fb.data[i] = ~fb.data[i];
	}
}

void yafg_framebuffer_mirror_x(void)
{
	uint8_t tmp;

	for (size_t i = 0; i < fb.width / 2; i++) {
		for (size_t k = 0; k < fb.height / 8; k++) {
			tmp = fb.data[i * (fb.height)/8 + k];
			fb.data[i * (fb.height/8) + k] =
				fb.data[(fb.width - i - 1) * (fb.height)/8 + k];
			fb.data[(fb.width - i - 1) * (fb.height)/8 + k] = tmp;
		}
	}
}

void yafg_framebuffer_mirror_y(void)
{
	uint8_t tmp;

	for (size_t i = 0; i < fb.width; i++) {
		for (size_t k = 0; k < fb.height / 16; k++) {
			tmp = fb.data[i * (fb.height/8) + k];
			fb.data[i * (fb.height/8) + k] =
				fb.data[i * (fb.height/8) + (fb.height/8) - 1 - k];
			fb.data[i * (fb.height/8) + (fb.height/8) - 1 - k] = tmp;
		}
	}
}

void yafg_print(char *str, size_t row, size_t x, size_t spacing)
{
	uint8_t pos = x;

	for (size_t i = 0; i < strlen(str); i++) {
		pos += spacing + draw_char(str[i], pos, row * 16);
	}
}

/* local -------------------------------------------------------------------- */

struct yafg_pic * lookup_char(uint8_t c)
{
	if (c < YAFG_GLYPHS_FIRST_CHAR || c > YAFG_GLYPHS_LAST_CHAR) {
		return &glyph_not_found;
	}

	return &yafg_glyphs[c - YAFG_GLYPHS_FIRST_CHAR];
}

uint8_t draw_char(char c, uint8_t x, uint8_t y)
{
	struct yafg_pic *pic = lookup_char(c);

	if (pic == NULL) {
		return 0;
	}

	yafg_draw(pic, x, y);
	return pic->width;
}
