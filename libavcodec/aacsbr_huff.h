/*
 * AAC SBR Huffman tables shared by the decoder and the encoder
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#ifndef AVCODEC_AACSBR_HUFF_H
#define AVCODEC_AACSBR_HUFF_H

#include <stdint.h>

/** {symbol, length} pairs of the ten SBR envelope/noise Huffman tables. */
extern const uint8_t ff_aac_sbr_huffman_tab[][2];
/** Number of codes in each of the ten tables. */
extern const uint8_t ff_aac_sbr_huffman_nb_codes[10];
/** Symbol offset of each of the ten tables. */
extern const int8_t  ff_aac_sbr_vlc_offsets[10];

#endif /* AVCODEC_AACSBR_HUFF_H */
