/*
 * AAC HE-AAC v1 (SBR) Encoder
 *
 * Copyright (c) 2026 Nils Schimmelmann
 * Ported to FFmpeg 2026
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

#ifndef AVCODEC_AACENC_SBR_H
#define AVCODEC_AACENC_SBR_H

#include <stdint.h>
#include "avcodec.h"
#include "put_bits.h"

typedef struct AACEncSBRContext AACEncSBRContext;

enum {
    FIXFIX = 0,
    FIXVAR = 1,
    VARFIX = 2,
    VARVAR = 3,
};

AACEncSBRContext *ff_aac_sbr_enc_init(AVCodecContext *avctx, int channels, int sample_rate, int64_t bit_rate);
void ff_aac_sbr_enc_close(AACEncSBRContext *s_ctx);

void ff_aac_sbr_enc_process_frame(AACEncSBRContext *s_ctx, int num_channels, const int *is_lfe,
                                  int frame_len, float **input_samples, float **core_samples);

int ff_aac_sbr_enc_write_payload(AACEncSBRContext *s_ctx, PutBitContext *pb,
                                 int elem_type, int ch0);

int ff_aac_sbr_enc_get_full_rate_idx(const AACEncSBRContext *s_ctx);
void ff_aac_sbr_enc_set_full_rate_idx(AACEncSBRContext *s_ctx, int idx);

#endif /* AVCODEC_AACENC_SBR_H */
