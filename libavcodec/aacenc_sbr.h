/*
 * AAC HE-AAC v1 (SBR) Encoder
 *
 * Copyright (c) 2026 Nils Schimmelmann
 *
 * Based on the SBR encoder of FAAC, by the same author.
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
#include "put_bits.h"

typedef struct AACEncSBRContext AACEncSBRContext;

/**
 * Delay in input samples of the encoder side, in addition to the 2 * 1024
 * samples of the core codec: the 2:1 decimation filter. This is what goes in
 * the priming of the container, as for the other HE-AAC encoders: the delay
 * of the QMF banks of the SBR decoder (2 * 481 samples at the full rate) is
 * left out, decoders add it themselves.
 */
#define AAC_SBR_ENC_EXTRA_DELAY AAC_SBR_DECIM_DELAY
#define AAC_SBR_DECIM_DELAY     48

/**
 * How much older than the frame the core is coding the SBR envelopes are
 * analysed, in full rate samples, so that they line up with the core signal
 * after the decoder's QMF delays. Calibrated against the FFmpeg and FAAD2
 * decoders using a high-band noise burst.
 */
#define AAC_SBR_ANALYSIS_DELAY  2464

/** Size of the per-channel full rate input history kept for HE-AAC. */
#define AAC_SBR_FULL_BUF_SIZE   8192

AACEncSBRContext *ff_aac_sbr_enc_init(int sample_rate, int64_t bit_rate,
                                      int channels, int full_rate_idx);
void ff_aac_sbr_enc_close(AACEncSBRContext *s_ctx);

void ff_aac_sbr_enc_resample(AACEncSBRContext *s_ctx, int num_channels,
                             int frame_len, float **input_samples, float **core_samples);

void ff_aac_sbr_enc_process_frame(AACEncSBRContext *s_ctx, int num_channels,
                                  int frame_len, float **input_samples);

/**
 * Write the SBR FIL element of one SCE/CPE; pb may be NULL to only get its
 * size in bits. Must be called after ff_aac_sbr_enc_process_frame().
 */
int ff_aac_sbr_enc_write_payload(AACEncSBRContext *s_ctx, PutBitContext *pb,
                                 int elem_type, int ch0);

int ff_aac_sbr_enc_get_full_rate_idx(const AACEncSBRContext *s_ctx);

/** Frequency (Hz) above which the decoder regenerates the spectrum. */
int ff_aac_sbr_enc_get_crossover(const AACEncSBRContext *s_ctx);

#endif /* AVCODEC_AACENC_SBR_H */
