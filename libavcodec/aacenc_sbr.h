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
#include <stdbool.h>
#include "avcodec.h"
#include "put_bits.h"
#include "aac.h"
#include "libavutil/tx.h"

#define SBR_QMF_BANDS_64     64
#define SBR_QMF_OVL_LEN_64   576
#define SBR_QMF_HIST_LEN     (SBR_QMF_BANDS_64 + SBR_QMF_OVL_LEN_64)

#define SBR_MAX_ENVELOPES    2
#define SBR_MAX_BANDS        48
#define SBR_HEADER_PERIOD    10

#define SBR_AMP_RES          1
#define SBR_INVF_MODE        0
#define SBR_NOISE_LEVEL_DEFAULT 0
#define SBR_ENV_DELTA_LIMIT_HIRES 12
#define SBR_ENV_DELTA_LIMIT_LORES 24
#define SBR_ENV_LEVEL_LOG2_OFFSET 6.0f
#define SBR_LOG_ENERGY_FLOOR 1e-10f

#define LOOKAHEAD_DEPTH 2
#define SBR_FRAME_FIFO (LOOKAHEAD_DEPTH + 2)

typedef enum SbrFrameClass {
    SBR_FRAME_CLASS_FIXFIX = 0,
    SBR_FRAME_CLASS_FIXVAR = 1,
    SBR_FRAME_CLASS_VARFIX = 2,
    SBR_FRAME_CLASS_VARVAR = 3
} SbrFrameClass;

typedef struct SignalAnalysisChannel {
    int   transient_slot;
    float transient_strength;
} SignalAnalysisChannel;

typedef struct SignalAnalysis {
    int num_slots;
    int sampled;

    SbrFrameClass frame_class;
    int num_envelopes;
    int t_env[SBR_MAX_ENVELOPES + 1];
    int bs_pointer;
    int env_sampled[SBR_MAX_ENVELOPES];

    SignalAnalysisChannel ch[16];
    float band_e[16][SBR_MAX_ENVELOPES][SBR_QMF_BANDS_64];
} SignalAnalysis;

typedef struct SBRChannel {
    float qmf_ovl64[SBR_QMF_HIST_LEN];
} SBRChannel;

typedef struct SbrFrameData {
    int num_envelopes;
    int eff_amp_res;
    SbrFrameClass frame_class;
    int t_env[SBR_MAX_ENVELOPES + 1];
    int bs_pointer;
    int freq_res;
    struct {
        int env_data[SBR_MAX_ENVELOPES][SBR_MAX_BANDS];
    } ch[16];
} SbrFrameData;

typedef struct SBRInfo {
    int sbr_present;
    int frame_count;
    int num_channels;
    int sample_rate;

    int kx;
    int k2;
    int num_bands;
    int band_edges[SBR_MAX_BANDS + 1];
    int num_bands_low;
    int band_edges_low[SBR_MAX_BANDS + 1];

    int bs_freq_res;
    int bs_start_freq;
    int bs_stop_freq;
    int bs_xover_band;
    int bs_alter_scale;
    int bs_freq_scale;
    int num_env_fixfix;

    int header_decided;
    int send_header_this_frame;

    SBRChannel ch[16];

    float twid_cos[SBR_QMF_BANDS_64];
    float twid_sin[SBR_QMF_BANDS_64];
    float odd_cos [SBR_QMF_BANDS_64];
    float odd_sin [SBR_QMF_BANDS_64];
} SBRInfo;

typedef struct AACSBREncContext {
    int full_sample_rate;
    int full_sample_rate_idx;
    SBRInfo *sbr_info;

    AVTXContext *fft_ctx;
    av_tx_fn fft_fn;

    SignalAnalysis signal_analysis;
    SbrFrameData frame_fifo[SBR_FRAME_FIFO];
    int frame_head;
} AACSBREncContext;

AACSBREncContext *ff_aac_sbr_enc_init(int channels, int sample_rate, int64_t bit_rate);
void ff_aac_sbr_enc_close(AACSBREncContext *s_ctx);

void ff_aac_sbr_enc_process_frame(AACSBREncContext *s_ctx, int num_channels, const int *is_lfe,
                                  int frame_len, float **input_samples, float **core_samples);

int ff_aac_sbr_enc_write_payload(AACSBREncContext *s_ctx, PutBitContext *pb,
                                 int elem_type, int ch0);

#endif /* AVCODEC_AACENC_SBR_H */
