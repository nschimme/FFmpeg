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

#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "libavutil/mem.h"
#include "libavutil/mathematics.h"
#include "aacenc_sbr.h"
#include "aacsbrdata.h"
#include "put_bits.h"

/* 31-tap halfband FIR anti-aliasing filter for 2:1 decimation */
static const float fir_halfband[16] = {
    0.50000000f,
    0.31557008f,  0.00000000f, -0.10006240f,  0.00000000f,  0.05260193f,
    0.00000000f, -0.03022513f,  0.00000000f,  0.01712431f,  0.00000000f,
   -0.00888206f,  0.00000000f,  0.00398687f,  0.00000000f, -0.00130986f
};

typedef struct SBRHuffEntry {
    uint32_t code : 24;
    uint32_t len  : 8;
} SBRHuffEntry;

#define F_HUFF_ENV_1_5DB_OFFSET  60
#define F_HUFF_ENV_1_5DB_NSYMS   121
#define F_HUFF_ENV_3_0DB_OFFSET  31
#define F_HUFF_ENV_3_0DB_NSYMS   63

/* Canonical SBR Huffman tables derived from FFmpeg sbr_huffman_tab */
static const SBRHuffEntry f_huff_env_1_5dB[121] = {
    {0x7ffe7, 19},    {0x7ffe8, 19},    {0xfffd2, 20},    {0xfffd3, 20},    {0xfffd4, 20},    {0xfffd5, 20},    {0xfffd6, 20},    {0xfffd7, 20},
    {0xfffd8, 20},    {0x7ffda, 19},    {0xfffd9, 20},    {0xfffda, 20},    {0xfffdb, 20},    {0xfffdc, 20},    {0x7ffdb, 19},    {0xfffdd, 20},
    {0x7ffdc, 19},    {0x7ffdd, 19},    {0xfffde, 20},    {0x3ffe4, 18},    {0xfffdf, 20},    {0xfffe0, 20},    {0xfffe1, 20},    {0x7ffde, 19},
    {0xfffe2, 20},    {0xfffe3, 20},    {0xfffe4, 20},    {0x7ffdf, 19},    {0xfffe5, 20},    {0x7ffe0, 19},    {0x3ffe8, 18},    {0x7ffe1, 19},
    {0x3ffe0, 18},    {0x3ffe9, 18},    {0x1ffef, 17},    {0x3ffe5, 18},    {0x1ffec, 17},    {0x1ffed, 17},    {0x1ffee, 17},    {0x0fff4, 16},
    {0x0fff3, 16},    {0x0fff0, 16},    {0x07ff7, 15},    {0x07ff6, 15},    {0x03ffa, 14},    {0x01ffa, 13},    {0x01ff9, 13},    {0x00ffa, 12},
    {0x00ff8, 12},    {0x007f9, 11},    {0x003fb, 10},    {0x001fc, 9},    {0x001fa, 9},    {0x000fb, 8},    {0x0007c, 7},    {0x0003c, 6},
    {0x0001c, 5},    {0x0000c, 4},    {0x00005, 3},    {0x00001, 2},    {0x00000, 2},    {0x00004, 3},    {0x0000d, 4},    {0x0001d, 5},
    {0x0003d, 6},    {0x000fa, 8},    {0x000fc, 8},    {0x001fb, 9},    {0x003fa, 10},    {0x007f8, 11},    {0x007fa, 11},    {0x007fb, 11},
    {0x00ff9, 12},    {0x00ffb, 12},    {0x01ff8, 13},    {0x01ffb, 13},    {0x03ff8, 14},    {0x03ff9, 14},    {0x0fff1, 16},    {0x0fff2, 16},
    {0x1ffea, 17},    {0x1ffeb, 17},    {0x3ffe1, 18},    {0x3ffe2, 18},    {0x3ffea, 18},    {0x3ffe3, 18},    {0x3ffe6, 18},    {0x3ffe7, 18},
    {0x3ffeb, 18},    {0xfffe6, 20},    {0x7ffe2, 19},    {0xfffe7, 20},    {0xfffe8, 20},    {0xfffe9, 20},    {0xfffea, 20},    {0xfffeb, 20},
    {0xfffec, 20},    {0x7ffe3, 19},    {0xfffed, 20},    {0xfffee, 20},    {0xfffef, 20},    {0xffff0, 20},    {0x7ffe4, 19},    {0xffff1, 20},
    {0x3ffec, 18},    {0xffff2, 20},    {0xffff3, 20},    {0x7ffe5, 19},    {0x7ffe6, 19},    {0xffff4, 20},    {0xffff5, 20},    {0xffff6, 20},
    {0xffff7, 20},    {0xffff8, 20},    {0xffff9, 20},    {0xffffa, 20},    {0xffffb, 20},    {0xffffc, 20},    {0xffffd, 20},    {0xffffe, 20},
    {0xfffff, 20}
};

static const SBRHuffEntry f_huff_env_3_0dB[63] = {
    {0x05ff7, 13},    {0x05ff8, 13},    {0x05ff9, 13},    {0x05ffa, 13},    {0x05ffb, 13},    {0x0bff8, 14},    {0x0bff9, 14},    {0x017fc, 11},
    {0x002fe, 8},    {0x0017e, 7},    {0x0002e, 4},    {0x0000a, 2},    {0x00004, 1},    {0x00016, 3},    {0x0005e, 5},    {0x000be, 6},
    {0x005fe, 9},    {0x02ffa, 12},    {0x05ff6, 13},    {0x0bffa, 14},    {0x0bffb, 14},    {0x0bffc, 14},    {0x0bffd, 14},    {0x0bffe, 14},
    {0x0bfff, 14},    {0x003fd, 10},    {0x001fd, 9},    {0x000fd, 8},    {0x0003e, 6},    {0x0000e, 4},    {0x00002, 2},    {0x00000, 1},
    {0x00006, 3},    {0x0001e, 5},    {0x000fc, 8},    {0x001fc, 9},    {0x003fc, 10},    {0x007fc, 11},    {0x00ffc, 12},    {0x01ffc, 13},
    {0x03ffa, 14},    {0x07ff9, 15},    {0x07ffa, 15},    {0x0fff8, 16},    {0x0fff9, 16},    {0x1fff6, 17},    {0x1fff7, 17},    {0x3fff5, 18},
    {0x3fff6, 18},    {0x3fff1, 18},    {0xffff8, 20},    {0x7fff1, 19},    {0x7fff2, 19},    {0x7fff3, 19},    {0xffff9, 20},    {0x7fff7, 19},
    {0x7fff4, 19},    {0xffffa, 20},    {0xffffb, 20},    {0xffffc, 20},    {0xffffd, 20},    {0xffffe, 20},    {0xfffff, 20}
};

static int compute_kx(int sample_rate, int bs_start_freq)
{
    int temp = (sample_rate < 32000) ? 3000 : (sample_rate < 64000) ? 4000 : 5000;
    int start_min = ((temp << 7) + (sample_rate >> 1)) / sample_rate;
    int row = (sample_rate <= 16000) ? 0 : (sample_rate <= 22050) ? 1 : (sample_rate <= 24000) ? 2 : (sample_rate <= 32000) ? 3 : (sample_rate <= 64000) ? 4 : 5;
    return av_clip(start_min + sbr_offset[row][bs_start_freq & 15], 1, 63);
}

static int cmp_int_fn(const void *a, const void *b) { return *(const int *)a - *(const int *)b; }
static int cmp_int16_fn(const void *a, const void *b) { return (int)(*(const int16_t *)a) - (int)(*(const int16_t *)b); }

static int compute_k2(int sample_rate, int bs_stop_freq)
{
    if (bs_stop_freq == 14 || bs_stop_freq == 15) return 64;
    int temp = (sample_rate < 32000) ? 3000 : (sample_rate < 64000) ? 4000 : 5000;
    int stop_min = ((temp << 8) + (sample_rate >> 1)) / sample_rate;
    int k2;
    if (bs_stop_freq < 14) {
        int16_t stop_dk[13];
        float prod = (float)stop_min;
        int prev = stop_min;
        float base = powf(64.0f / (float)stop_min, (float)(1.0f / 13.0f));
        for (int i = 0; i < 12; i++) {
            prod *= base;
            int present = (int)lrintf(prod);
            stop_dk[i] = (int16_t)(present - prev);
            prev = present;
        }
        stop_dk[12] = (int16_t)(64 - prev);
        qsort(stop_dk, 13, sizeof(int16_t), cmp_int16_fn);
        k2 = stop_min;
        for (int i = 0; i < bs_stop_freq; i++) k2 += stop_dk[i];
    } else {
        k2 = 64;
    }

    return k2;
}

static void build_freq_table(SBRInfo *sbr)
{
    int kx = sbr->kx, k2 = sbr->k2;
    int *edges = sbr->band_edges;
    int n_master;

    int prev = kx;
    int bands_per_octave = 14 - 2 * sbr->bs_freq_scale;
    n_master = 2 * (int)(bands_per_octave * log2f((float)k2 / (float)kx) / 2.0f + 0.5f);
    n_master = av_clip(n_master, 1, SBR_MAX_BANDS);
    for (int k = 0; k < n_master; k++) {
        int edge = (int)(kx * powf((float)k2 / (float)kx, (float)(k + 1) / (float)n_master) + 0.5f);
        edges[1 + k] = edge - prev;
        prev = edge;
    }
    qsort(edges + 1, n_master, sizeof(int), cmp_int_fn);
    edges[0] = kx;
    for (int k = 1; k <= n_master; k++) edges[k] += edges[k - 1];
    sbr->num_bands = n_master;

    int n_low = (n_master + 1) >> 1;
    sbr->num_bands_low = n_low;
    int off = n_master & 1;
    for (int k = 0; k <= n_low; k++) {
        int idx = 2 * k - off;
        if (idx < 0) idx = 0;
        sbr->band_edges_low[k] = edges[idx];
    }
}

static inline int sbr_env_bands(const SBRInfo *sbr, const SbrFrameData *fd)
{
    return fd->freq_res ? sbr->num_bands : sbr->num_bands_low;
}

static inline const int *sbr_env_edges(const SBRInfo *sbr, const SbrFrameData *fd)
{
    return fd->freq_res ? sbr->band_edges : sbr->band_edges_low;
}

static void sbr_qmf_analysis(AACSBREncContext *s_ctx, SBRInfo *sbr, const float *ovl_pos, float *energy, int kx, int k2)
{
    AVComplexFloat x[64], y[64];
    const INTFLOAT *p0 = sbr_qmf_window_us;

    for (int m = 0; m < 64; m++) {
        int n0 = 2 * m;
        float a = p0[0]   * ovl_pos[639 - n0]
                    + p0[128] * ovl_pos[511 - n0]
                    + p0[256] * ovl_pos[383 - n0]
                    + p0[384] * ovl_pos[255 - n0]
                    + p0[512] * ovl_pos[127 - n0];
        float b = p0[1]   * ovl_pos[638 - n0]
                    + p0[129] * ovl_pos[510 - n0]
                    + p0[257] * ovl_pos[382 - n0]
                    + p0[385] * ovl_pos[254 - n0]
                    + p0[513] * ovl_pos[126 - n0];
        x[m].re = a * sbr->twid_cos[m] - b * sbr->twid_sin[m];
        x[m].im = -(a * sbr->twid_sin[m] + b * sbr->twid_cos[m]);
        p0 += 2;
    }

    s_ctx->fft_fn(s_ctx->fft_ctx, y, x, sizeof(AVComplexFloat));

    for (int k = kx; k < k2; k++) {
        int kr = 63 - k;
        float Ar = 0.5f * (y[k].re + y[kr].re);
        float Ai = 0.5f * (y[kr].im - y[k].im);
        float Br = -0.5f * (y[k].im + y[kr].im);
        float Bi = 0.5f * (y[kr].re - y[k].re);
        float wr = sbr->odd_cos[k];
        float wi = sbr->odd_sin[k];
        float Sr = Ar + wr * Br - wi * Bi;
        float Si = Ai + wr * Bi + wi * Br;
        energy[k] += Sr * Sr + Si * Si;
    }
}

static inline int sbr_env_of_slot(int num_envelopes, const int *env_start, int slot)
{
    int e = 0;
    while (e + 1 < num_envelopes && slot >= env_start[e + 1]) e++;
    return e;
}

static void sbr_analyze(AACSBREncContext *s_ctx, SignalAnalysis *sa, float **full_ptrs, int nch, const int *is_lfe, int num_samples, SBRInfo *sbr)
{
    int num_slots = num_samples / SBR_QMF_BANDS_64;
    sa->num_slots = num_slots;
    sa->sampled = num_slots;

    for (int ch = 0; ch < nch; ch++) {
        sa->ch[ch].transient_slot = -1;
        sa->ch[ch].transient_strength = 0.0f;
    }

    int trans_slot = -1;
    for (int ch = 0; ch < nch; ch++) {
        if (is_lfe && is_lfe[ch]) continue;
        if (sa->ch[ch].transient_slot >= 0 && sa->ch[ch].transient_strength > 2.0f) {
            trans_slot = sa->ch[ch].transient_slot;
            break;
        }
    }

    if (trans_slot >= 0) {
        sa->frame_class = SBR_FRAME_CLASS_VARFIX;
        sa->num_envelopes = 2;
        int border = av_clip(trans_slot, 2, num_slots - 2);
        sa->t_env[0] = 0;
        sa->t_env[1] = border;
        sa->t_env[2] = num_slots;
        sa->bs_pointer = 0;
    } else {
        sa->frame_class = SBR_FRAME_CLASS_FIXFIX;
        sa->num_envelopes = sbr->num_env_fixfix;
        sa->bs_pointer = 0;
        for (int i = 0; i <= sa->num_envelopes; i++)
            sa->t_env[i] = (i * num_slots) / sa->num_envelopes;
    }

    for (int e = 0; e < sa->num_envelopes; e++)
        sa->env_sampled[e] = sa->t_env[e + 1] - sa->t_env[e];

    for (int ch = 0; ch < nch; ch++) {
        if (is_lfe && is_lfe[ch]) continue;
        for (int e = 0; e < sa->num_envelopes; e++)
            memset(sa->band_e[ch][e], 0, SBR_QMF_BANDS_64 * sizeof(float));

        float ovl_buf[SBR_QMF_HIST_LEN];
        memcpy(ovl_buf, sbr->ch[ch].qmf_ovl64, SBR_QMF_HIST_LEN * sizeof(float));

        for (int s = 0; s < num_slots; s++) {
            memmove(ovl_buf, ovl_buf + SBR_QMF_BANDS_64, (SBR_QMF_HIST_LEN - SBR_QMF_BANDS_64) * sizeof(float));
            memcpy(ovl_buf + SBR_QMF_HIST_LEN - SBR_QMF_BANDS_64, full_ptrs[ch] + s * SBR_QMF_BANDS_64, SBR_QMF_BANDS_64 * sizeof(float));

            int e = sbr_env_of_slot(sa->num_envelopes, sa->t_env, s);
            sbr_qmf_analysis(s_ctx, sbr, ovl_buf, sa->band_e[ch][e], sbr->kx, sbr->k2);
        }
    }
}

static void sbr_adopt_envelope_grid(const SBRInfo *sbr, const SignalAnalysis *sa, SbrFrameData *fd)
{
    fd->num_envelopes = sa->num_envelopes;
    fd->frame_class   = sa->frame_class;
    fd->bs_pointer    = sa->bs_pointer;
    for (int i = 0; i <= sa->num_envelopes; i++) fd->t_env[i] = sa->t_env[i];
    fd->eff_amp_res = (fd->num_envelopes == 1) ? 0 : SBR_AMP_RES;
    fd->freq_res = sbr->bs_freq_res;
}

static void sbr_quantize_envelopes(const SBRInfo *sbr, int nch, const int *is_lfe,
                                   const SignalAnalysis *sa, SbrFrameData *fd)
{
    int n_env = fd->num_envelopes;
    int nb = sbr_env_bands(sbr, fd);
    const int *edges = sbr_env_edges(sbr, fd);

    for (int ch = 0; ch < nch; ch++) {
        if (is_lfe && is_lfe[ch]) continue;
        const float (*band_e)[SBR_QMF_BANDS_64] = sa->band_e[ch];
        int dlav = fd->eff_amp_res ? SBR_ENV_DELTA_LIMIT_HIRES : SBR_ENV_DELTA_LIMIT_LORES;
        for (int e = 0; e < n_env; e++) {
            int prev_level = -1;
            for (int b = 0; b < nb; b++) {
                int k_lo = edges[b], k_hi = edges[b+1];
                int e_slots = sa->env_sampled[e];
                if (e_slots < 1) e_slots = 1;
                float E = 0;
                for (int k = k_lo; k < k_hi; k++) E += band_e[e][k];
                E = (E * 1073741824.0f) / (float)(e_slots * (k_hi - k_lo));
                float factor = fd->eff_amp_res ? 1.0f : 2.0f;
                int level = lrintf(factor * (log2f(E + SBR_LOG_ENERGY_FLOOR) - SBR_ENV_LEVEL_LOG2_OFFSET));
                int max_val = fd->eff_amp_res ? 63 : 127;
                int raw_level = av_clip(level, 0, max_val);
                if (prev_level < 0) {
                    raw_level = av_clip(raw_level, 0, max_val);
                    fd->ch[ch].env_data[e][b] = raw_level;
                    prev_level = raw_level;
                } else {
                    int delta = av_clip(raw_level - prev_level, -dlav, dlav);
                    delta = av_clip(prev_level + delta, 0, max_val) - prev_level;
                    fd->ch[ch].env_data[e][b] = delta;
                    prev_level += delta;
                }
            }
        }
    }
}

static void sbr_encode(SBRInfo *sbr, float **time_domain, int num_channels, const int *is_lfe, int num_samples, SignalAnalysis *sa, SbrFrameData *fd)
{
    for (int ch = 0; ch < num_channels; ch++)
        if (!is_lfe || !is_lfe[ch])
            memcpy(sbr->ch[ch].qmf_ovl64, time_domain[ch] + num_samples - SBR_QMF_HIST_LEN, SBR_QMF_HIST_LEN * sizeof(float));

    sbr_adopt_envelope_grid(sbr, sa, fd);
    sbr_quantize_envelopes(sbr, num_channels, is_lfe, sa, fd);
}

static void sbr_frame_silence(SbrFrameData *fd)
{
    fd->num_envelopes = 1;
    fd->eff_amp_res  = 0;
    fd->frame_class   = SBR_FRAME_CLASS_FIXFIX;
    fd->t_env[0]      = 0;
    fd->t_env[1]      = 32;
    fd->bs_pointer    = 0;
    fd->freq_res      = 1;
    for (int ch = 0; ch < 16; ch++) {
        for (int e = 0; e < SBR_MAX_ENVELOPES; e++) {
            for (int b = 0; b < SBR_MAX_BANDS; b++)
                fd->ch[ch].env_data[e][b] = 0;
        }
    }
}

static void write_sbr_header(const SBRInfo *sbr, PutBitContext *pb)
{
    put_bits(pb, 1, SBR_AMP_RES);
    put_bits(pb, 4, sbr->bs_start_freq);
    put_bits(pb, 4, sbr->bs_stop_freq);
    put_bits(pb, 3, sbr->bs_xover_band);
    put_bits(pb, 2, 0);
    put_bits(pb, 1, 1);
    put_bits(pb, 1, 0);
    put_bits(pb, 2, sbr->bs_freq_scale);
    put_bits(pb, 1, sbr->bs_alter_scale);
    put_bits(pb, 2, 0);
}

static const int sbr_ceil_log2[] = { 0, 1, 2, 2, 3, 3 };

static void write_sbr_grid(const SBRInfo *sbr, const SbrFrameData *fd, PutBitContext *pb)
{
    int num_env = fd->num_envelopes;
    put_bits(pb, 2, fd->frame_class);
    if (fd->frame_class == SBR_FRAME_CLASS_VARFIX) {
        put_bits(pb, 2, fd->t_env[0]);
        put_bits(pb, 2, num_env - 1);
        for (int i = 0; i < num_env - 1; i++)
            put_bits(pb, 2, (fd->t_env[i + 1] - fd->t_env[i] - 2) / 2);
        int ptr_len = sbr_ceil_log2[num_env];
        put_bits(pb, ptr_len, fd->bs_pointer);
        for (int i = 0; i < num_env; i++)
            put_bits(pb, 1, sbr->bs_freq_res);
    } else {
        put_bits(pb, 2, num_env > 1 ? 1 : 0);
        put_bits(pb, 1, sbr->bs_freq_res);
    }
}

static void write_sbr_dtdf(const SbrFrameData *fd, PutBitContext *pb)
{
    int n_q = fd->num_envelopes > 1 ? 2 : 1;
    int len = fd->num_envelopes + n_q;
    put_bits(pb, len, 0);
}

static void write_sbr_invf(PutBitContext *pb)
{
    put_bits(pb, 2, SBR_INVF_MODE);
}

static void put_huff(PutBitContext *pb, const SBRHuffEntry *table, int nsyms, int offset, int delta)
{
    int sym = av_clip(delta + offset, 0, nsyms - 1);
    put_bits(pb, table[sym].len, table[sym].code);
}

static void write_sbr_envelope(const SBRInfo *sbr, const SbrFrameData *fd, PutBitContext *pb, int ch)
{
    const SBRHuffEntry *table = fd->eff_amp_res ? f_huff_env_3_0dB : f_huff_env_1_5dB;
    int nsyms = fd->eff_amp_res ? F_HUFF_ENV_3_0DB_NSYMS : F_HUFF_ENV_1_5DB_NSYMS;
    int offset = fd->eff_amp_res ? F_HUFF_ENV_3_0DB_OFFSET : F_HUFF_ENV_1_5DB_OFFSET;
    int first_bits = fd->eff_amp_res ? 6 : 7;
    int first_max = (1 << first_bits) - 1;
    int nb = sbr_env_bands(sbr, fd);

    for (int e = 0; e < fd->num_envelopes; e++) {
        const int *env_ch = fd->ch[ch].env_data[e];
        put_bits(pb, first_bits, av_clip(env_ch[0], 0, first_max));
        for (int b = 1; b < nb; b++)
            put_huff(pb, table, nsyms, offset, env_ch[b]);
    }
}

static void write_sbr_noise(const SbrFrameData *fd, PutBitContext *pb)
{
    int n_q = fd->num_envelopes > 1 ? 2 : 1;
    for (int ne = 0; ne < n_q; ne++)
        put_bits(pb, 5, SBR_NOISE_LEVEL_DEFAULT);
}

static void write_sbr_data(const SBRInfo *sbr, const SbrFrameData *fd, PutBitContext *pb, int elem_type, int ch0)
{
    int nch = (elem_type == TYPE_CPE) ? 2 : 1;

    if (elem_type == TYPE_CPE) {
        put_bits(pb, 1, 0); // bs_data_extra = 0
        put_bits(pb, 1, 0); // bs_coupling = 0
    } else {
        put_bits(pb, 1, 0); // bs_data_extra = 0
    }

    for (int ch = 0; ch < nch; ch++)
        write_sbr_grid(sbr, fd, pb);
    for (int ch = 0; ch < nch; ch++)
        write_sbr_dtdf(fd, pb);
    for (int ch = 0; ch < nch; ch++)
        write_sbr_invf(pb);
    for (int ch = 0; ch < nch; ch++)
        write_sbr_envelope(sbr, fd, pb, ch0 + ch);
    for (int ch = 0; ch < nch; ch++)
        write_sbr_noise(fd, pb);

    for (int ch = 0; ch < nch; ch++)
        put_bits(pb, 1, 0); // bs_add_harmonic_flag = 0

    put_bits(pb, 1, 0); // bs_extended_data = 0
}

static void emit_sbr_payload(const SBRInfo *sbr, const SbrFrameData *fd, PutBitContext *pb, int elem_type, int ch0, int send_header)
{
    put_bits(pb, 4, 13); /* EXT_SBR_DATA (13 = 0xd) */
    put_bits(pb, 1, send_header & 1);
    if (send_header)
        write_sbr_header(sbr, pb);
    write_sbr_data(sbr, fd, pb, elem_type, ch0);
}

int ff_aac_sbr_enc_write_payload(AACSBREncContext *s_ctx, PutBitContext *pb, int elem_type, int ch0)
{
    if (!s_ctx || !s_ctx->sbr_info || !s_ctx->sbr_info->sbr_present) return 0;

    SBRInfo *sbr = s_ctx->sbr_info;
    const SbrFrameData *fd = &s_ctx->frame_fifo[s_ctx->frame_head];

    if (!sbr->header_decided) {
        sbr->send_header_this_frame = (sbr->frame_count++ % SBR_HEADER_PERIOD == 0);
        sbr->header_decided = 1;
    }

    PutBitContext pb_tmp;
    uint8_t tmp_buf[1024];
    init_put_bits(&pb_tmp, tmp_buf, sizeof(tmp_buf));
    emit_sbr_payload(sbr, fd, &pb_tmp, elem_type, ch0, sbr->send_header_this_frame);
    int payload_bits = put_bits_count(&pb_tmp);

    int fill_bytes = (payload_bits + 7) / 8;
    int pad_bits = fill_bytes * 8 - payload_bits;

    put_bits(pb, 3, TYPE_FIL);
    if (fill_bytes < 15) {
        put_bits(pb, 4, fill_bytes);
    } else {
        put_bits(pb, 4, 15);
        put_bits(pb, 8, fill_bytes - 14);
    }

    emit_sbr_payload(sbr, fd, pb, elem_type, ch0, sbr->send_header_this_frame);
    if (pad_bits > 0)
        put_bits(pb, pad_bits, 0);

    return 0;
}

static SBRInfo *sbr_init(int channels, int sample_rate, int64_t bit_rate)
{
    SBRInfo *sbr_info = av_mallocz(sizeof(SBRInfo));
    if (!sbr_info) return NULL;

    sbr_info->num_channels = channels;
    sbr_info->sample_rate  = sample_rate;
    sbr_info->sbr_present  = 1;

    sbr_info->bs_freq_scale = 2;
    sbr_info->bs_alter_scale = 1;
    sbr_info->bs_start_freq = 5;
    sbr_info->bs_stop_freq  = 0;
    sbr_info->bs_xover_band = 0;
    sbr_info->bs_freq_res   = 1;
    sbr_info->num_env_fixfix   = 1;

    sbr_info->kx = compute_kx(sample_rate, sbr_info->bs_start_freq);
    sbr_info->k2 = compute_k2(sample_rate, sbr_info->bs_stop_freq);

    build_freq_table(sbr_info);

    for (int m = 0; m < SBR_QMF_BANDS_64; m++) {
        sbr_info->twid_cos[m] = (float)cos(M_PI * m / 64.0);
        sbr_info->twid_sin[m] = (float)sin(M_PI * m / 64.0);
        sbr_info->odd_cos[m]  = (float)cos(M_PI * (2 * m + 1) / 128.0);
        sbr_info->odd_sin[m]  = (float)sin(M_PI * (2 * m + 1) / 128.0);
    }

    return sbr_info;
}

AACSBREncContext *ff_aac_sbr_enc_init(int channels, int sample_rate, int64_t bit_rate)
{
    AACSBREncContext *s_ctx = av_mallocz(sizeof(AACSBREncContext));
    if (!s_ctx) return NULL;

    s_ctx->full_sample_rate    = sample_rate;
    s_ctx->full_sample_rate_idx = 0;
    s_ctx->sbr_info           = sbr_init(channels, sample_rate, bit_rate);
    if (!s_ctx->sbr_info) {
        av_free(s_ctx);
        return NULL;
    }

    float scale = 1.0f;
    if (av_tx_init(&s_ctx->fft_ctx, &s_ctx->fft_fn, AV_TX_FLOAT_FFT, 0, 64, &scale, 0) < 0) {
        av_free(s_ctx->sbr_info);
        av_free(s_ctx);
        return NULL;
    }

    for (int i = 0; i < SBR_FRAME_FIFO; i++)
        sbr_frame_silence(&s_ctx->frame_fifo[i]);

    return s_ctx;
}

void ff_aac_sbr_enc_close(AACSBREncContext *s_ctx)
{
    if (!s_ctx) return;
    if (s_ctx->fft_ctx)
        av_tx_uninit(&s_ctx->fft_ctx);
    if (s_ctx->sbr_info)
        av_free(s_ctx->sbr_info);
    av_free(s_ctx);
}

void ff_aac_sbr_enc_process_frame(AACSBREncContext *s_ctx, int num_channels, const int *is_lfe,
                                  int frame_len, float **input_samples, float **core_samples)
{
    if (!s_ctx) return;

    /* 31-tap anti-aliasing FIR halfband filter for 2:1 decimation */
    for (int ch = 0; ch < num_channels; ch++) {
        float *in = input_samples[ch];
        float *out = core_samples[ch];
        int max_idx = 2 * frame_len;
        for (int i = 0; i < frame_len; i++) {
            int idx = 2 * i;
            float sum = fir_halfband[0] * in[idx];
            for (int k = 1; k < 16; k++) {
                float s0 = (idx - k >= 0) ? in[idx - k] : in[0];
                float s1 = (idx + k < max_idx) ? in[idx + k] : in[max_idx - 1];
                sum += fir_halfband[k] * (s0 + s1);
            }
            out[i] = sum;
        }
    }

    s_ctx->frame_head = (s_ctx->frame_head + 1) % SBR_FRAME_FIFO;
    SbrFrameData *fd = &s_ctx->frame_fifo[s_ctx->frame_head];
    s_ctx->sbr_info->header_decided = 0;

    sbr_analyze(s_ctx, &s_ctx->signal_analysis, input_samples, num_channels, is_lfe, 2 * frame_len, s_ctx->sbr_info);
    sbr_encode(s_ctx->sbr_info, input_samples, num_channels, is_lfe, 2 * frame_len, &s_ctx->signal_analysis, fd);
}
