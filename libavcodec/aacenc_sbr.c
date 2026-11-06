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

#include <limits.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "libavutil/mem.h"
#include "libavutil/mathematics.h"
#include "libavutil/tx.h"
#include "aacenc_sbr.h"
#include "aac.h"
#include "aacsbr_huff.h"
#include "aacsbrdata.h"
#include "sbr.h"
#include "put_bits.h"

#define SBR_QMF_BANDS_64     64
#define SBR_QMF_OVL_LEN_64   576
#define SBR_QMF_HIST_LEN     (SBR_QMF_BANDS_64 + SBR_QMF_OVL_LEN_64)

#define SBR_MAX_ENVELOPES    4
#define SBR_MAX_BANDS        48

#define SBR_DECIM_HALF       AAC_SBR_DECIM_DELAY
#define SBR_DECIM_TAPS       (2 * SBR_DECIM_HALF + 1)
#define SBR_DECIM_BETA       8.0

/* Only FIXFIX frame class is generated */
#define SBR_FRAME_FIXFIX     0
#define SBR_HEADER_PERIOD    10

#define SBR_AMP_RES          1
/* The noise floor is a constant. A high index (little noise to add) keeps the
 * high band shaped in time and across channels by the low band it is copied
 * from: with no index the band is nearly all noise, which smears attacks into
 * pre-echo and decorrelates the channels. Inverse filtering is a constant per
 * rate too: it tames the tonality of the copy, more so at the higher rates. */
#define SBR_NOISE_LEVEL_DEFAULT 18
#define SBR_INVF_MODE_LOW_RATE  1
#define SBR_INVF_MODE_HIGH_RATE 3
#define SBR_INVF_HIGH_RATE_BPS  20000u
#define SBR_ENV_DELTA_LIMIT_HIRES 31
#define SBR_ENV_DELTA_LIMIT_LORES 60
#define SBR_ENV_LEVEL_LOG2_OFFSET 6.0f
#define SBR_LOG_ENERGY_FLOOR 1e-10f

#define SBR_MIN_SPAN 4
#define SBR_FOUR_ENV_BITRATE_BPS        17000u
#define SBR_FREQ_SCALE_COARSE_BPS       12000u
#define SBR_STOP_FREQ_MIN               0
#define SBR_STOP_FREQ_MAX               13
/* Frequency where the decoder starts regenerating the spectrum. Between 6 and
 * 11 kHz it matters little on the HE-AAC scenarios of the faac-benchmark suite
 * (a few hundredths of a point of score, about 1 dB of pre-echo); about 6 kHz
 * below 32 kbps per channel and 11 kHz above scored best of the fixed choices
 * and level with the best of them for the score, on clips that were not used to
 * pick it too. These are targets in Hz, as the same index lands at a different
 * frequency for each sample rate. */
#define SBR_CROSSOVER_LOW_HZ            6300
#define SBR_CROSSOVER_HIGH_HZ           11000
#define SBR_CROSSOVER_HIGH_BPS          32000u
#define SBR_TWO_REGION_MIN_RATE         44100
#define SBR_TWO_REGION_MAX_RATE         48000
#define SBR_ONE_REGION_NUM              22449
#define SBR_ONE_REGION_DEN              10000
#define SBR_TWO_REGION_RATIO            2.2449f
#define SBR_UPPER_REGION_WARP           1.3f
#define SBR_NARROW_STOP_BPS             10000u
#define SBR_NARROW_STOP_MIN_RATE        32000
#define SBR_STOP_FREQ_TARGET_NARROW_HZ  14000
#define SBR_STOP_FREQ_TARGET_HZ         20000

typedef struct SBRHuffEntry {
    uint32_t code : 24;
    uint32_t len  : 8;
} SBRHuffEntry;

typedef struct AACEncSignalAnalysis {
    int num_slots;

    int num_envelopes;
    int t_env[SBR_MAX_ENVELOPES + 1];
    int env_sampled[SBR_MAX_ENVELOPES];

    float band_e[16][SBR_MAX_ENVELOPES][SBR_QMF_BANDS_64];
} AACEncSignalAnalysis;

typedef struct AACEncSBRChannel {
    float qmf_ovl64[SBR_QMF_HIST_LEN];

    /* last envelope written, the reference of time-differential coding */
    int prev_env[SBR_MAX_BANDS];
    int prev_nb;
    int prev_amp_res;
    int prev_valid;
} AACEncSBRChannel;

typedef struct AACEncSBRFrameData {
    int num_envelopes;
    int eff_amp_res;
    int t_env[SBR_MAX_ENVELOPES + 1];
    int freq_res;
    struct {
        int env_data[SBR_MAX_ENVELOPES][SBR_MAX_BANDS];
    } ch[16];
} AACEncSBRFrameData;

typedef struct AACEncSBRInfo {
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
    int invf_mode;
    int bs_alter_scale;
    int bs_freq_scale;
    int num_env_fixfix;

    int header_decided;
    int send_header_this_frame;

    AACEncSBRChannel ch[16];

    float twid_cos[SBR_QMF_BANDS_64];
    float twid_sin[SBR_QMF_BANDS_64];
    float odd_cos [SBR_QMF_BANDS_64];
    float odd_sin [SBR_QMF_BANDS_64];
} AACEncSBRInfo;

struct AACEncSBRContext {
    int full_sample_rate;
    int full_sample_rate_idx;
    AACEncSBRInfo *sbr_info;

    AVTXContext *fft_ctx;
    av_tx_fn fft_fn;

    float fir[SBR_DECIM_TAPS];  /* 2:1 half-band decimation filter */

    SBRHuffEntry huff_env_1_5dB[121];   /* frequency-differential */
    SBRHuffEntry huff_env_3_0dB[63];
    SBRHuffEntry huff_env_t_1_5dB[121]; /* time-differential */
    SBRHuffEntry huff_env_t_3_0dB[63];
    SBRHuffEntry huff_noise_t[63];

    AACEncSignalAnalysis signal_analysis;
    AACEncSBRFrameData cur_frame_data;
};

#define F_HUFF_ENV_1_5DB_OFFSET  60
#define F_HUFF_ENV_1_5DB_NSYMS   121
#define F_HUFF_ENV_3_0DB_OFFSET  31
#define F_HUFF_ENV_3_0DB_NSYMS   63
#define T_HUFF_NOISE_OFFSET      31
#define T_HUFF_NOISE_NSYMS       63

static void build_sbr_huff_table(SBRHuffEntry *out, const uint8_t (*tab)[2], int nb_codes)
{
    uint32_t code = 0;
    for (int i = 0; i < nb_codes; i++) {
        int sym = tab[i][0];
        int len = tab[i][1];
        if (len > 0) {
            out[sym].code = code >> (32 - len);
            out[sym].len  = len;
            code += 1U << (32 - len);
        }
    }
}

static int compute_kx(int sample_rate, int bs_start_freq)
{
    int temp = (sample_rate < 32000) ? 3000 : (sample_rate < 64000) ? 4000 : 5000;
    int start_min = ((temp << 7) + (sample_rate >> 1)) / sample_rate;
    int row = (sample_rate <= 16000) ? 0 : (sample_rate <= 22050) ? 1 : (sample_rate <= 24000) ? 2 : (sample_rate <= 32000) ? 3 : (sample_rate <= 64000) ? 4 : 5;
    return av_clip(start_min + sbr_offset[row][bs_start_freq & 15], 1, 63);
}

/**
 * The bs_start_freq whose crossover is closest to target_hz, the target being
 * capped at the Nyquist frequency of the core signal.
 */
static int pick_start_freq(int sample_rate, int target_hz)
{
    int best = 0, best_err = INT_MAX;

    target_hz = FFMIN(target_hz, sample_rate / 4);
    for (int sf = 0; sf < 16; sf++) {
        int err = FFABS(compute_kx(sample_rate, sf) * sample_rate / (2 * SBR_QMF_BANDS_64) - target_hz);
        if (err < best_err) {
            best_err = err;
            best     = sf;
        }
    }
    return best;
}

static int cmp_int_fn(const void *a, const void *b) { return *(const int *)a - *(const int *)b; }
static int cmp_int16_fn(const void *a, const void *b) { return (int)(*(const int16_t *)a) - (int)(*(const int16_t *)b); }

static int max_sbr_span(int sample_rate)
{
    return (sample_rate <= 32000) ? 48 : (sample_rate <= 44100) ? 35 : 32;
}

static int pick_stop_freq(int sample_rate, int kx, int target_hz, int *k2_out)
{
    int temp = (sample_rate < 32000) ? 3000 : (sample_rate < 64000) ? 4000 : 5000;
    int stop_min = ((temp << 8) + (sample_rate >> 1)) / sample_rate;
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

    int two_region = sample_rate >= SBR_TWO_REGION_MIN_RATE && sample_rate <= SBR_TWO_REGION_MAX_RATE;
    int best = SBR_STOP_FREQ_MIN, best_k2 = stop_min, k2 = stop_min;
    for (int sf = SBR_STOP_FREQ_MIN; sf <= SBR_STOP_FREQ_MAX; sf++) {
        if (sf > 0) k2 += stop_dk[sf - 1];
        if ((!two_region && k2 * SBR_ONE_REGION_DEN > SBR_ONE_REGION_NUM * kx) || k2 - kx > max_sbr_span(sample_rate)) break;
        best = sf;
        best_k2 = k2;
        if (k2 - kx >= SBR_MIN_SPAN && (int64_t)k2 * sample_rate / (2 * SBR_QMF_BANDS_64) >= target_hz) break;
    }
    *k2_out = best_k2;
    return best;
}

static int build_log_band_widths(int k_start, int k_end, int bands_per_octave, float warp, int *vdk)
{
    int nb = 2 * (int)lrintf(bands_per_octave * log2f((float)k_end / (float)k_start) / (2.0f * warp));
    nb = av_clip(nb, 1, SBR_MAX_BANDS);
    int prev = k_start;
    int i = 0;
    do {
        int cur = (int)lrintf(k_start * powf((float)k_end / (float)k_start, (float)(i + 1) / (float)nb));
        vdk[i] = cur - prev;
        prev = cur;
    } while (++i < nb);
    qsort(vdk, nb, sizeof(int), cmp_int_fn);
    return nb;
}

static void build_freq_table(AACEncSBRInfo *sbr)
{
    int kx = sbr->kx, k2 = sbr->k2;
    int *edges = sbr->band_edges;
    int bands_per_octave = 14 - 2 * sbr->bs_freq_scale;
    int two_region = (float)k2 / (float)kx > SBR_TWO_REGION_RATIO;
    int k1 = two_region ? 2 * kx : k2;
    int widths[2 * SBR_MAX_BANDS];
    int n_master = build_log_band_widths(kx, k1, bands_per_octave, 1.0f, widths);

    if (two_region) {
        int *upper = widths + n_master;
        float warp = sbr->bs_alter_scale ? SBR_UPPER_REGION_WARP : 1.0f;
        int nb1 = build_log_band_widths(k1, k2, bands_per_octave, warp, upper);
        if (upper[0] < widths[n_master - 1]) {
            int change = widths[n_master - 1] - upper[0];
            int half = (upper[nb1 - 1] - upper[0]) / 2;
            if (change > half) change = half;
            upper[0] += change;
            upper[nb1 - 1] -= change;
            qsort(upper, nb1, sizeof(int), cmp_int_fn);
        }
        n_master += nb1;
    }

    edges[0] = kx;
    for (int i = 0; i < n_master; i++) edges[i + 1] = edges[i] + widths[i];
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

static inline int sbr_env_bands(const AACEncSBRInfo *sbr, const AACEncSBRFrameData *fd)
{
    return fd->freq_res ? sbr->num_bands : sbr->num_bands_low;
}

static inline const int *sbr_env_edges(const AACEncSBRInfo *sbr, const AACEncSBRFrameData *fd)
{
    return fd->freq_res ? sbr->band_edges : sbr->band_edges_low;
}

static void sbr_qmf_analysis(AACEncSBRContext *s_ctx, AACEncSBRInfo *sbr, const float *ovl_pos, float *energy, int kx, int k2)
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

static void sbr_analyze(AACEncSBRContext *s_ctx, AACEncSignalAnalysis *sa, float **full_ptrs, int nch, int num_samples, AACEncSBRInfo *sbr)
{
    int num_slots = num_samples / SBR_QMF_BANDS_64;
    sa->num_slots = num_slots;

    sa->num_envelopes = sbr->num_env_fixfix;
    for (int i = 0; i <= sa->num_envelopes; i++)
        sa->t_env[i] = (i * num_slots) / sa->num_envelopes;

    for (int e = 0; e < sa->num_envelopes; e++)
        sa->env_sampled[e] = sa->t_env[e + 1] - sa->t_env[e];

    for (int ch = 0; ch < nch; ch++) {
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

static void sbr_adopt_envelope_grid(const AACEncSBRInfo *sbr, const AACEncSignalAnalysis *sa, AACEncSBRFrameData *fd)
{
    fd->num_envelopes = sa->num_envelopes;
    for (int i = 0; i <= sa->num_envelopes; i++) fd->t_env[i] = sa->t_env[i];
    fd->eff_amp_res = (fd->num_envelopes == 1) ? 0 : SBR_AMP_RES;
    fd->freq_res = sbr->bs_freq_res;
}

static void sbr_quantize_envelopes(const AACEncSBRInfo *sbr, int nch,
                                   const AACEncSignalAnalysis *sa, AACEncSBRFrameData *fd)
{
    int n_env = fd->num_envelopes;
    int nb = sbr_env_bands(sbr, fd);
    const int *edges = sbr_env_edges(sbr, fd);

    for (int ch = 0; ch < nch; ch++) {
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
                /* 2^31: full scale 16-bit energy convention, with the factor 2 of the
                 * one-sided QMF analysis output */
                E = (E * 2147483648.0f) / (float)(e_slots * (k_hi - k_lo));
                float factor = fd->eff_amp_res ? 1.0f : 2.0f;
                int level = lrintf(factor * (log2f(E + SBR_LOG_ENERGY_FLOOR) - SBR_ENV_LEVEL_LOG2_OFFSET));
                int max_val = fd->eff_amp_res ? 63 : 127;
                int raw_level = av_clip(level, 0, max_val);
                /* keep the frequency-differential chain codable */
                if (prev_level >= 0)
                    raw_level = av_clip(raw_level, prev_level - dlav, prev_level + dlav);
                fd->ch[ch].env_data[e][b] = prev_level = raw_level;
            }
        }
    }
}

static void sbr_encode(AACEncSBRInfo *sbr, float **time_domain, int num_channels, int num_samples, AACEncSignalAnalysis *sa, AACEncSBRFrameData *fd)
{
    for (int ch = 0; ch < num_channels; ch++)
        memcpy(sbr->ch[ch].qmf_ovl64, time_domain[ch] + num_samples - SBR_QMF_HIST_LEN, SBR_QMF_HIST_LEN * sizeof(float));

    sbr_adopt_envelope_grid(sbr, sa, fd);
    sbr_quantize_envelopes(sbr, num_channels, sa, fd);
}

static void sbr_frame_silence(AACEncSBRFrameData *fd)
{
    fd->num_envelopes = 1;
    fd->eff_amp_res  = 0;
    fd->t_env[0]      = 0;
    fd->t_env[1]      = 32;
    fd->freq_res      = 1;
    for (int ch = 0; ch < 16; ch++) {
        for (int e = 0; e < SBR_MAX_ENVELOPES; e++) {
            for (int b = 0; b < SBR_MAX_BANDS; b++)
                fd->ch[ch].env_data[e][b] = 0;
        }
    }
}

static void write_sbr_header(const AACEncSBRInfo *sbr, PutBitContext *pb)
{
    put_bits(pb, 1, SBR_AMP_RES);
    put_bits(pb, 4, sbr->bs_start_freq);
    put_bits(pb, 4, sbr->bs_stop_freq);
    put_bits(pb, 3, sbr->bs_xover_band);
    put_bits(pb, 2, 0);                  /* reserved */
    put_bits(pb, 1, 1);                  /* bs_header_extra_1 */
    put_bits(pb, 1, 1);                  /* bs_header_extra_2 */
    put_bits(pb, 2, sbr->bs_freq_scale);
    put_bits(pb, 1, sbr->bs_alter_scale);
    put_bits(pb, 2, 0);                  /* bs_noise_bands */
    /* Limiter bands 2, gains 3, interpolation and smoothing on. The default
     * gain limit of the decoder (2) stops it from amplifying a high band
     * copied from a low band that is weak or empty up to the requested
     * envelope: the band stays empty. */
    put_bits(pb, 2, 2);
    put_bits(pb, 2, 3);
    put_bits(pb, 1, 1);
    put_bits(pb, 1, 1);
}

static void write_sbr_grid(const AACEncSBRInfo *sbr, const AACEncSBRFrameData *fd, PutBitContext *pb)
{
    put_bits(pb, 2, SBR_FRAME_FIXFIX);
    put_bits(pb, 2, av_log2(fd->num_envelopes)); /* bs_num_env is log2 */
    put_bits(pb, 1, sbr->bs_freq_res);
}

static void write_sbr_invf(const AACEncSBRInfo *sbr, PutBitContext *pb)
{
    put_bits(pb, 2, sbr->invf_mode);
}

static void put_huff(PutBitContext *pb, const SBRHuffEntry *table, int nsyms, int offset, int delta)
{
    int sym = av_clip(delta + offset, 0, nsyms - 1);
    put_bits(pb, table[sym].len, table[sym].code);
}

/** Per-frame coding decisions of one channel. */
typedef struct SBREnvCoding {
    int dt_mask;        /* bit e set: envelope e is time-differential */
    int noise_linked;   /* first noise floor is coded against the previous frame */
} SBREnvCoding;

/**
 * Cost in bits of envelope values against ref, or INT_MAX if some delta has
 * no code.
 */
static int env_cost(const SBRHuffEntry *table, int nsyms, int offset, const int *v,
                    const int *ref, int nb, int first_bits)
{
    int bits = ref ? 0 : first_bits;

    for (int b = ref ? 0 : 1; b < nb; b++) {
        int sym = v[b] - (ref ? ref[b] : v[b - 1]) + offset;
        if (sym < 0 || sym >= nsyms || !table[sym].len)
            return INT_MAX;
        bits += table[sym].len;
    }
    return bits;
}

static SBREnvCoding choose_env_coding(const AACEncSBRContext *s_ctx, const AACEncSBRInfo *sbr,
                                      const AACEncSBRFrameData *fd, int ch, int send_header)
{
    const AACEncSBRChannel *c = &sbr->ch[ch];
    const int hi = fd->eff_amp_res;
    const SBRHuffEntry *ft = hi ? s_ctx->huff_env_3_0dB : s_ctx->huff_env_1_5dB;
    const SBRHuffEntry *tt = hi ? s_ctx->huff_env_t_3_0dB : s_ctx->huff_env_t_1_5dB;
    const int nsyms = hi ? F_HUFF_ENV_3_0DB_NSYMS : F_HUFF_ENV_1_5DB_NSYMS;
    const int offset = hi ? F_HUFF_ENV_3_0DB_OFFSET : F_HUFF_ENV_1_5DB_OFFSET;
    const int first_bits = hi ? 6 : 7;
    const int nb = sbr_env_bands(sbr, fd);
    /* header frames stay self-contained so decoding can start there */
    SBREnvCoding r = { 0, !send_header && c->prev_valid };
    int linked = r.noise_linked && c->prev_nb == nb && c->prev_amp_res == hi;

    for (int e = 0; e < fd->num_envelopes; e++) {
        const int *v = fd->ch[ch].env_data[e];
        int cf = env_cost(ft, nsyms, offset, v, NULL, nb, first_bits);
        if (e || linked) {
            const int *ref = e ? fd->ch[ch].env_data[e - 1] : c->prev_env;
            if (env_cost(tt, nsyms, offset, v, ref, nb, first_bits) < cf)
                r.dt_mask |= 1 << e;
        }
    }
    return r;
}

static void write_sbr_dtdf(const AACEncSBRFrameData *fd, const SBREnvCoding *cod, PutBitContext *pb)
{
    int n_q = fd->num_envelopes > 1 ? 2 : 1;

    for (int e = 0; e < fd->num_envelopes; e++)
        put_bits(pb, 1, cod->dt_mask >> e & 1);
    for (int q = 0; q < n_q; q++)
        put_bits(pb, 1, q | cod->noise_linked);
}

static void write_sbr_envelope(const AACEncSBRContext *s_ctx, const AACEncSBRInfo *sbr,
                               const AACEncSBRFrameData *fd, const SBREnvCoding *cod,
                               PutBitContext *pb, int ch)
{
    const int hi = fd->eff_amp_res;
    const SBRHuffEntry *ft = hi ? s_ctx->huff_env_3_0dB : s_ctx->huff_env_1_5dB;
    const SBRHuffEntry *tt = hi ? s_ctx->huff_env_t_3_0dB : s_ctx->huff_env_t_1_5dB;
    const int nsyms = hi ? F_HUFF_ENV_3_0DB_NSYMS : F_HUFF_ENV_1_5DB_NSYMS;
    const int offset = hi ? F_HUFF_ENV_3_0DB_OFFSET : F_HUFF_ENV_1_5DB_OFFSET;
    const int first_bits = hi ? 6 : 7;
    const int nb = sbr_env_bands(sbr, fd);

    for (int e = 0; e < fd->num_envelopes; e++) {
        const int *v = fd->ch[ch].env_data[e];
        if (cod->dt_mask >> e & 1) {
            const int *ref = e ? fd->ch[ch].env_data[e - 1] : sbr->ch[ch].prev_env;
            for (int b = 0; b < nb; b++)
                put_huff(pb, tt, nsyms, offset, v[b] - ref[b]);
        } else {
            put_bits(pb, first_bits, v[0]);
            for (int b = 1; b < nb; b++)
                put_huff(pb, ft, nsyms, offset, v[b] - v[b - 1]);
        }
    }
}

static void write_sbr_noise(const AACEncSBRContext *s_ctx, const AACEncSBRFrameData *fd,
                            const SBREnvCoding *cod, PutBitContext *pb)
{
    int n_q = fd->num_envelopes > 1 ? 2 : 1;

    for (int q = 0; q < n_q; q++) {
        if (q | cod->noise_linked) /* constant level: the time delta is 0 */
            put_huff(pb, s_ctx->huff_noise_t, T_HUFF_NOISE_NSYMS, T_HUFF_NOISE_OFFSET, 0);
        else
            put_bits(pb, 5, SBR_NOISE_LEVEL_DEFAULT);
    }
}

static void write_sbr_data(const AACEncSBRContext *s_ctx, const AACEncSBRInfo *sbr, const AACEncSBRFrameData *fd,
                           PutBitContext *pb, int elem_type, int ch0, int send_header)
{
    int nch = (elem_type == TYPE_CPE) ? 2 : 1;
    SBREnvCoding cod[2];

    for (int ch = 0; ch < nch; ch++)
        cod[ch] = choose_env_coding(s_ctx, sbr, fd, ch0 + ch, send_header);

    put_bits(pb, 1, 0); // bs_data_extra = 0
    if (elem_type == TYPE_CPE)
        put_bits(pb, 1, 0); // bs_coupling = 0

    for (int ch = 0; ch < nch; ch++)
        write_sbr_grid(sbr, fd, pb);
    for (int ch = 0; ch < nch; ch++)
        write_sbr_dtdf(fd, &cod[ch], pb);
    for (int ch = 0; ch < nch; ch++)
        write_sbr_invf(sbr, pb);
    for (int ch = 0; ch < nch; ch++)
        write_sbr_envelope(s_ctx, sbr, fd, &cod[ch], pb, ch0 + ch);
    for (int ch = 0; ch < nch; ch++)
        write_sbr_noise(s_ctx, fd, &cod[ch], pb);

    for (int ch = 0; ch < nch; ch++)
        put_bits(pb, 1, 0); // bs_add_harmonic_flag = 0

    put_bits(pb, 1, 0); // bs_extended_data = 0
}

/** Remember what was written as the reference of the next frame. */
static void commit_sbr_state(AACEncSBRInfo *sbr, const AACEncSBRFrameData *fd, int elem_type, int ch0)
{
    int nch = (elem_type == TYPE_CPE) ? 2 : 1;
    int nb  = sbr_env_bands(sbr, fd);

    for (int ch = ch0; ch < ch0 + nch; ch++) {
        AACEncSBRChannel *c = &sbr->ch[ch];
        memcpy(c->prev_env, fd->ch[ch].env_data[fd->num_envelopes - 1], nb * sizeof(int));
        c->prev_nb      = nb;
        c->prev_amp_res = fd->eff_amp_res;
        c->prev_valid   = 1;
    }
}

static void emit_sbr_payload(const AACEncSBRContext *s_ctx, const AACEncSBRInfo *sbr, const AACEncSBRFrameData *fd, PutBitContext *pb, int elem_type, int ch0, int send_header)
{
    put_bits(pb, 4, 13); /* EXT_SBR_DATA (13 = 0xd) */
    put_bits(pb, 1, send_header & 1);
    if (send_header)
        write_sbr_header(sbr, pb);
    write_sbr_data(s_ctx, sbr, fd, pb, elem_type, ch0, send_header);
}

/**
 * Write the SBR extension payload (a FIL element) for one SCE/CPE.
 * With pb == NULL nothing is written and only the size is returned.
 *
 * @return number of bits of the FIL element, 0 if none was or could be written
 */
int ff_aac_sbr_enc_write_payload(AACEncSBRContext *s_ctx, PutBitContext *pb,
                                 int elem_type, int ch0)
{
    AACEncSBRInfo *sbr = s_ctx->sbr_info;
    const AACEncSBRFrameData *fd = &s_ctx->cur_frame_data;
    PutBitContext pb_tmp;
    uint8_t tmp_buf[2048];
    int payload_bits, fill_bytes, pad_bits;

    if (!sbr->header_decided) {
        sbr->send_header_this_frame = (sbr->frame_count++ % SBR_HEADER_PERIOD == 0);
        sbr->header_decided = 1;
    }

    init_put_bits(&pb_tmp, tmp_buf, sizeof(tmp_buf));
    emit_sbr_payload(s_ctx, sbr, fd, &pb_tmp, elem_type, ch0, sbr->send_header_this_frame);
    payload_bits = put_bits_count(&pb_tmp);
    fill_bytes   = (payload_bits + 7) / 8;
    pad_bits     = fill_bytes * 8 - payload_bits;

    /* FIL count: 4 bits, escaping to 14 + 8 bits */
    if (fill_bytes > 14 + 255)
        return 0;

    if (pb) {
        put_bits(pb, 3, TYPE_FIL);
        if (fill_bytes < 15) {
            put_bits(pb, 4, fill_bytes);
        } else {
            put_bits(pb, 4, 15);
            put_bits(pb, 8, fill_bytes - 14);
        }
        emit_sbr_payload(s_ctx, sbr, fd, pb, elem_type, ch0, sbr->send_header_this_frame);
        put_bits(pb, pad_bits, 0);
        commit_sbr_state(sbr, fd, elem_type, ch0);
    }

    return 3 + 4 + (fill_bytes >= 15 ? 8 : 0) + fill_bytes * 8;
}

static AACEncSBRInfo *sbr_init(int channels, int sample_rate, int64_t bit_rate)
{
    AACEncSBRInfo *sbr_info = av_mallocz(sizeof(AACEncSBRInfo));
    if (!sbr_info) return NULL;

    sbr_info->num_channels = channels;
    sbr_info->sample_rate  = sample_rate;
    sbr_info->sbr_present  = 1;

    int rate_per_ch = bit_rate > 0 ? (int)(bit_rate / channels) : 32000;
    sbr_info->num_env_fixfix = (rate_per_ch >= SBR_FOUR_ENV_BITRATE_BPS) ? 4 : 1;
    sbr_info->bs_start_freq  = pick_start_freq(sample_rate, rate_per_ch >= SBR_CROSSOVER_HIGH_BPS ?
                                                        SBR_CROSSOVER_HIGH_HZ : SBR_CROSSOVER_LOW_HZ);
    sbr_info->bs_freq_scale  = (rate_per_ch >= SBR_FREQ_SCALE_COARSE_BPS) ? 3 : 2;
    sbr_info->bs_alter_scale = 0;
    sbr_info->bs_xover_band  = 0;
    sbr_info->invf_mode      = rate_per_ch >= SBR_INVF_HIGH_RATE_BPS ? SBR_INVF_MODE_HIGH_RATE
                                                                      : SBR_INVF_MODE_LOW_RATE;
    sbr_info->bs_freq_res    = 1;

    sbr_info->kx = compute_kx(sample_rate, sbr_info->bs_start_freq);
    int target_hz = (rate_per_ch < SBR_NARROW_STOP_BPS && sample_rate > SBR_NARROW_STOP_MIN_RATE) ?
                    SBR_STOP_FREQ_TARGET_NARROW_HZ : SBR_STOP_FREQ_TARGET_HZ;
    sbr_info->bs_stop_freq = pick_stop_freq(sample_rate, sbr_info->kx, target_hz, &sbr_info->k2);

    build_freq_table(sbr_info);

    for (int m = 0; m < SBR_QMF_BANDS_64; m++) {
        sbr_info->twid_cos[m] = (float)cos(M_PI * m / 64.0);
        sbr_info->twid_sin[m] = (float)sin(M_PI * m / 64.0);
        sbr_info->odd_cos[m]  = (float)cos(M_PI * (2 * m + 1) / 128.0);
        sbr_info->odd_sin[m]  = (float)sin(M_PI * (2 * m + 1) / 128.0);
    }

    return sbr_info;
}

static double bessel_i0(double x)
{
    double sum = 1.0, term = 1.0;
    for (int k = 1; k < 32; k++) {
        term *= (x / (2.0 * k)) * (x / (2.0 * k));
        sum  += term;
    }
    return sum;
}

/* Kaiser-windowed sinc low-pass at a quarter of the input rate, unity DC gain */
static void build_decim_filter(float *fir)
{
    double h[SBR_DECIM_TAPS], sum = 0.0;

    for (int i = 0; i < SBR_DECIM_TAPS; i++) {
        double n = i - SBR_DECIM_HALF;
        double r = n / SBR_DECIM_HALF;
        double w = bessel_i0(SBR_DECIM_BETA * sqrt(1.0 - r * r)) / bessel_i0(SBR_DECIM_BETA);
        h[i] = (n == 0.0 ? 0.5 : sin(M_PI * n / 2.0) / (M_PI * n)) * w;
        sum += h[i];
    }
    for (int i = 0; i < SBR_DECIM_TAPS; i++)
        fir[i] = h[i] / sum;
}

AACEncSBRContext *ff_aac_sbr_enc_init(int sample_rate, int64_t bit_rate,
                                      int channels, int full_rate_idx)
{
    AACEncSBRContext *s_ctx = av_mallocz(sizeof(*s_ctx));
    float scale = 1.0f;

    if (!s_ctx)
        return NULL;

    s_ctx->full_sample_rate     = sample_rate;
    s_ctx->full_sample_rate_idx = full_rate_idx;
    s_ctx->sbr_info             = sbr_init(channels, sample_rate, bit_rate);
    if (!s_ctx->sbr_info ||
        av_tx_init(&s_ctx->fft_ctx, &s_ctx->fft_fn, AV_TX_FLOAT_FFT, 0, 64, &scale, 0) < 0) {
        ff_aac_sbr_enc_close(s_ctx);
        return NULL;
    }

    build_decim_filter(s_ctx->fir);

    /* Build SBR Huffman encoder lookup tables from the shared decoder tables */
    const uint8_t (*tab)[2] = ff_aac_sbr_huffman_tab;
    int off1 = ff_aac_sbr_huffman_nb_codes[0];
    int off5 = off1 + ff_aac_sbr_huffman_nb_codes[1] + ff_aac_sbr_huffman_nb_codes[2] +
               ff_aac_sbr_huffman_nb_codes[3] + ff_aac_sbr_huffman_nb_codes[4];

    build_sbr_huff_table(s_ctx->huff_env_1_5dB, tab + off1, ff_aac_sbr_huffman_nb_codes[1]);
    build_sbr_huff_table(s_ctx->huff_env_3_0dB, tab + off5, ff_aac_sbr_huffman_nb_codes[5]);
    build_sbr_huff_table(s_ctx->huff_env_t_1_5dB, tab, ff_aac_sbr_huffman_nb_codes[0]);
    build_sbr_huff_table(s_ctx->huff_env_t_3_0dB, tab + off5 - ff_aac_sbr_huffman_nb_codes[4],
                         ff_aac_sbr_huffman_nb_codes[4]);
    {
        int off8 = off5 + ff_aac_sbr_huffman_nb_codes[5] + ff_aac_sbr_huffman_nb_codes[6] +
                   ff_aac_sbr_huffman_nb_codes[7];
        build_sbr_huff_table(s_ctx->huff_noise_t, tab + off8, ff_aac_sbr_huffman_nb_codes[8]);
    }

    sbr_frame_silence(&s_ctx->cur_frame_data);

    return s_ctx;
}

void ff_aac_sbr_enc_close(AACEncSBRContext *s_ctx)
{
    if (!s_ctx)
        return;
    av_tx_uninit(&s_ctx->fft_ctx);
    av_freep(&s_ctx->sbr_info);
    av_free(s_ctx);
}

/**
 * Decimate 2*frame_len full-rate samples to frame_len core samples.
 * input_samples[ch] must be preceded by at least 2*SBR_DECIM_HALF valid
 * history samples.
 */
void ff_aac_sbr_enc_resample(AACEncSBRContext *s_ctx, int num_channels,
                             int frame_len, float **input_samples, float **core_samples)
{
    for (int ch = 0; ch < num_channels; ch++) {
        const float *in = input_samples[ch];
        for (int n = 0; n < frame_len; n++) {
            const float *x = in + 2 * n;
            float acc = 0.0f;
            for (int k = 0; k < SBR_DECIM_TAPS; k++)
                acc += s_ctx->fir[k] * x[-k];
            core_samples[ch][n] = acc;
        }
    }
}

void ff_aac_sbr_enc_process_frame(AACEncSBRContext *s_ctx, int num_channels,
                                  int frame_len, float **input_samples)
{
    AACEncSBRFrameData *fd = &s_ctx->cur_frame_data;
    s_ctx->sbr_info->header_decided = 0;

    sbr_analyze(s_ctx, &s_ctx->signal_analysis, input_samples, num_channels, 2 * frame_len, s_ctx->sbr_info);
    sbr_encode(s_ctx->sbr_info, input_samples, num_channels, 2 * frame_len, &s_ctx->signal_analysis, fd);
}

int ff_aac_sbr_enc_get_full_rate_idx(const AACEncSBRContext *s_ctx)
{
    return s_ctx->full_sample_rate_idx;
}

int ff_aac_sbr_enc_get_crossover(const AACEncSBRContext *s_ctx)
{
    return s_ctx->sbr_info->kx * s_ctx->full_sample_rate / (2 * SBR_QMF_BANDS_64);
}
