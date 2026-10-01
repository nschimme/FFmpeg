/*
 * FAAC AAC encoder wrapper
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

#include <faac.h>

#include "libavutil/channel_layout.h"
#include "libavutil/common.h"
#include "libavutil/log.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "avcodec.h"
#include "audio_frame_queue.h"
#include "codec_internal.h"
#include "encode.h"
#include "profiles.h"

typedef struct FaacEncContext {
    const AVClass *class;
    faac_encoder *handle;
    AudioFrameQueue afq;
    uint32_t max_output_bytes;

    int mpegversion;      /* FAAC_MPEG4 / FAAC_MPEG2 */
    int object_type;      /* FAAC_OBJ_AUTO, FAAC_OBJ_LOW, FAAC_OBJ_HE_AAC_V1 */
    int rate_control;     /* FAAC_RC_AUTO, FAAC_RC_VBR, FAAC_RC_ABR, FAAC_RC_CBR */
    int joint_mode;       /* FAAC_JOINT_NONE, FAAC_JOINT_MS, FAAC_JOINT_IS, FAAC_JOINT_MIXED */
    int short_control;    /* FAAC_SHORTCTL_NORMAL, FAAC_SHORTCTL_NOSHORT, FAAC_SHORTCTL_NOLONG */
    int tns;              /* -1 = library default (true), 1, 0 */
    int pns;              /* -1 = library default (true), 1, 0 */
    int max_bit_rate;     /* Whole-stream peak bitrate ceiling (0 = unlimited) */
} FaacEncContext;

static av_cold int faac_encode_init(AVCodecContext *avctx)
{
    FaacEncContext *s = avctx->priv_data;
    faac_params params;
    faac_encoder_info info;
    faac_status status;
    int channels = avctx->ch_layout.nb_channels;
    int qual = 0;

    status = faac_params_init(&params, sizeof(params));
    if (status != FAAC_OK) {
        av_log(avctx, AV_LOG_ERROR, "faac_params_init failed: %s (%d)\n",
               faac_strerror(status), status);
        return AVERROR_EXTERNAL;
    }

    params.sample_rate  = avctx->sample_rate;
    params.num_channels = channels;

    switch (avctx->sample_fmt) {
    case AV_SAMPLE_FMT_S16:
        params.input_format = FAAC_INPUT_16BIT;
        break;
    case AV_SAMPLE_FMT_S32:
        params.input_format = FAAC_INPUT_32BIT;
        break;
    case AV_SAMPLE_FMT_FLT:
        params.input_format = FAAC_INPUT_FLOAT;
        break;
    default:
        av_log(avctx, AV_LOG_ERROR, "Unsupported sample format\n");
        return AVERROR(EINVAL);
    }

    params.mpeg_version  = s->mpegversion;
    params.joint_mode    = s->joint_mode;
    params.short_control = s->short_control;
    if (s->tns >= 0)
        params.use_tns   = !!s->tns;
    if (s->pns >= 0)
        params.use_pns   = !!s->pns;

    params.use_lfe = (av_channel_layout_index_from_channel(&avctx->ch_layout,
                                                           AV_CHAN_LOW_FREQUENCY) >= 0);

    /* Profile / Object Type selection */
    if (s->object_type != FAAC_OBJ_AUTO) {
        params.object_type = s->object_type;
    } else {
        switch (avctx->profile) {
        case AV_PROFILE_AAC_LOW:
            params.object_type = FAAC_OBJ_LOW;
            break;
        case AV_PROFILE_MPEG2_AAC_LOW:
            params.object_type  = FAAC_OBJ_LOW;
            params.mpeg_version = FAAC_MPEG2;
            break;
        case AV_PROFILE_AAC_HE:
        case AV_PROFILE_MPEG2_AAC_HE:
            params.object_type  = FAAC_OBJ_HE_AAC_V1;
            params.mpeg_version = FAAC_MPEG4;
            if (avctx->profile == AV_PROFILE_MPEG2_AAC_HE || s->mpegversion == FAAC_MPEG2)
                av_log(avctx, AV_LOG_WARNING, "HE-AAC v1 requires MPEG-4; forcing MPEG-4\n");
            break;
        default:
            params.object_type = FAAC_OBJ_AUTO;
            break;
        }
    }

    if (params.object_type == FAAC_OBJ_HE_AAC_V1 && params.mpeg_version == FAAC_MPEG2) {
        av_log(avctx, AV_LOG_WARNING, "HE-AAC v1 requires MPEG-4; forcing MPEG-4\n");
        params.mpeg_version = FAAC_MPEG4;
    }

    if (params.mpeg_version == FAAC_MPEG2 && params.use_pns) {
        av_log(avctx, AV_LOG_WARNING, "PNS requires MPEG-4; disabling PNS for MPEG-2\n");
        params.use_pns = 0;
    }

    /* Bitrate translation: FFmpeg provides total stream bitrate, FAAC expects per-channel */
    if (avctx->bit_rate > 0)
        params.bit_rate = (uint32_t)(avctx->bit_rate / channels);

    if (avctx->global_quality > 0) {
        qual = (avctx->global_quality > 5000) ? (avctx->global_quality / FF_QP2LAMBDA)
                                              : avctx->global_quality;
        qual = av_clip(qual, 1, 5000);
    }

    /* Rate Control mode configuration */
    if (s->rate_control != FAAC_RC_AUTO) {
        params.rate_control = s->rate_control;
    } else if (qual > 0 && avctx->bit_rate <= 0) {
        params.rate_control = FAAC_RC_VBR;
    } else if (avctx->rc_max_rate > 0 && avctx->rc_max_rate == avctx->bit_rate) {
        params.rate_control = FAAC_RC_CBR;
    } else if (avctx->bit_rate > 0) {
        params.rate_control = FAAC_RC_ABR;
    } else {
        params.rate_control = FAAC_RC_AUTO;
    }

    if (params.rate_control == FAAC_RC_VBR) {
        params.bit_rate = 0;
        params.quant_quality = qual > 0 ? qual : 100;
    } else if (qual > 0) {
        params.quant_quality = qual;
    }

    /* Peak bitrate ceiling */
    if (s->max_bit_rate > 0)
        params.max_bit_rate = s->max_bit_rate;
    else if (avctx->rc_max_rate > 0)
        params.max_bit_rate = avctx->rc_max_rate;

    /* Cutoff / Bandwidth */
    if (avctx->cutoff > 0)
        params.bandwidth = avctx->cutoff;

    /* Stream Format & Extradata */
    if (avctx->flags & AV_CODEC_FLAG_GLOBAL_HEADER)
        params.output_format = FAAC_STREAM_RAW;
    else
        params.output_format = FAAC_STREAM_ADTS;

    status = faac_encoder_open(&params, &s->handle);
    if (status != FAAC_OK) {
        av_log(avctx, AV_LOG_ERROR, "faac_encoder_open failed: %s (%d)\n",
               faac_strerror(status), status);
        return AVERROR_EXTERNAL;
    }

    if (params.output_format == FAAC_STREAM_RAW) {
        const uint8_t *asc_buf = NULL;
        uint32_t asc_len = 0;

        status = faac_encoder_asc(s->handle, &asc_buf, &asc_len);
        if (status == FAAC_OK && asc_len > 0) {
            avctx->extradata = av_malloc(asc_len + AV_INPUT_BUFFER_PADDING_SIZE);
            if (!avctx->extradata)
                return AVERROR(ENOMEM);
            memcpy(avctx->extradata, asc_buf, asc_len);
            memset(avctx->extradata + asc_len, 0, AV_INPUT_BUFFER_PADDING_SIZE);
            avctx->extradata_size = asc_len;
        } else if (status != FAAC_OK) {
            av_log(avctx, AV_LOG_ERROR, "faac_encoder_asc failed: %s\n", faac_strerror(status));
            return AVERROR_EXTERNAL;
        }
    }

    info.struct_size = sizeof(info);
    status = faac_encoder_get_info(s->handle, &info);
    if (status != FAAC_OK) {
        av_log(avctx, AV_LOG_ERROR, "faac_encoder_get_info failed: %s\n", faac_strerror(status));
        return AVERROR_EXTERNAL;
    }

    avctx->frame_size      = info.frame_samples;
    avctx->initial_padding = info.encoder_delay;
    s->max_output_bytes    = info.max_output_bytes;

    ff_af_queue_init(avctx, &s->afq);

    return 0;
}

static int faac_encode_frame(AVCodecContext *avctx, AVPacket *avpkt,
                             const AVFrame *frame, int *got_packet_ptr)
{
    FaacEncContext *s = avctx->priv_data;
    const void *in_buf = NULL;
    uint32_t in_samples = 0;
    uint32_t bytes_written = 0;
    faac_status status;
    int ret;

    if (frame) {
        in_buf     = frame->data[0];
        in_samples = frame->nb_samples * avctx->ch_layout.nb_channels;
        if ((ret = ff_af_queue_add(&s->afq, frame)) < 0)
            return ret;
    }

    if ((ret = ff_alloc_packet(avctx, avpkt, s->max_output_bytes)) < 0)
        return ret;

    status = faac_encoder_encode(s->handle, in_buf, in_samples,
                                 avpkt->data, s->max_output_bytes,
                                 &bytes_written);
    if (status != FAAC_OK) {
        av_log(avctx, AV_LOG_ERROR, "faac_encoder_encode failed: %s (%d)\n",
               faac_strerror(status), status);
        av_packet_unref(avpkt);
        return AVERROR_EXTERNAL;
    }

    if (bytes_written > 0) {
        avpkt->size = bytes_written;
        ff_af_queue_remove(&s->afq, avctx->frame_size, avpkt);
        *got_packet_ptr = 1;
    } else {
        av_packet_unref(avpkt);
        *got_packet_ptr = 0;
    }

    return 0;
}

static av_cold int faac_encode_close(AVCodecContext *avctx)
{
    FaacEncContext *s = avctx->priv_data;

    if (s->handle)
        faac_encoder_close(&s->handle);

    ff_af_queue_close(&s->afq);

    return 0;
}

#define OFFSET(x) offsetof(FaacEncContext, x)
#define AE AV_OPT_FLAG_AUDIO_PARAM | AV_OPT_FLAG_ENCODING_PARAM

static const AVOption faac_enc_options[] = {
    { "mpegversion", "MPEG version", OFFSET(mpegversion), AV_OPT_TYPE_INT, { .i64 = FAAC_MPEG4 }, FAAC_MPEG4, FAAC_MPEG2, AE, .unit = "mpegversion" },
        { "mpeg4", "MPEG-4", 0, AV_OPT_TYPE_CONST, { .i64 = FAAC_MPEG4 }, 0, 0, AE, .unit = "mpegversion" },
        { "mpeg2", "MPEG-2", 0, AV_OPT_TYPE_CONST, { .i64 = FAAC_MPEG2 }, 0, 0, AE, .unit = "mpegversion" },

    { "object_type", "AAC object type", OFFSET(object_type), AV_OPT_TYPE_INT, { .i64 = FAAC_OBJ_AUTO }, FAAC_OBJ_AUTO, FAAC_OBJ_HE_AAC_V1, AE, .unit = "object_type" },
        { "auto",      "Auto profile selection", 0, AV_OPT_TYPE_CONST, { .i64 = FAAC_OBJ_AUTO },      0, 0, AE, .unit = "object_type" },
        { "lc",        "AAC Low Complexity (AAC-LC)", 0, AV_OPT_TYPE_CONST, { .i64 = FAAC_OBJ_LOW },  0, 0, AE, .unit = "object_type" },
        { "he_aac_v1", "HE-AAC v1 (AAC-LC + SBR)",   0, AV_OPT_TYPE_CONST, { .i64 = FAAC_OBJ_HE_AAC_V1 }, 0, 0, AE, .unit = "object_type" },

    { "rate_control", "Rate control mode", OFFSET(rate_control), AV_OPT_TYPE_INT, { .i64 = FAAC_RC_AUTO }, FAAC_RC_AUTO, FAAC_RC_CBR, AE, .unit = "rate_control" },
        { "auto", "Auto rate control", 0, AV_OPT_TYPE_CONST, { .i64 = FAAC_RC_AUTO }, 0, 0, AE, .unit = "rate_control" },
        { "vbr",  "Variable bit rate", 0, AV_OPT_TYPE_CONST, { .i64 = FAAC_RC_VBR },  0, 0, AE, .unit = "rate_control" },
        { "abr",  "Average bit rate",  0, AV_OPT_TYPE_CONST, { .i64 = FAAC_RC_ABR },  0, 0, AE, .unit = "rate_control" },
        { "cbr",  "Constant bit rate", 0, AV_OPT_TYPE_CONST, { .i64 = FAAC_RC_CBR },  0, 0, AE, .unit = "rate_control" },

    { "joint_mode", "Joint stereo mode", OFFSET(joint_mode), AV_OPT_TYPE_INT, { .i64 = FAAC_JOINT_MIXED }, FAAC_JOINT_NONE, FAAC_JOINT_MIXED, AE, .unit = "joint_mode" },
        { "none",  "Independent L/R stereo",           0, AV_OPT_TYPE_CONST, { .i64 = FAAC_JOINT_NONE },  0, 0, AE, .unit = "joint_mode" },
        { "ms",    "Mid/Side stereo",                  0, AV_OPT_TYPE_CONST, { .i64 = FAAC_JOINT_MS },    0, 0, AE, .unit = "joint_mode" },
        { "is",    "Intensity stereo",                 0, AV_OPT_TYPE_CONST, { .i64 = FAAC_JOINT_IS },    0, 0, AE, .unit = "joint_mode" },
        { "mixed", "Per-band mix of M/S and Intensity", 0, AV_OPT_TYPE_CONST, { .i64 = FAAC_JOINT_MIXED }, 0, 0, AE, .unit = "joint_mode" },

    { "short_control", "Short block control mode", OFFSET(short_control), AV_OPT_TYPE_INT, { .i64 = FAAC_SHORTCTL_NORMAL }, FAAC_SHORTCTL_NORMAL, FAAC_SHORTCTL_NOLONG, AE, .unit = "short_control" },
        { "normal",  "Let block switching decide", 0, AV_OPT_TYPE_CONST, { .i64 = FAAC_SHORTCTL_NORMAL },  0, 0, AE, .unit = "short_control" },
        { "noshort", "Force long blocks only",     0, AV_OPT_TYPE_CONST, { .i64 = FAAC_SHORTCTL_NOSHORT }, 0, 0, AE, .unit = "short_control" },
        { "nolong",  "Force short blocks only",    0, AV_OPT_TYPE_CONST, { .i64 = FAAC_SHORTCTL_NOLONG },  0, 0, AE, .unit = "short_control" },

    { "aac_tns", "Temporal noise shaping", OFFSET(tns), AV_OPT_TYPE_BOOL, { .i64 = -1 }, -1, 1, AE },
    { "tns",     "Temporal noise shaping", OFFSET(tns), AV_OPT_TYPE_BOOL, { .i64 = -1 }, -1, 1, AE },

    { "aac_pns", "Perceptual noise substitution", OFFSET(pns), AV_OPT_TYPE_BOOL, { .i64 = -1 }, -1, 1, AE },
    { "pns",     "Perceptual noise substitution", OFFSET(pns), AV_OPT_TYPE_BOOL, { .i64 = -1 }, -1, 1, AE },

    { "max_bit_rate", "Whole-stream peak bitrate ceiling in bits/s (0 = unlimited)", OFFSET(max_bit_rate), AV_OPT_TYPE_INT, { .i64 = 0 }, 0, INT_MAX, AE },

    FF_AAC_PROFILE_OPTS
    { NULL }
};

static const AVClass faac_enc_class = {
    .class_name = "libfaac",
    .item_name  = av_default_item_name,
    .option     = faac_enc_options,
    .version    = LIBAVUTIL_VERSION_INT,
};

static const enum AVSampleFormat faac_sample_fmts[] = {
    AV_SAMPLE_FMT_S16,
    AV_SAMPLE_FMT_S32,
    AV_SAMPLE_FMT_FLT,
    AV_SAMPLE_FMT_NONE,
};

static const FFCodecDefault faac_encode_defaults[] = {
    { "b", "0" },
    { NULL }
};

const FFCodec ff_libfaac_encoder = {
    .p.name         = "libfaac",
    CODEC_LONG_NAME("libfaac AAC (Advanced Audio Coding)"),
    .p.type         = AVMEDIA_TYPE_AUDIO,
    .p.id           = AV_CODEC_ID_AAC,
    .p.capabilities = AV_CODEC_CAP_DR1 | AV_CODEC_CAP_DELAY |
                      AV_CODEC_CAP_SMALL_LAST_FRAME,
    CODEC_SAMPLEFMTS_ARRAY(faac_sample_fmts),
    .p.profiles     = NULL_IF_CONFIG_SMALL(ff_aac_profiles),
    .p.priv_class   = &faac_enc_class,
    .p.wrapper_name = "libfaac",
    .priv_data_size = sizeof(FaacEncContext),
    .init           = faac_encode_init,
    FF_CODEC_ENCODE_CB(faac_encode_frame),
    .close          = faac_encode_close,
    .defaults       = faac_encode_defaults,
};
