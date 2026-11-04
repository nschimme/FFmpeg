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

    int mpegversion;      /* FAAC_STREAM_ADTS / FAAC_STREAM_ADTS_MPEG2 */
    int object_type;      /* FAAC_OBJ_AUTO, FAAC_OBJ_LC, FAAC_OBJ_HE_AAC_V1, FAAC_OBJ_HE_AAC_V2 */
    int rate_control;     /* -1 = auto, FAAC_RC_ABR, FAAC_RC_VBR, FAAC_RC_CBR */
    int stereo_mode;      /* FAAC_STEREO_AUTO, FAAC_STEREO_LR, FAAC_STEREO_MS, FAAC_STEREO_IS */
    int short_control;    /* FAAC_SHORTCTL_NORMAL, FAAC_SHORTCTL_NOSHORT, FAAC_SHORTCTL_NOLONG */
    int tns;              /* -1 = library default (true), 1, 0 */
    int pns;              /* -1 = library default (true), 1, 0 */
    int peak_bit_rate;    /* Whole-stream peak bitrate ceiling (0 = unlimited) */
    int drc_profile;      /* FAAC_DRC_OFF .. FAAC_DRC_SPEECH */
    float drc_ref_level;  /* -31.75 .. 0.0 LUFS/dBFS */
} FaacEncContext;

static av_cold int faac_encode_init(AVCodecContext *avctx)
{
    FaacEncContext *s = avctx->priv_data;
    faac_params params;
    faac_encoder_info info;
    faac_library_info lib_info;
    faac_status status;
    int channels = avctx->ch_layout.nb_channels;
    int qual = 0;

    lib_info.struct_size = sizeof(lib_info);
    status = faac_get_library_info(&lib_info);
    if (status != FAAC_OK) {
        av_log(avctx, AV_LOG_ERROR, "faac_get_library_info failed: %s (%d)\n",
               faac_strerror(status), status);
        return AVERROR_EXTERNAL;
    }

    if (channels > (int)lib_info.max_channels) {
        av_log(avctx, AV_LOG_ERROR, "Channel count %d exceeds libfaac maximum of %u\n",
               channels, (unsigned)lib_info.max_channels);
        return AVERROR(EINVAL);
    }

    status = faac_params_init(&params, sizeof(params));
    if (status != FAAC_OK) {
        av_log(avctx, AV_LOG_ERROR, "faac_params_init failed: %s (%d)\n",
               faac_strerror(status), status);
        return AVERROR_EXTERNAL;
    }

    params.sample_rate = avctx->sample_rate;
    params.channels    = channels;

    switch (avctx->sample_fmt) {
    case AV_SAMPLE_FMT_S16:
        params.input_format = FAAC_INPUT_16BIT;
        break;
    case AV_SAMPLE_FMT_S32:
        params.input_format = FAAC_INPUT_24IN32;
        break;
    case AV_SAMPLE_FMT_FLT:
        params.input_format = FAAC_INPUT_FLOAT;
        break;
    default:
        av_log(avctx, AV_LOG_ERROR, "Unsupported sample format\n");
        return AVERROR(EINVAL);
    }

    params.stereo_mode   = s->stereo_mode;
    params.short_control = s->short_control;
    params.channel_layout = FAAC_LAYOUT_WAV;

    if (s->tns == 0)
        params.flags &= ~FAAC_FLAG_TNS;
    else if (s->tns > 0)
        params.flags |= FAAC_FLAG_TNS;

    if (s->pns == 0)
        params.flags &= ~FAAC_FLAG_PNS;
    else if (s->pns > 0)
        params.flags |= FAAC_FLAG_PNS;

    /* Profile / Object Type selection */
    if (s->object_type != FAAC_OBJ_AUTO) {
        params.object_type = s->object_type;
    } else {
        switch (avctx->profile) {
        case AV_PROFILE_AAC_LOW:
            params.object_type = FAAC_OBJ_LC;
            break;
        case AV_PROFILE_MPEG2_AAC_LOW:
            params.object_type = FAAC_OBJ_LC;
            s->mpegversion     = 2;
            break;
        case AV_PROFILE_AAC_HE:
        case AV_PROFILE_MPEG2_AAC_HE:
            params.object_type = FAAC_OBJ_HE_AAC_V1;
            if (avctx->profile == AV_PROFILE_MPEG2_AAC_HE || s->mpegversion == 2)
                av_log(avctx, AV_LOG_WARNING, "HE-AAC v1 requires MPEG-4; forcing MPEG-4\n");
            s->mpegversion = 4;
            break;
        case AV_PROFILE_AAC_HE_V2:
            params.object_type = FAAC_OBJ_HE_AAC_V2;
            s->mpegversion = 4;
            break;
        default:
            params.object_type = FAAC_OBJ_AUTO;
            break;
        }
    }

    /* Check runtime library capabilities */
    if (params.object_type == FAAC_OBJ_HE_AAC_V1 || params.object_type == FAAC_OBJ_HE_AAC_V2) {
        if (!(lib_info.flags & FAAC_LIB_SBR)) {
            av_log(avctx, AV_LOG_ERROR, "HE-AAC requested but libfaac was compiled without SBR support\n");
            return AVERROR(EINVAL);
        }
    }
    if (params.object_type == FAAC_OBJ_HE_AAC_V2) {
        if (!(lib_info.flags & FAAC_LIB_PS)) {
            av_log(avctx, AV_LOG_ERROR, "HE-AAC v2 requested but libfaac was compiled without PS support\n");
            return AVERROR(EINVAL);
        }
        if (channels != 2) {
            av_log(avctx, AV_LOG_ERROR, "HE-AAC v2 requires stereo input (2 channels)\n");
            return AVERROR(EINVAL);
        }
    }

    if ((params.object_type == FAAC_OBJ_HE_AAC_V1 || params.object_type == FAAC_OBJ_HE_AAC_V2) &&
        s->mpegversion == 2) {
        av_log(avctx, AV_LOG_WARNING, "HE-AAC requires MPEG-4; forcing MPEG-4\n");
        s->mpegversion = 4;
    }

    /* Stream Format & Extradata */
    if (avctx->flags & AV_CODEC_FLAG_GLOBAL_HEADER)
        params.stream_format = FAAC_STREAM_RAW;
    else if (s->mpegversion == 2)
        params.stream_format = FAAC_STREAM_ADTS_MPEG2;
    else
        params.stream_format = FAAC_STREAM_ADTS;

    if (params.stream_format == FAAC_STREAM_ADTS_MPEG2 && (params.flags & FAAC_FLAG_PNS)) {
        av_log(avctx, AV_LOG_WARNING, "PNS requires MPEG-4; disabling PNS for MPEG-2\n");
        params.flags &= ~FAAC_FLAG_PNS;
    }

    /* Bitrate translation: whole stream in ABI 3.x */
    if (avctx->bit_rate > 0)
        params.bit_rate = (uint32_t)avctx->bit_rate;

    if (avctx->global_quality > 0) {
        qual = (avctx->global_quality > 5000) ? (avctx->global_quality / FF_QP2LAMBDA)
                                              : avctx->global_quality;
        qual = av_clip(qual, 1, 5000);
    }

    /* Rate Control mode configuration */
    if (s->rate_control >= 0) {
        params.rate_control = s->rate_control;
    } else if (qual > 0 && avctx->bit_rate <= 0) {
        params.rate_control = FAAC_RC_VBR;
    } else if (avctx->rc_max_rate > 0 && avctx->rc_max_rate == avctx->bit_rate) {
        params.rate_control = FAAC_RC_CBR;
    } else {
        params.rate_control = FAAC_RC_ABR;
    }

    if (params.rate_control == FAAC_RC_VBR) {
        params.bit_rate = 0;
        params.quality = qual > 0 ? qual : 100;
    } else {
        params.quality = 0;
    }

    /* Peak bitrate ceiling */
    if (s->peak_bit_rate > 0)
        params.peak_bit_rate = s->peak_bit_rate;
    else if (avctx->rc_max_rate > 0)
        params.peak_bit_rate = avctx->rc_max_rate;

    if (params.peak_bit_rate > 0 && params.peak_bit_rate < params.bit_rate) {
        av_log(avctx, AV_LOG_WARNING, "Peak bitrate (%u) < bit_rate (%u), raising peak bitrate\n",
               params.peak_bit_rate, params.bit_rate);
        params.peak_bit_rate = params.bit_rate;
    }

    /* DRC configuration */
    if (s->drc_profile != FAAC_DRC_OFF) {
        if (!(lib_info.flags & FAAC_LIB_DRC)) {
            av_log(avctx, AV_LOG_ERROR, "DRC profile requested but libfaac was compiled without DRC support\n");
            return AVERROR(EINVAL);
        }
        params.drc_profile   = s->drc_profile;
        params.drc_ref_level = av_clipf(s->drc_ref_level, -31.75f, 0.0f);
    }

    /* Cutoff / Bandwidth */
    if (avctx->cutoff > 0)
        params.bandwidth = avctx->cutoff;

    status = faac_encoder_open(&params, &s->handle);
    if (status != FAAC_OK) {
        av_log(avctx, AV_LOG_ERROR, "faac_encoder_open failed: %s (%d)\n",
               faac_strerror(status), status);
        return AVERROR_EXTERNAL;
    }

    if (params.stream_format == FAAC_STREAM_RAW) {
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
    { "mpegversion", "MPEG version", OFFSET(mpegversion), AV_OPT_TYPE_INT, { .i64 = 4 }, 2, 4, AE, .unit = "mpegversion" },
        { "mpeg4", "MPEG-4", 0, AV_OPT_TYPE_CONST, { .i64 = 4 }, 0, 0, AE, .unit = "mpegversion" },
        { "mpeg2", "MPEG-2", 0, AV_OPT_TYPE_CONST, { .i64 = 2 }, 0, 0, AE, .unit = "mpegversion" },

    { "object_type", "AAC object type", OFFSET(object_type), AV_OPT_TYPE_INT, { .i64 = FAAC_OBJ_AUTO }, FAAC_OBJ_AUTO, FAAC_OBJ_HE_AAC_V2, AE, .unit = "object_type" },
        { "auto",      "Auto profile selection",     0, AV_OPT_TYPE_CONST, { .i64 = FAAC_OBJ_AUTO },      0, 0, AE, .unit = "object_type" },
        { "lc",        "AAC Low Complexity (AAC-LC)", 0, AV_OPT_TYPE_CONST, { .i64 = FAAC_OBJ_LC },        0, 0, AE, .unit = "object_type" },
        { "he_aac_v1", "HE-AAC v1 (AAC-LC + SBR)",   0, AV_OPT_TYPE_CONST, { .i64 = FAAC_OBJ_HE_AAC_V1 }, 0, 0, AE, .unit = "object_type" },
        { "he_aac_v2", "HE-AAC v2 (AAC-LC + SBR + PS)", 0, AV_OPT_TYPE_CONST, { .i64 = FAAC_OBJ_HE_AAC_V2 }, 0, 0, AE, .unit = "object_type" },

    { "rate_control", "Rate control mode", OFFSET(rate_control), AV_OPT_TYPE_INT, { .i64 = -1 }, -1, FAAC_RC_CBR, AE, .unit = "rate_control" },
        { "auto", "Auto rate control", 0, AV_OPT_TYPE_CONST, { .i64 = -1 },           0, 0, AE, .unit = "rate_control" },
        { "abr",  "Average bit rate",  0, AV_OPT_TYPE_CONST, { .i64 = FAAC_RC_ABR },  0, 0, AE, .unit = "rate_control" },
        { "vbr",  "Variable bit rate", 0, AV_OPT_TYPE_CONST, { .i64 = FAAC_RC_VBR },  0, 0, AE, .unit = "rate_control" },
        { "cbr",  "Constant bit rate", 0, AV_OPT_TYPE_CONST, { .i64 = FAAC_RC_CBR },  0, 0, AE, .unit = "rate_control" },

    { "stereo_mode", "Stereo mode", OFFSET(stereo_mode), AV_OPT_TYPE_INT, { .i64 = FAAC_STEREO_AUTO }, FAAC_STEREO_LR, FAAC_STEREO_AUTO, AE, .unit = "stereo_mode" },
        { "lr",   "Independent (L/R) stereo",           0, AV_OPT_TYPE_CONST, { .i64 = FAAC_STEREO_LR },   0, 0, AE, .unit = "stereo_mode" },
        { "ms",   "Mid/Side stereo",                    0, AV_OPT_TYPE_CONST, { .i64 = FAAC_STEREO_MS },   0, 0, AE, .unit = "stereo_mode" },
        { "is",   "Intensity stereo",                   0, AV_OPT_TYPE_CONST, { .i64 = FAAC_STEREO_IS },   0, 0, AE, .unit = "stereo_mode" },
        { "auto", "Per-band mix of M/S and Intensity",  0, AV_OPT_TYPE_CONST, { .i64 = FAAC_STEREO_AUTO }, 0, 0, AE, .unit = "stereo_mode" },
    { "joint_mode", "Joint stereo mode (alias for stereo_mode)", OFFSET(stereo_mode), AV_OPT_TYPE_INT, { .i64 = FAAC_STEREO_AUTO }, FAAC_STEREO_LR, FAAC_STEREO_AUTO, AE, .unit = "stereo_mode" },

    { "short_control", "Short block control mode", OFFSET(short_control), AV_OPT_TYPE_INT, { .i64 = FAAC_SHORTCTL_NORMAL }, FAAC_SHORTCTL_NORMAL, FAAC_SHORTCTL_NOLONG, AE, .unit = "short_control" },
        { "normal",  "Let block switching decide", 0, AV_OPT_TYPE_CONST, { .i64 = FAAC_SHORTCTL_NORMAL },  0, 0, AE, .unit = "short_control" },
        { "noshort", "Force long blocks only",     0, AV_OPT_TYPE_CONST, { .i64 = FAAC_SHORTCTL_NOSHORT }, 0, 0, AE, .unit = "short_control" },
        { "nolong",  "Force short blocks only",    0, AV_OPT_TYPE_CONST, { .i64 = FAAC_SHORTCTL_NOLONG },  0, 0, AE, .unit = "short_control" },

    { "aac_tns", "Temporal noise shaping", OFFSET(tns), AV_OPT_TYPE_BOOL, { .i64 = -1 }, -1, 1, AE },
    { "tns",     "Temporal noise shaping", OFFSET(tns), AV_OPT_TYPE_BOOL, { .i64 = -1 }, -1, 1, AE },

    { "aac_pns", "Perceptual noise substitution", OFFSET(pns), AV_OPT_TYPE_BOOL, { .i64 = -1 }, -1, 1, AE },
    { "pns",     "Perceptual noise substitution", OFFSET(pns), AV_OPT_TYPE_BOOL, { .i64 = -1 }, -1, 1, AE },

    { "peak_bit_rate", "Whole-stream peak bitrate ceiling in bits/s (0 = unlimited)", OFFSET(peak_bit_rate), AV_OPT_TYPE_INT, { .i64 = 0 }, 0, INT_MAX, AE },
    { "max_bit_rate",  "Whole-stream peak bitrate ceiling in bits/s (alias for peak_bit_rate)", OFFSET(peak_bit_rate), AV_OPT_TYPE_INT, { .i64 = 0 }, 0, INT_MAX, AE },

    { "drc_profile", "DRC profile", OFFSET(drc_profile), AV_OPT_TYPE_INT, { .i64 = FAAC_DRC_OFF }, FAAC_DRC_OFF, FAAC_DRC_SPEECH, AE, .unit = "drc_profile" },
        { "off",         "No DRC element",  0, AV_OPT_TYPE_CONST, { .i64 = FAAC_DRC_OFF },         0, 0, AE, .unit = "drc_profile" },
        { "film",        "Film standard",   0, AV_OPT_TYPE_CONST, { .i64 = FAAC_DRC_FILM },        0, 0, AE, .unit = "drc_profile" },
        { "film_light",  "Film light",      0, AV_OPT_TYPE_CONST, { .i64 = FAAC_DRC_FILM_LIGHT },  0, 0, AE, .unit = "drc_profile" },
        { "music",       "Music standard",  0, AV_OPT_TYPE_CONST, { .i64 = FAAC_DRC_MUSIC },       0, 0, AE, .unit = "drc_profile" },
        { "music_light", "Music light",     0, AV_OPT_TYPE_CONST, { .i64 = FAAC_DRC_MUSIC_LIGHT }, 0, 0, AE, .unit = "drc_profile" },
        { "speech",      "Speech standard", 0, AV_OPT_TYPE_CONST, { .i64 = FAAC_DRC_SPEECH },      0, 0, AE, .unit = "drc_profile" },

    { "drc_ref_level", "DRC program reference level in LUFS/dBFS (-31.75 to 0.0)", OFFSET(drc_ref_level), AV_OPT_TYPE_FLOAT, { .dbl = -23.0 }, -31.75, 0.0, AE },

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
