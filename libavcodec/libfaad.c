/*
 * FAAD AAC decoder wrapper using libfaad3
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

#include <faad3.h>

#include "libavutil/channel_layout.h"
#include "libavutil/common.h"
#include "libavutil/internal.h"
#include "libavutil/log.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "avcodec.h"
#include "codec_internal.h"
#include "decode.h"
#include "profiles.h"

typedef struct FaadDecContext {
    const AVClass *class;
    faad_decoder *handle;
    uint8_t *pcm_buf;
    uint32_t pcm_buf_size;

    AVChannelLayout downmix_layout;
    float drc_scale;
    int drc_boost;
    int drc_cut;
    float drc_target_level;

    int sample_size;      /* bytes per sample */
    int initialized;
} FaadDecContext;

static void faad_update_format(AVCodecContext *avctx, const faad_stream_info *info)
{
    if (info->sample_rate > 0)
        avctx->sample_rate = info->sample_rate;

    if (info->channel_mask > 0) {
        av_channel_layout_uninit(&avctx->ch_layout);
        av_channel_layout_from_mask(&avctx->ch_layout, info->channel_mask);
    } else if (info->channels > 0) {
        av_channel_layout_uninit(&avctx->ch_layout);
        av_channel_layout_default(&avctx->ch_layout, info->channels);
    }

    switch (info->object_type) {
    case FAAD_OBJ_LC:
        avctx->profile = AV_PROFILE_AAC_LOW;
        break;
    case FAAD_OBJ_HE_AAC_V1:
        avctx->profile = AV_PROFILE_AAC_HE;
        break;
    case FAAD_OBJ_HE_AAC_V2:
        avctx->profile = AV_PROFILE_AAC_HE_V2;
        break;
    default:
        break;
    }
}

static av_cold int faad_decode_close(AVCodecContext *avctx)
{
    FaadDecContext *s = avctx->priv_data;

    if (s->handle)
        faad_decoder_close(&s->handle);

    av_freep(&s->pcm_buf);
    s->pcm_buf_size = 0;
    s->initialized  = 0;

    return 0;
}

static av_cold int faad_decode_init(AVCodecContext *avctx)
{
    FaadDecContext *s = avctx->priv_data;
    faad_library_info lib_info = { .struct_size = sizeof(lib_info) };
    faad_config cfg;
    faad_stream_info info = { .struct_size = sizeof(info) };
    faad_status status;

    status = faad_get_library_info(&lib_info);
    if (status != FAAD_OK) {
        av_log(avctx, AV_LOG_ERROR, "faad_get_library_info failed: %s (%d)\n",
               faad_strerror(status), status);
        return AVERROR_EXTERNAL;
    }

    status = faad_config_init(&cfg, sizeof(cfg));
    if (status != FAAD_OK) {
        av_log(avctx, AV_LOG_ERROR, "faad_config_init failed: %s (%d)\n",
               faad_strerror(status), status);
        return AVERROR_EXTERNAL;
    }

    if (avctx->extradata && avctx->extradata_size > 0)
        cfg.stream_format = FAAD_STREAM_RAW;
    else
        cfg.stream_format = FAAD_STREAM_ADTS;

    if (avctx->request_sample_fmt == AV_SAMPLE_FMT_S16) {
        cfg.output_format = FAAD_OUTPUT_16BIT;
        avctx->sample_fmt = AV_SAMPLE_FMT_S16;
        s->sample_size    = 2;
    } else {
        cfg.output_format = FAAD_OUTPUT_FLOAT;
        avctx->sample_fmt = AV_SAMPLE_FMT_FLT;
        s->sample_size    = 4;
    }

    /* Downmix configuration */
    if (s->downmix_layout.nb_channels > 0) {
        if (!av_channel_layout_compare(&s->downmix_layout, &(AVChannelLayout)AV_CHANNEL_LAYOUT_STEREO) ||
            !av_channel_layout_compare(&s->downmix_layout, &(AVChannelLayout)AV_CHANNEL_LAYOUT_STEREO_DOWNMIX)) {
            cfg.downmix_mode = FAAD_DOWNMIX_STEREO;
        } else if (!av_channel_layout_compare(&s->downmix_layout, &(AVChannelLayout)AV_CHANNEL_LAYOUT_MONO)) {
            cfg.downmix_mode = FAAD_DOWNMIX_MONO;
        } else {
            av_log(avctx, AV_LOG_WARNING, "Unsupported downmix layout requested\n");
            cfg.downmix_mode = FAAD_DOWNMIX_NONE;
        }
    } else {
        cfg.downmix_mode = FAAD_DOWNMIX_NONE;
    }

    /* DRC configuration */
    if (s->drc_scale >= 0.0f) {
        uint8_t scaled = (uint8_t)av_clip((int)(s->drc_scale * 127.0f + 0.5f), 0, 127);
        cfg.drc_cut   = scaled;
        cfg.drc_boost = scaled;
    }
    if (s->drc_cut >= 0)
        cfg.drc_cut = av_clip(s->drc_cut, 0, 127);
    if (s->drc_boost >= 0)
        cfg.drc_boost = av_clip(s->drc_boost, 0, 127);
    if (s->drc_target_level != 0.0f)
        cfg.drc_target_level = av_clipf(s->drc_target_level, -31.75f, -10.0f);

    status = faad_decoder_open(&cfg, avctx->extradata, avctx->extradata_size, &s->handle);
    if (status != FAAD_OK) {
        av_log(avctx, AV_LOG_ERROR, "faad_decoder_open failed: %s (%d)\n",
               faad_strerror(status), status);
        return AVERROR_EXTERNAL;
    }

    status = faad_decoder_get_info(s->handle, &info);
    if (status != FAAD_OK) {
        av_log(avctx, AV_LOG_ERROR, "faad_decoder_get_info failed: %s (%d)\n",
               faad_strerror(status), status);
        faad_decode_close(avctx);
        return AVERROR_EXTERNAL;
    }

    s->pcm_buf_size = info.max_output_bytes;
    s->pcm_buf      = av_malloc(s->pcm_buf_size + AV_INPUT_BUFFER_PADDING_SIZE);
    if (!s->pcm_buf) {
        faad_decode_close(avctx);
        return AVERROR(ENOMEM);
    }

    if (info.flags & FAAD_STREAM_FORMAT_KNOWN) {
        faad_update_format(avctx, &info);
    }

    s->initialized = 1;

    return 0;
}

static int faad_decode_packet(AVCodecContext *avctx, AVFrame *frame,
                              int *got_frame_ptr, AVPacket *avpkt)
{
    FaadDecContext *s = avctx->priv_data;
    const uint8_t *buf = avpkt->data;
    int buf_size = avpkt->size;
    uint32_t consumed = 0, written = 0, flags = 0;
    faad_status status;
    int ret;

    *got_frame_ptr = 0;

    if (!buf || buf_size == 0)
        return 0;

    status = faad_decode_frame(s->handle, buf, (uint32_t)buf_size, &consumed,
                               s->pcm_buf, s->pcm_buf_size, &written, &flags);

    if (status == FAAD_ERR_NEED_MORE_DATA) {
        return consumed > 0 ? (int)consumed : AVERROR(EAGAIN);
    }

    if (status != FAAD_OK) {
        av_log(avctx, AV_LOG_ERROR, "faad_decode_frame error: %s (%d), consumed %u bytes\n",
               faad_strerror(status), status, consumed);
        return consumed > 0 ? (int)consumed : AVERROR_INVALIDDATA;
    }

    if (flags & FAAD_FRAME_FORMAT_CHANGED) {
        faad_stream_info info = { .struct_size = sizeof(info) };
        status = faad_decoder_get_info(s->handle, &info);
        if (status != FAAD_OK) {
            av_log(avctx, AV_LOG_ERROR, "faad_decoder_get_info failed on format change: %s (%d)\n",
                   faad_strerror(status), status);
            return AVERROR_INVALIDDATA;
        }
        faad_update_format(avctx, &info);
    }

    if (written > 0) {
        int bytes_per_sample_group;

        if (written > s->pcm_buf_size) {
            av_log(avctx, AV_LOG_ERROR, "Decoded bytes (%u) exceed buffer size (%u)\n",
                   written, s->pcm_buf_size);
            return AVERROR_INVALIDDATA;
        }

        if (avctx->ch_layout.nb_channels <= 0 || s->sample_size <= 0) {
            av_log(avctx, AV_LOG_ERROR, "Invalid channels or sample size\n");
            return AVERROR_INVALIDDATA;
        }

        bytes_per_sample_group = avctx->ch_layout.nb_channels * s->sample_size;
        if (written % bytes_per_sample_group != 0) {
            av_log(avctx, AV_LOG_ERROR, "Decoded byte count %u is not a multiple of sample size group %d\n",
                   written, bytes_per_sample_group);
            return AVERROR_INVALIDDATA;
        }

        frame->nb_samples = written / bytes_per_sample_group;

        ret = ff_get_buffer(avctx, frame, 0);
        if (ret < 0)
            return ret;

        memcpy(frame->data[0], s->pcm_buf, written);
        *got_frame_ptr = 1;
    }

    return (int)consumed;
}

static av_cold void faad_decode_flush(AVCodecContext *avctx)
{
    FaadDecContext *s = avctx->priv_data;

    if (s->handle)
        faad_decoder_flush(s->handle);
}

#define OFFSET(x) offsetof(FaadDecContext, x)
#define AD AV_OPT_FLAG_AUDIO_PARAM | AV_OPT_FLAG_DECODING_PARAM

static const AVOption faad_dec_options[] = {
    { "downmix", "Request a specific channel layout from the decoder", OFFSET(downmix_layout), AV_OPT_TYPE_CHLAYOUT, { .str = NULL }, .flags = AD },
    { "drc_scale", "Percentage of dynamic range compression to apply [0.0 - 1.0]", OFFSET(drc_scale), AV_OPT_TYPE_FLOAT, { .dbl = -1.0 }, -1.0, 1.0, AD },
    { "drc_boost", "Dynamic Range Control: boost factor [0 - 127]", OFFSET(drc_boost), AV_OPT_TYPE_INT, { .i64 = -1 }, -1, 127, AD },
    { "drc_cut",   "Dynamic Range Control: attenuation factor [0 - 127]", OFFSET(drc_cut), AV_OPT_TYPE_INT, { .i64 = -1 }, -1, 127, AD },
    { "drc_target_level", "Dynamic Range Control: target loudness in dBFS (-31.75 to -10.0, 0 = off)", OFFSET(drc_target_level), AV_OPT_TYPE_FLOAT, { .dbl = 0.0 }, -31.75, 0.0, AD },
    { NULL }
};

static const AVClass faad_dec_class = {
    .class_name = "libfaad",
    .item_name  = av_default_item_name,
    .option     = faad_dec_options,
    .version    = LIBAVUTIL_VERSION_INT,
};

static const enum AVSampleFormat faad_sample_fmts[] = {
    AV_SAMPLE_FMT_FLT,
    AV_SAMPLE_FMT_S16,
    AV_SAMPLE_FMT_NONE,
};

const FFCodec ff_libfaad_decoder = {
    .p.name         = "libfaad",
    CODEC_LONG_NAME("libfaad AAC (Advanced Audio Coding)"),
    .p.type         = AVMEDIA_TYPE_AUDIO,
    .p.id           = AV_CODEC_ID_AAC,
    .priv_data_size = sizeof(FaadDecContext),
    .init           = faad_decode_init,
    FF_CODEC_DECODE_CB(faad_decode_packet),
    .close          = faad_decode_close,
    .flush          = faad_decode_flush,
    .p.capabilities = AV_CODEC_CAP_DR1 | AV_CODEC_CAP_CHANNEL_CONF,
    .caps_internal  = FF_CODEC_CAP_INIT_CLEANUP,
    CODEC_SAMPLEFMTS_ARRAY(faad_sample_fmts),
    .p.profiles     = NULL_IF_CONFIG_SMALL(ff_aac_profiles),
    .p.priv_class   = &faad_dec_class,
    .p.wrapper_name = "libfaad",
};
