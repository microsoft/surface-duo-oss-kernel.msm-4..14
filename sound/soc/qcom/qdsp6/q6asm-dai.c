// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2011-2017, The Linux Foundation. All rights reserved.
// Copyright (c) 2018, Linaro Limited

#include <linux/init.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <sound/soc.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <linux/spinlock.h>
#include <sound/compress_driver.h>
#include <asm/dma.h>
#include <linux/dma-mapping.h>
#include <linux/of_device.h>
#include <sound/pcm_params.h>
#include "q6asm.h"
#include "q6routing.h"
#include "q6dsp-errno.h"

#define DRV_NAME	"q6asm-fe-dai"

#define PLAYBACK_MIN_NUM_PERIODS    2
#define PLAYBACK_MAX_NUM_PERIODS   8
#define PLAYBACK_MAX_PERIOD_SIZE    65536
#define PLAYBACK_MIN_PERIOD_SIZE    128
#define CAPTURE_MIN_NUM_PERIODS     2
#define CAPTURE_MAX_NUM_PERIODS     8
#define CAPTURE_MAX_PERIOD_SIZE     4096
#define CAPTURE_MIN_PERIOD_SIZE     320
#define SID_MASK_DEFAULT	0xF

/* Default values used if user space does not set */
#define COMPR_PLAYBACK_MIN_FRAGMENT_SIZE (8 * 1024)
#define COMPR_PLAYBACK_MAX_FRAGMENT_SIZE (128 * 1024)
#define COMPR_PLAYBACK_MIN_NUM_FRAGMENTS (4)
#define COMPR_PLAYBACK_MAX_NUM_FRAGMENTS (16 * 4)
#define Q6ASM_DAI_TX_RX	0
#define Q6ASM_DAI_TX	1
#define Q6ASM_DAI_RX	2


enum stream_state {
	Q6ASM_STREAM_IDLE = 0,
	Q6ASM_STREAM_STOPPED,
	Q6ASM_STREAM_RUNNING,
};

struct q6asm_dai_rtd {
	struct snd_pcm_substream *substream;
	struct snd_compr_stream *cstream;
	struct snd_compr_params codec_param;
	struct snd_dma_buffer dma_buffer;

	phys_addr_t phys;
	void    *buffer; /* virtual address */
	spinlock_t lock;
	int xrun;
	unsigned int pcm_size;
	unsigned int pcm_count;
	unsigned int pcm_irq_pos;       /* IRQ position */
	unsigned int periods;

	unsigned int byte_offset;
	unsigned int bytes_sent;
	unsigned int bytes_received;
	unsigned int copy_pointer;
	unsigned int copied_total;

	uint16_t bits_per_sample;
	uint16_t source; /* Encoding source bit mask */
	struct audio_client *audio_client;
	uint16_t session_id;
	enum stream_state state;
};

struct q6asm_dai_data {
	long long int sid;
};

static struct snd_pcm_hardware q6asm_dai_hardware_capture = {
	.info =                 (SNDRV_PCM_INFO_MMAP |
				SNDRV_PCM_INFO_BLOCK_TRANSFER |
				SNDRV_PCM_INFO_MMAP_VALID |
				SNDRV_PCM_INFO_INTERLEAVED |
				SNDRV_PCM_INFO_PAUSE | SNDRV_PCM_INFO_RESUME),
	.formats =              (SNDRV_PCM_FMTBIT_S16_LE |
				SNDRV_PCM_FMTBIT_S24_LE),
	.rates =                SNDRV_PCM_RATE_8000_48000,
	.rate_min =             8000,
	.rate_max =             48000,
	.channels_min =         1,
	.channels_max =         4,
	.buffer_bytes_max =     CAPTURE_MAX_NUM_PERIODS *
				CAPTURE_MAX_PERIOD_SIZE,
	.period_bytes_min =	CAPTURE_MIN_PERIOD_SIZE,
	.period_bytes_max =     CAPTURE_MAX_PERIOD_SIZE,
	.periods_min =          CAPTURE_MIN_NUM_PERIODS,
	.periods_max =          CAPTURE_MAX_NUM_PERIODS,
	.fifo_size =            0,
};

static struct snd_pcm_hardware q6asm_dai_hardware_playback = {
	.info =                 (SNDRV_PCM_INFO_MMAP |
				SNDRV_PCM_INFO_BLOCK_TRANSFER |
				SNDRV_PCM_INFO_MMAP_VALID |
				SNDRV_PCM_INFO_INTERLEAVED |
				SNDRV_PCM_INFO_PAUSE | SNDRV_PCM_INFO_RESUME),
	.formats =              (SNDRV_PCM_FMTBIT_S16_LE |
				SNDRV_PCM_FMTBIT_S24_LE),
	.rates =                SNDRV_PCM_RATE_8000_192000,
	.rate_min =             8000,
	.rate_max =             192000,
	.channels_min =         1,
	.channels_max =         8,
	.buffer_bytes_max =     (PLAYBACK_MAX_NUM_PERIODS *
				PLAYBACK_MAX_PERIOD_SIZE),
	.period_bytes_min =	PLAYBACK_MIN_PERIOD_SIZE,
	.period_bytes_max =     PLAYBACK_MAX_PERIOD_SIZE,
	.periods_min =          PLAYBACK_MIN_NUM_PERIODS,
	.periods_max =          PLAYBACK_MAX_NUM_PERIODS,
	.fifo_size =            0,
};

#define Q6ASM_FEDAI_DRIVER(num) { \
		.playback = {						\
			.stream_name = "MultiMedia"#num" Playback",	\
			.rates = (SNDRV_PCM_RATE_8000_192000|		\
					SNDRV_PCM_RATE_KNOT),		\
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |		\
					SNDRV_PCM_FMTBIT_S24_LE),	\
			.channels_min = 1,				\
			.channels_max = 8,				\
			.rate_min =     8000,				\
			.rate_max =	192000,				\
		},							\
		.capture = {						\
			.stream_name = "MultiMedia"#num" Capture",	\
			.rates = (SNDRV_PCM_RATE_8000_48000|		\
					SNDRV_PCM_RATE_KNOT),		\
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |		\
				    SNDRV_PCM_FMTBIT_S24_LE),		\
			.channels_min = 1,				\
			.channels_max = 4,				\
			.rate_min =     8000,				\
			.rate_max =	48000,				\
		},							\
		.name = "MultiMedia"#num,				\
		.probe = fe_dai_probe,					\
		.id = MSM_FRONTEND_DAI_MULTIMEDIA##num,			\
	}

/* Conventional and unconventional sample rate supported */
static unsigned int supported_sample_rates[] = {
	8000, 11025, 12000, 16000, 22050, 24000, 32000, 44100, 48000,
	88200, 96000, 176400, 192000
};

static struct snd_pcm_hw_constraint_list constraints_sample_rates = {
	.count = ARRAY_SIZE(supported_sample_rates),
	.list = supported_sample_rates,
	.mask = 0,
};

static void event_handler(uint32_t opcode, uint32_t token,
			  uint32_t *payload, void *priv)
{
	struct q6asm_dai_rtd *prtd = priv;
	struct snd_pcm_substream *substream = prtd->substream;

	switch (opcode) {
	case ASM_CLIENT_EVENT_CMD_RUN_DONE:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			q6asm_write_async(prtd->audio_client,
				   prtd->pcm_count, 0, 0, NO_TIMESTAMP);
		break;
	case ASM_CLIENT_EVENT_CMD_EOS_DONE:
		prtd->state = Q6ASM_STREAM_STOPPED;
		break;
	case ASM_CLIENT_EVENT_DATA_WRITE_DONE: {
		prtd->pcm_irq_pos += prtd->pcm_count;
		snd_pcm_period_elapsed(substream);
		if (prtd->state == Q6ASM_STREAM_RUNNING)
			q6asm_write_async(prtd->audio_client,
					   prtd->pcm_count, 0, 0, NO_TIMESTAMP);

		break;
		}
	case ASM_CLIENT_EVENT_DATA_READ_DONE:
		prtd->pcm_irq_pos += prtd->pcm_count;
		snd_pcm_period_elapsed(substream);
		if (prtd->state == Q6ASM_STREAM_RUNNING)
			q6asm_read(prtd->audio_client);

		break;
	default:
		break;
	}
}

static int q6asm_dai_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *soc_prtd = substream->private_data;
	struct q6asm_dai_rtd *prtd = runtime->private_data;
	struct snd_soc_component *c = snd_soc_rtdcom_lookup(soc_prtd, DRV_NAME);
	struct q6asm_dai_data *pdata;
	int ret, i;

	pdata = snd_soc_component_get_drvdata(c);
	if (!pdata)
		return -EINVAL;

	if (!prtd || !prtd->audio_client) {
		pr_err("%s: private data null or audio client freed\n",
			__func__);
		return -EINVAL;
	}

	prtd->pcm_count = snd_pcm_lib_period_bytes(substream);
	prtd->pcm_irq_pos = 0;
	/* rate and channels are sent to audio driver */
	if (prtd->state) {
		/* clear the previous setup if any  */
		q6asm_cmd(prtd->audio_client, CMD_CLOSE);
		q6asm_unmap_memory_regions(substream->stream,
					   prtd->audio_client);
		q6routing_stream_close(soc_prtd->dai_link->id,
					 substream->stream);
	}

	ret = q6asm_map_memory_regions(substream->stream, prtd->audio_client,
				       prtd->phys,
				       (prtd->pcm_size / prtd->periods),
				       prtd->periods);

	if (ret < 0) {
		pr_err("Audio Start: Buffer Allocation failed rc = %d\n",
							ret);
		return -ENOMEM;
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		ret = q6asm_open_write(prtd->audio_client, FORMAT_LINEAR_PCM,
				       prtd->bits_per_sample);
	} else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		ret = q6asm_open_read(prtd->audio_client, FORMAT_LINEAR_PCM,
				       prtd->bits_per_sample);
	}

	if (ret < 0) {
		pr_err("%s: q6asm_open_write failed\n", __func__);
		q6asm_audio_client_free(prtd->audio_client);
		prtd->audio_client = NULL;
		return -ENOMEM;
	}

	prtd->session_id = q6asm_get_session_id(prtd->audio_client);
	ret = q6routing_stream_open(soc_prtd->dai_link->id, LEGACY_PCM_MODE,
			      prtd->session_id, substream->stream);
	if (ret) {
		pr_err("%s: stream reg failed ret:%d\n", __func__, ret);
		return ret;
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		ret = q6asm_media_format_block_multi_ch_pcm(
				prtd->audio_client, runtime->rate,
				runtime->channels, NULL,
				prtd->bits_per_sample);
	} else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		ret = q6asm_enc_cfg_blk_pcm_format_support(prtd->audio_client,
					runtime->rate, runtime->channels,
					prtd->bits_per_sample);

		/* Queue the buffers */
		for (i = 0; i < runtime->periods; i++)
			q6asm_read(prtd->audio_client);

	}
	if (ret < 0)
		pr_info("%s: CMD Format block failed\n", __func__);

	prtd->state = Q6ASM_STREAM_RUNNING;

	return 0;
}

static int q6asm_dai_trigger(struct snd_pcm_substream *substream, int cmd)
{
	int ret = 0;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct q6asm_dai_rtd *prtd = runtime->private_data;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		ret = q6asm_run_nowait(prtd->audio_client, 0, 0, 0);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		prtd->state = Q6ASM_STREAM_STOPPED;
		ret = q6asm_cmd_nowait(prtd->audio_client, CMD_EOS);
		break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		ret = q6asm_cmd_nowait(prtd->audio_client, CMD_PAUSE);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int q6asm_dai_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *soc_prtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = soc_prtd->cpu_dai;
	struct snd_soc_component *c = snd_soc_rtdcom_lookup(soc_prtd, DRV_NAME);
	struct q6asm_dai_rtd *prtd;
	struct q6asm_dai_data *pdata;
	struct device *dev = c->dev;
	int ret = 0;
	int stream_id;

	stream_id = cpu_dai->driver->id;

	pdata = snd_soc_component_get_drvdata(c);
	if (!pdata) {
		pr_err("Drv data not found ..\n");
		return -EINVAL;
	}

	prtd = kzalloc(sizeof(struct q6asm_dai_rtd), GFP_KERNEL);
	if (prtd == NULL)
		return -ENOMEM;

	prtd->substream = substream;
	prtd->audio_client = q6asm_audio_client_alloc(dev,
				(q6asm_cb)event_handler, prtd, stream_id,
				LEGACY_PCM_MODE);
	if (!prtd->audio_client) {
		pr_info("%s: Could not allocate memory\n", __func__);
		kfree(prtd);
		return -ENOMEM;
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		runtime->hw = q6asm_dai_hardware_playback;
	else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		runtime->hw = q6asm_dai_hardware_capture;

	ret = snd_pcm_hw_constraint_list(runtime, 0,
				SNDRV_PCM_HW_PARAM_RATE,
				&constraints_sample_rates);
	if (ret < 0)
		pr_info("snd_pcm_hw_constraint_list failed\n");
	/* Ensure that buffer size is a multiple of period size */
	ret = snd_pcm_hw_constraint_integer(runtime,
					    SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0)
		pr_info("snd_pcm_hw_constraint_integer failed\n");

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		ret = snd_pcm_hw_constraint_minmax(runtime,
			SNDRV_PCM_HW_PARAM_BUFFER_BYTES,
			PLAYBACK_MIN_NUM_PERIODS * PLAYBACK_MIN_PERIOD_SIZE,
			PLAYBACK_MAX_NUM_PERIODS * PLAYBACK_MAX_PERIOD_SIZE);
		if (ret < 0) {
			pr_err("constraint for buffer bytes min max ret = %d\n",
									ret);
		}
	}

	ret = snd_pcm_hw_constraint_step(runtime, 0,
		SNDRV_PCM_HW_PARAM_PERIOD_BYTES, 32);
	if (ret < 0) {
		pr_err("constraint for period bytes step ret = %d\n",
								ret);
	}
	ret = snd_pcm_hw_constraint_step(runtime, 0,
		SNDRV_PCM_HW_PARAM_BUFFER_BYTES, 32);
	if (ret < 0) {
		pr_err("constraint for buffer bytes step ret = %d\n",
								ret);
	}

	runtime->private_data = prtd;

	snd_soc_set_runtime_hwparams(substream, &q6asm_dai_hardware_playback);

	runtime->dma_bytes = q6asm_dai_hardware_playback.buffer_bytes_max;


	if (pdata->sid < 0)
		prtd->phys = substream->dma_buffer.addr;
	else
		prtd->phys = substream->dma_buffer.addr | (pdata->sid << 32);

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);

	return 0;
}

static int q6asm_dai_close(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *soc_prtd = substream->private_data;
	struct q6asm_dai_rtd *prtd = runtime->private_data;

	if (prtd->audio_client) {
		if (prtd->state)
			q6asm_cmd(prtd->audio_client, CMD_CLOSE);

		q6asm_unmap_memory_regions(substream->stream,
					   prtd->audio_client);
		q6asm_audio_client_free(prtd->audio_client);
		prtd->audio_client = NULL;
	}
	q6routing_stream_close(soc_prtd->dai_link->id,
						substream->stream);
	kfree(prtd);
	return 0;
}

static snd_pcm_uframes_t q6asm_dai_pointer(struct snd_pcm_substream *substream)
{

	struct snd_pcm_runtime *runtime = substream->runtime;
	struct q6asm_dai_rtd *prtd = runtime->private_data;

	if (prtd->pcm_irq_pos >= prtd->pcm_size)
		prtd->pcm_irq_pos = 0;

	return bytes_to_frames(runtime, (prtd->pcm_irq_pos));
}

static int q6asm_dai_mmap(struct snd_pcm_substream *substream,
				struct vm_area_struct *vma)
{

	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *soc_prtd = substream->private_data;
	struct snd_soc_component *c = snd_soc_rtdcom_lookup(soc_prtd, DRV_NAME);
	struct device *dev = c->dev;

	return dma_mmap_coherent(dev, vma,
			runtime->dma_area, runtime->dma_addr,
			runtime->dma_bytes);
}

static int q6asm_dai_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct q6asm_dai_rtd *prtd = runtime->private_data;

	prtd->pcm_size = params_buffer_bytes(params);
	prtd->periods = params_periods(params);

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		prtd->bits_per_sample = 16;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		prtd->bits_per_sample = 24;
		break;
	}

	return 0;
}

static struct snd_pcm_ops q6asm_dai_ops = {
	.open           = q6asm_dai_open,
	.hw_params	= q6asm_dai_hw_params,
	.close          = q6asm_dai_close,
	.ioctl          = snd_pcm_lib_ioctl,
	.prepare        = q6asm_dai_prepare,
	.trigger        = q6asm_dai_trigger,
	.pointer        = q6asm_dai_pointer,
	.mmap		= q6asm_dai_mmap,
};

static void compress_event_handler(uint32_t opcode, uint32_t token,
				   uint32_t *payload, void *priv)
{
	struct q6asm_dai_rtd *prtd = priv;
	struct snd_compr_stream *substream = prtd->cstream;
	unsigned long flags;
	uint64_t avail;

	switch (opcode) {
	case ASM_CLIENT_EVENT_CMD_RUN_DONE:
		spin_lock_irqsave(&prtd->lock, flags);
		avail = prtd->bytes_received - prtd->bytes_sent;
		if (!prtd->bytes_sent) {
			if (avail < substream->runtime->fragment_size) {
				prtd->xrun = 1;
			} else {
				q6asm_write_async(prtd->audio_client,
						  prtd->pcm_count,
						  0, 0, NO_TIMESTAMP);
				prtd->bytes_sent += prtd->pcm_count;
			}
		}

		spin_unlock_irqrestore(&prtd->lock, flags);
		break;
	case ASM_CLIENT_EVENT_CMD_EOS_DONE:
		prtd->state = Q6ASM_STREAM_STOPPED;
		break;
	case ASM_CLIENT_EVENT_DATA_WRITE_DONE:
		spin_lock_irqsave(&prtd->lock, flags);
		prtd->byte_offset += prtd->pcm_count;
		prtd->copied_total += prtd->pcm_count;

		if (prtd->byte_offset >= prtd->pcm_size)
			prtd->byte_offset -= prtd->pcm_size;

		snd_compr_fragment_elapsed(substream);
		if (prtd->state != Q6ASM_STREAM_RUNNING) {
			spin_unlock_irqrestore(&prtd->lock, flags);
			break;
		}

		avail = prtd->bytes_received - prtd->bytes_sent;
		if (avail < substream->runtime->fragment_size) {
			prtd->xrun = 1;
		} else {
			q6asm_write_async(prtd->audio_client,
					   prtd->pcm_count, 0, 0, NO_TIMESTAMP);
			prtd->bytes_sent += prtd->pcm_count;
		}

		spin_unlock_irqrestore(&prtd->lock, flags);

		break;
	default:
		break;
	}
}


static int q6asm_dai_compr_open(struct snd_compr_stream *stream)
{
	struct snd_soc_pcm_runtime *rtd = stream->private_data;
	struct snd_soc_component *c = snd_soc_rtdcom_lookup(rtd, DRV_NAME);
	struct snd_compr_runtime *runtime = stream->runtime;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct q6asm_dai_data *pdata;
	struct device *dev = c->dev;
	struct q6asm_dai_rtd *prtd;
	int stream_id;

	stream_id = cpu_dai->driver->id;
	pdata = snd_soc_component_get_drvdata(c);
	if (!pdata) {
		dev_err(dev, "Drv data not found ..\n");
		return -EINVAL;
	}

	prtd = kzalloc(sizeof(*prtd), GFP_KERNEL);
	if (prtd == NULL)
		return -ENOMEM;

	prtd->cstream = stream;
	prtd->audio_client = q6asm_audio_client_alloc(dev,
					(q6asm_cb)compress_event_handler,
					prtd, stream_id, LEGACY_PCM_MODE);
	if (!prtd->audio_client) {
		dev_err(dev, "Could not allocate memory\n");
		kfree(prtd);
		return -ENOMEM;
	}

	spin_lock_init(&prtd->lock);
	runtime->private_data = prtd;

	return 0;
}

static int q6asm_dai_compr_free(struct snd_compr_stream *stream)
{
	struct snd_compr_runtime *runtime = stream->runtime;
	struct q6asm_dai_rtd *prtd = runtime->private_data;
	struct snd_soc_pcm_runtime *rtd = stream->private_data;

	if (prtd->audio_client) {
		if (prtd->state)
			q6asm_cmd(prtd->audio_client, CMD_CLOSE);

		q6asm_unmap_memory_regions(stream->direction,
					   prtd->audio_client);
		q6asm_audio_client_free(prtd->audio_client);
		prtd->audio_client = NULL;
	}
	q6routing_stream_close(rtd->dai_link->id, stream->direction);
	kfree(prtd);

	return 0;
}

static int q6asm_dai_compr_set_params(struct snd_compr_stream *stream,
				      struct snd_compr_params *params)
{

	struct snd_compr_runtime *runtime = stream->runtime;
	struct q6asm_dai_rtd *prtd = runtime->private_data;
	struct snd_soc_pcm_runtime *rtd = stream->private_data;
	struct snd_soc_component *c = snd_soc_rtdcom_lookup(rtd, DRV_NAME);
	int dir = stream->direction;
	struct q6asm_dai_data *pdata;
	struct device *dev = c->dev;
	int ret;

	memcpy(&prtd->codec_param, params, sizeof(*params));

	pdata = snd_soc_component_get_drvdata(c);
	if (!pdata)
		return -EINVAL;

	if (!prtd || !prtd->audio_client) {
		dev_err(dev, "private data null or audio client freed\n");
		return -EINVAL;
	}

	runtime->fragments = prtd->codec_param.buffer.fragments;
	runtime->fragment_size = prtd->codec_param.buffer.fragment_size;
	prtd->periods = runtime->fragments;
	prtd->pcm_count = runtime->fragment_size;
	prtd->pcm_size = runtime->fragments * runtime->fragment_size;

	ret = snd_dma_alloc_pages(SNDRV_DMA_TYPE_DEV, dev, prtd->pcm_size,
				  &prtd->dma_buffer);
	if (ret) {
		dev_err(dev, "Cannot allocate buffer(s)\n");
		return ret;
	}

	if (pdata->sid < 0)
		prtd->phys = prtd->dma_buffer.addr;
	else
		prtd->phys = prtd->dma_buffer.addr | (pdata->sid << 32);

	prtd->buffer = prtd->dma_buffer.area;
	prtd->copy_pointer = 0;

	prtd->bits_per_sample = 16;
	if (dir == SND_COMPRESS_PLAYBACK) {
		ret = q6asm_open_write(prtd->audio_client, params->codec.id,
					prtd->bits_per_sample);
	}

	if (ret < 0) {
		dev_err(dev, "q6asm_open_write failed\n");
		q6asm_audio_client_free(prtd->audio_client);
		prtd->audio_client = NULL;
		return -ENOMEM;
	}

	prtd->session_id = q6asm_get_session_id(prtd->audio_client);
	ret = q6routing_stream_open(rtd->dai_link->id, LEGACY_PCM_MODE,
			      prtd->session_id, dir);
	if (ret) {
		dev_err(dev, "Stream reg failed ret:%d\n", ret);
		return ret;
	}

	ret = q6asm_map_memory_regions(dir, prtd->audio_client, prtd->phys,
				       (prtd->pcm_size / prtd->periods),
				       prtd->periods);

	if (ret < 0) {
		dev_err(dev, "Buffer Mapping failed ret:%d\n", ret);
		return -ENOMEM;
	}

	prtd->state = Q6ASM_STREAM_RUNNING;

	return 0;
}

static int q6asm_dai_compr_trigger(struct snd_compr_stream *stream, int cmd)
{
	struct snd_compr_runtime *runtime = stream->runtime;
	struct q6asm_dai_rtd *prtd = runtime->private_data;
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		ret = q6asm_run_nowait(prtd->audio_client, 0, 0, 0);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		prtd->state = Q6ASM_STREAM_STOPPED;
		ret = q6asm_cmd_nowait(prtd->audio_client, CMD_EOS);
		break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		ret = q6asm_cmd_nowait(prtd->audio_client, CMD_PAUSE);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int q6asm_dai_compr_pointer(struct snd_compr_stream *stream,
		struct snd_compr_tstamp *tstamp)
{
	struct snd_compr_runtime *runtime = stream->runtime;
	struct q6asm_dai_rtd *prtd = runtime->private_data;
	unsigned long flags;

	spin_lock_irqsave(&prtd->lock, flags);

	tstamp->byte_offset = prtd->byte_offset;
	tstamp->copied_total = prtd->copied_total;

	spin_unlock_irqrestore(&prtd->lock, flags);

	return 0;
}

static int q6asm_dai_compr_copy(struct snd_compr_stream *stream,
				char __user *buf, size_t count)
{
	struct snd_compr_runtime *runtime = stream->runtime;
	struct q6asm_dai_rtd *prtd = runtime->private_data;
	uint64_t avail = 0;
	unsigned long flags;
	size_t copy;
	void *dstn;

	dstn = prtd->buffer + prtd->copy_pointer;
	if (count < prtd->pcm_size - prtd->copy_pointer) {
		if (copy_from_user(dstn, buf, count))
			return -EFAULT;

		prtd->copy_pointer += count;
	} else {
		copy = prtd->pcm_size - prtd->copy_pointer;
		if (copy_from_user(dstn, buf, copy))
			return -EFAULT;

		if (copy_from_user(prtd->buffer, buf + copy, count - copy))
			return -EFAULT;
		prtd->copy_pointer = count - copy;
	}

	spin_lock_irqsave(&prtd->lock, flags);
	prtd->bytes_received += count;

	if (prtd->state == Q6ASM_STREAM_RUNNING && prtd->xrun) {
		avail = prtd->bytes_received - prtd->copied_total;
		if (avail >= runtime->fragment_size) {
			prtd->xrun = 0;
			q6asm_write_async(prtd->audio_client,
				   prtd->pcm_count, 0, 0, NO_TIMESTAMP);
			prtd->bytes_sent += prtd->pcm_count;
		}
	}
	spin_unlock_irqrestore(&prtd->lock, flags);

	return count;
}

static int q6asm_dai_compr_mmap(struct snd_compr_stream *stream,
		struct vm_area_struct *vma)
{
	struct snd_compr_runtime *runtime = stream->runtime;
	struct q6asm_dai_rtd *prtd = runtime->private_data;
	struct snd_soc_pcm_runtime *rtd = stream->private_data;
	struct snd_soc_component *c = snd_soc_rtdcom_lookup(rtd, DRV_NAME);
	struct device *dev = c->dev;

	return dma_mmap_coherent(dev, vma,
			prtd->dma_buffer.area, prtd->dma_buffer.addr,
			prtd->dma_buffer.bytes);
}

static int q6asm_dai_compr_get_caps(struct snd_compr_stream *stream,
				    struct snd_compr_caps *caps)
{
	caps->direction = SND_COMPRESS_PLAYBACK;
	caps->min_fragment_size = COMPR_PLAYBACK_MIN_FRAGMENT_SIZE;
	caps->max_fragment_size = COMPR_PLAYBACK_MAX_FRAGMENT_SIZE;
	caps->min_fragments = COMPR_PLAYBACK_MIN_NUM_FRAGMENTS;
	caps->max_fragments = COMPR_PLAYBACK_MAX_NUM_FRAGMENTS;
	caps->num_codecs = 1;
	caps->codecs[0] = SND_AUDIOCODEC_MP3;

	return 0;
}

static int q6asm_dai_compr_get_codec_caps(struct snd_compr_stream *stream,
					  struct snd_compr_codec_caps *codec)
{
	switch (codec->codec) {
	case SND_AUDIOCODEC_MP3:
		codec->num_descriptors = 2;
		codec->descriptor[0].max_ch = 2;
		memcpy(codec->descriptor[0].sample_rates,
		       supported_sample_rates,
		       sizeof(supported_sample_rates));
		codec->descriptor[0].num_sample_rates =
			sizeof(supported_sample_rates)/sizeof(unsigned int);
		codec->descriptor[0].bit_rate[0] = 320; /* 320kbps */
		codec->descriptor[0].bit_rate[1] = 128;
		codec->descriptor[0].num_bitrates = 2;
		codec->descriptor[0].profiles = 0;
		codec->descriptor[0].modes = SND_AUDIOCHANMODE_MP3_STEREO;
		codec->descriptor[0].formats = 0;
		break;
	default:
		break;
	}

	return 0;
}

static struct snd_compr_ops q6asm_dai_compr_ops = {
	.open		= q6asm_dai_compr_open,
	.free		= q6asm_dai_compr_free,
	.set_params	= q6asm_dai_compr_set_params,
	.pointer	= q6asm_dai_compr_pointer,
	.trigger	= q6asm_dai_compr_trigger,
	.get_caps	= q6asm_dai_compr_get_caps,
	.get_codec_caps	= q6asm_dai_compr_get_codec_caps,
	.mmap		= q6asm_dai_compr_mmap,
	.copy		= q6asm_dai_compr_copy,
};

static int q6asm_dai_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_pcm_substream *psubstream, *csubstream;
	struct snd_soc_component *c = snd_soc_rtdcom_lookup(rtd, DRV_NAME);
	struct snd_pcm *pcm = rtd->pcm;
	struct device *dev;
	int size, ret;

	dev = c->dev;
	size = q6asm_dai_hardware_playback.buffer_bytes_max;
	psubstream = pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream;
	if (psubstream) {
		ret = snd_dma_alloc_pages(SNDRV_DMA_TYPE_DEV, dev, size,
					  &psubstream->dma_buffer);
		if (ret) {
			dev_err(dev, "Cannot allocate buffer(s)\n");
			return ret;
		}
	}

	csubstream = pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream;
	if (csubstream) {
		ret = snd_dma_alloc_pages(SNDRV_DMA_TYPE_DEV, dev, size,
					  &csubstream->dma_buffer);
		if (ret) {
			dev_err(dev, "Cannot allocate buffer(s)\n");
			if (psubstream)
				snd_dma_free_pages(&psubstream->dma_buffer);
			return ret;
		}
	}

	return ret;
}

static void q6asm_dai_pcm_free(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	int i;

	for (i = 0; i < ARRAY_SIZE(pcm->streams); i++) {
		substream = pcm->streams[i].substream;
		if (substream) {
			snd_dma_free_pages(&substream->dma_buffer);
			substream->dma_buffer.area = NULL;
			substream->dma_buffer.addr = 0;
		}
	}
}

static const struct snd_soc_dapm_route afe_pcm_routes[] = {
	{"MM_DL1",  NULL, "MultiMedia1 Playback" },
	{"MM_DL2",  NULL, "MultiMedia2 Playback" },
	{"MM_DL3",  NULL, "MultiMedia3 Playback" },
	{"MM_DL4",  NULL, "MultiMedia4 Playback" },
	{"MM_DL5",  NULL, "MultiMedia5 Playback" },
	{"MM_DL6",  NULL, "MultiMedia6 Playback" },
	{"MM_DL7",  NULL, "MultiMedia7 Playback" },
	{"MM_DL7",  NULL, "MultiMedia8 Playback" },
	{"MultiMedia1 Capture", NULL, "MM_UL1"},
	{"MultiMedia2 Capture", NULL, "MM_UL2"},
	{"MultiMedia3 Capture", NULL, "MM_UL3"},
	{"MultiMedia4 Capture", NULL, "MM_UL4"},
	{"MultiMedia5 Capture", NULL, "MM_UL5"},
	{"MultiMedia6 Capture", NULL, "MM_UL6"},
	{"MultiMedia7 Capture", NULL, "MM_UL7"},
	{"MultiMedia8 Capture", NULL, "MM_UL8"},

};

static int fe_dai_probe(struct snd_soc_dai *dai)
{
	struct snd_soc_dapm_context *dapm;

	dapm = snd_soc_component_get_dapm(dai->component);
	snd_soc_dapm_add_routes(dapm, afe_pcm_routes,
				ARRAY_SIZE(afe_pcm_routes));

	return 0;
}


static const struct snd_soc_component_driver q6asm_fe_dai_component = {
	.name		= DRV_NAME,
	.ops		= &q6asm_dai_ops,
	.pcm_new	= q6asm_dai_pcm_new,
	.pcm_free	= q6asm_dai_pcm_free,
	.compr_ops	= &q6asm_dai_compr_ops,
};

static struct snd_soc_dai_driver q6asm_fe_dais[] = {
	Q6ASM_FEDAI_DRIVER(1),
	Q6ASM_FEDAI_DRIVER(2),
	Q6ASM_FEDAI_DRIVER(3),
	Q6ASM_FEDAI_DRIVER(4),
	Q6ASM_FEDAI_DRIVER(5),
	Q6ASM_FEDAI_DRIVER(6),
	Q6ASM_FEDAI_DRIVER(7),
	Q6ASM_FEDAI_DRIVER(8),
};

static int of_q6asm_parse_dai_data(struct device *dev,
				    struct q6asm_dai_data *pdata)
{
	static struct snd_soc_dai_driver *dai_drv;
	struct snd_soc_pcm_stream empty_stream;
	struct device_node *node;
	int ret, id, dir;

	memset(&empty_stream, 0, sizeof(empty_stream));

	for_each_child_of_node(dev->of_node, node) {
		ret = of_property_read_u32(node, "reg", &id);
		if (ret || id > MAX_SESSIONS || id < 0) {
			dev_err(dev, "valid dai id not found:%d\n", ret);
			continue;
		}

		dai_drv = &q6asm_fe_dais[id];

		ret = of_property_read_u32(node, "direction", &dir);
		if (ret)
			continue;

		if (dir == Q6ASM_DAI_RX)
			dai_drv->capture = empty_stream;
		else if (dir == Q6ASM_DAI_TX)
			dai_drv->playback = empty_stream;

		if (of_property_read_bool(node, "is-compress-dai"))
			dai_drv->compress_new = snd_soc_new_compress;
	}

	return 0;
}

static int q6asm_dai_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct of_phandle_args args;
	struct q6asm_dai_data *pdata;
	int rc;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	rc = of_parse_phandle_with_fixed_args(node, "iommus", 1, 0, &args);
	if (rc < 0)
		pdata->sid = -1;
	else
		pdata->sid = args.args[0] & SID_MASK_DEFAULT;

	dev_set_drvdata(dev, pdata);

	of_q6asm_parse_dai_data(dev, pdata);

	return devm_snd_soc_register_component(dev, &q6asm_fe_dai_component,
					q6asm_fe_dais,
					ARRAY_SIZE(q6asm_fe_dais));
}

static const struct of_device_id q6asm_dai_device_id[] = {
	{ .compatible = "qcom,q6asm-dais" },
	{},
};
MODULE_DEVICE_TABLE(of, q6asm_dai_device_id);

static struct platform_driver q6asm_dai_platform_driver = {
	.driver = {
		.name = "q6asm-dai",
		.of_match_table = of_match_ptr(q6asm_dai_device_id),
	},
	.probe = q6asm_dai_probe,
};
module_platform_driver(q6asm_dai_platform_driver);

MODULE_DESCRIPTION("Q6ASM dai driver");
MODULE_LICENSE("GPL v2");
