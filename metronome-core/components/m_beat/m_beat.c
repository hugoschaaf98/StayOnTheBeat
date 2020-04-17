#include "m_beat.h"

#include <stdio.h>
#include <string.h>
#include <sys/fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"

#include "audio_file.h"

#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "esp_log.h"

static const char *TAG = "beat";

/**
 * Audio 
 * Here we use the built in DAC with i2s driver.
 * We can drive the DAC in a DMA style for more comfortability
 */
#include "driver/i2s.h"
#include "freertos/queue.h"

#define BEAT_MAX_SAMPLE_RATE 		44100UL			/* Hz, for mono audio. It hugely enough for a metronome.... */
#define BEAT_AUDIO_SAMPLE_RATE		16000UL

#define BEAT_PLAY_TASK_PRIORITY		10
/**
 * I2S with DMA configuration 
 * audio data can be up to @44.1kHz - 8 bits - stereo.
 */
#define BEAT_I2S_PORT_NUM			I2S_NUM_0		/* don't change it if use built in DAC */
#define	BEAT_I2S_BITS_PER_SAMPLE	I2S_BITS_PER_SAMPLE_16BIT
#define BEAT_I2S_DMA_BUF_LEN		1024			/* Size of a single dma buffer */
#define BEAT_I2S_DMA_BUF_COUNT		6				/* Count of dma buffers */

/* utils */
#define BEAT_MS_TO_SAMPLES(t)		((uint32_t)t*BEAT_AUDIO_SAMPLE_RATE/1000)
#define BEAT_I2S_WRITE(src, btw, bw) i2s_write(BEAT_I2S_PORT_NUM, src, btw, (size_t*)bw, portMAX_DELAY)

/* the play beat task handler and queue */
static TaskHandle_t xBeat_play_task_handle = NULL;
// static QueueHandle_t xi2s_queue_handle = NULL;

/* Buffer to store the beat sound data*/
// DRAM_ATTR static uint8_t* sound_buf = NULL;

WORD_ALIGNED_ATTR static uint8_t dummy_buf_127_values[BEAT_I2S_DMA_BUF_LEN] = {0};

/************************************************
 *  			AUDIO OUTPUT
 ***********************************************/

/**
 * Init the I2S driver to directly output audio data
 * via the 2 built-in DACs on GPIO25-GPIO26
 * 
 */
static void beat_i2s_init(void)
{
	i2s_config_t i2s_config = {
		.mode = I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN,
		.sample_rate = BEAT_AUDIO_SAMPLE_RATE,
		.bits_per_sample = BEAT_I2S_BITS_PER_SAMPLE,		
		.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,		/* RIGHT -> DAC1 ie GPIO25; LEFT -> DAC2 ie GPIO26*/
		.communication_format = I2S_COMM_FORMAT_PCM,
		.intr_alloc_flags = 0,								/* default interrupt priority */
		.dma_buf_count = BEAT_I2S_DMA_BUF_COUNT,			/* should be between 2 and 128 */ 									
		.dma_buf_len = BEAT_I2S_DMA_BUF_LEN,				/* should be between 8 and 1024 */							
		.use_apll = false,
		.tx_desc_auto_clear = false
	};
	i2s_driver_install(BEAT_I2S_PORT_NUM, &i2s_config, 0, NULL);   /* install and start i2s driver */
	i2s_set_dac_mode(I2S_DAC_CHANNEL_BOTH_EN);
	// i2s_set_clk(BEAT_I2S_PORT_NUM, BEAT_AUDIO_SAMPLE_RATE, BEAT_I2S_BITS_PER_SAMPLE, 1);
}


/**
 * @brief Scale data to 16bit for I2S DMA output.
 *        DAC can only output 8bit data value.
 *        I2S DMA will still send 16 bit or 32bit data, the highest 8bit contains DAC data.
 */
static int beat_i2s_dac_data_scale(uint8_t* d_buff, const uint8_t* s_buff, uint32_t len)
{
    uint32_t j = 0;
    for (int i = 0; i < len; i++) {
        d_buff[j++] = 0;
        d_buff[j++] = s_buff[i];
    }
    return (len * 2);
}

static void beat_play_task(void* arg)
{
	BaseType_t ret;
	BeatMachine* bm = (BeatMachine*)arg;
	int offset, bw, btw;

	for(;;)
	{
		offset = 0;
		bw = 0;
		btw = 0;

		switch(bm->status)
		{
			/* fill the sound of the beat then dummy values to maintain the average value of the signal */
			case PLAY:
			
				btw = bm->period_bytes; /* save the value if bpm changed while BeatMachine running*/
				while (offset < btw)
				{
					if(offset < bm->sound_buf_len) /* write beat sound */
					{
						BEAT_I2S_WRITE((bm->sound_buf)+offset, (bm->sound_buf_len)-offset, &bw);
						offset += bw;
					}
					else /* fill with dummy bytes */
					{
						BEAT_I2S_WRITE(dummy_buf_127_values, btw-offset<sizeof(dummy_buf_127_values)?btw-offset:sizeof(dummy_buf_127_values), &bw);
						offset += bw;
					}
				}
				ESP_LOGV(TAG,"offset :%d bytes written.", offset);
				break;

			case PAUSE:
				/* write at leat the equivalent of the entire DMA buffers with dummy values */
				while(offset < BEAT_I2S_DMA_BUF_COUNT*BEAT_I2S_DMA_BUF_LEN)
				{
					BEAT_I2S_WRITE(dummy_buf_127_values, sizeof(dummy_buf_127_values), &bw);
					offset += bw;
				}
				ESP_LOGV(TAG,"PAUSE. offset:%d", offset);
				break;

			default:
				ESP_LOGE(TAG,"beat_play_task(): default behaviour triggerd. Stopping BeatMachine.");
				beat_stop(bm);
				break;
		}
		//vTaskDelay();
	}
	vTaskDelete(NULL); /* just in case... */
} 


/************************************************
 *  			BEAT MACHINE
 ***********************************************/

BeatMachine beat_create(void)
{
	BeatMachine bm = {
		.bpm = 0,
		.sound_buf = NULL,
		.sound_buf_len = 0,
		.period_samples = 0,
		.period_bytes = 0,
		.status = PAUSE,
		.bytes_per_sample = BEAT_I2S_BITS_PER_SAMPLE/8
	};
	return bm;
}


esp_err_t beat_init(BeatMachine* bm)
{
	/* initialize dummy values */
	memset(dummy_buf_127_values, 127, sizeof(dummy_buf_127_values));
	/* init the audio output I2S driver */
	beat_i2s_init();
	if(bm->sound_buf == NULL)
	{
		ESP_LOGE(TAG,"BeatMachine failed to initialize. No sound registered !");
		return ESP_FAIL;
	}
	if(xTaskCreatePinnedToCore(beat_play_task, "Beat playback task", 2048, bm, BEAT_PLAY_TASK_PRIORITY, &xBeat_play_task_handle, 1) != pdPASS ) return ESP_FAIL;
	return ESP_OK;
}


esp_err_t beat_start(BeatMachine* bm)
{
	esp_err_t ret = ESP_OK;
	bm->status = PLAY;
	return ret;
}


esp_err_t beat_stop(BeatMachine* bm)
{
	esp_err_t ret = ESP_OK;
	bm->status = PAUSE;
	return ret;
}


esp_err_t beat_set_bpm(BeatMachine* bm, bpm_t bpm)
{
	/* set bpm and its corresponding value in ticks and 100us multiple
	an perform some check */
	bm->bpm = bpm > BEAT_MAX_BPM ? BEAT_MAX_BPM : (bpm < BEAT_MIN_BPM ? BEAT_MIN_BPM : bpm );
	bm->period_samples = BEAT_AUDIO_SAMPLE_RATE/bm->bpm*60;
	bm->period_bytes = bm->period_samples*(uint32_t)(bm->bytes_per_sample);
	ESP_LOGI(TAG,"beat_set_bpm(): BeatMachine attributes set to \nbpm:%d\nperiod_samples:%d\nperiod_bytes:%d", bm->bpm, bm->period_samples, bm->period_bytes);
	return ESP_OK;
}


esp_err_t beat_register_sound(BeatMachine* bm, const char* filename)
{
    ESP_LOGV(TAG, "entering <%s()>", __ASSERT_FUNC);
	beat_stop(bm); /* the sound file should not be modified when BeatMachine running */
	if(filename != NULL)
	{
		// size_t btr = 0; /* bytes to read */
		// int fd;
		// struct stat filestats;
		// /* open the specified file */
		// if( (fd = open(filename, O_RDONLY)) < 0)
		// {
		// 	ESP_LOGE(TAG, "Couldn't open <%s>", filename);
		// 	perror("open()");  
		// 	return ESP_FAIL;
		// }
		// /* retrieve file size */
		// if(fstat(fd, &filestats) < 0)
		// {
		// 	ESP_LOGE(TAG, "Failed to retrieve <%s> size", filename);
		// 	perror("fstat()");  
		// 	return ESP_FAIL;
		// }
		// btr = (size_t)(filestats.st_size > BEAT_MAX_SOUND_BUF_LEN ? BEAT_MAX_SOUND_BUF_LEN : filestats.st_size);
		// /* allocate memory to store data */
		// if ((sound_buf = (uint8_t *)heap_caps_realloc(sound_buf, btr, MALLOC_CAP_DMA)) == NULL)
		// {
		// 	ESP_LOGE(TAG, "Failed to allocate %d bytes for sound_buf", btr);
		// 	perror("heap_caps_realloc()");
		// 	return ESP_FAIL;
		// }
		
		// /* read the file and fill the buffer to pass to i2s driver */
		// if ((read(fd, sound_buf, btr)) < 0)
		// {
		// 	ESP_LOGE(TAG, "Failed to read %d bytes from <%s>", btr, filename);
		// 	perror("read()");
		// 	return ESP_FAIL;
		// }

		// bm->sound_buf = sound_buf;
		// bm->sound_buf_len = btr;

		// bm->sound_buf = audio_table;
		// bm->sound_buf_len = sizeof(audio_table);
		bm->sound_buf = (uint8_t *)heap_caps_realloc(bm->sound_buf, 2*sizeof(audio_table), MALLOC_CAP_DMA);
		if(bm->sound_buf == NULL)
		{
			ESP_LOGE(TAG, "sound_buf malloc failed");
			abort();
		}
		bm->sound_buf_len = (size_t) beat_i2s_dac_data_scale(bm->sound_buf, audio_table, sizeof(audio_table)); /* get 16bits per sample to match the DMA requirements */
		ESP_LOGV(TAG, "audio_table is <%d> bytes long. sound_buf is <%d> bytes long.", sizeof(audio_table), bm->sound_buf_len);
	}
	return ESP_OK;
}