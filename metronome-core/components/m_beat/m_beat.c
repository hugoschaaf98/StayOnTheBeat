#include "m_beat.h"

#include <stdio.h>
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

#define BEAT_MAX_SAMPLE_RATE 		44100			/* Hz, for mono audio. It hugely enough for a metronome.... */
#define BEAT_AUDIO_SAMPLE_RATE		16000


/**
 * I2S with DMA configuration 
 * audio data can be up to @44.1kHz - 8 bits - stereo.
 */
#define BEAT_I2S_PORT_NUM			I2S_NUM_0		/* don't change it if use built in DAC */
#define	BEAT_I2S_BITS_PER_SAMPLE	I2S_BITS_PER_SAMPLE_16BIT
#define BEAT_I2S_DMA_BUF_LEN		1024			/* Size of a single dma buffer */
#define BEAT_I2S_DMA_BUF_COUNT		6				/* Count of dma buffers */


/* Utility stuffs around timer
 * 
 */
#define TIMER_FREQ(tim_base, tim_div)				(tim_base / tim_div)
#define TIMER_PERIOD_100US(tim_base, tim_div)		(tim_div*10000UL/tim_base)	/* returns the period multiple of 100US */ 				 				
#define ALARM_VAL_FROM_FREQ(alarm_f, timer_f)		(timer_f / alarm_f)	/* the two members must share same unit	*/ 
#define ALARM_VAL_FROM_PERIOD(alarm_p, timer_p) 	(alarm_p / timer_p)	/* the two members must share same unit	*/

/**
 * Beat Timer
 * The beat timer is to generate a periodic task notification
 * @bpm to playback the beat sound generated via i2s 
 */
#define BEAT_TASK_PRIORITY 				10		/* arbitrary but high enough to be the highest priority task after
                           			  			 * beat_play_task() */
#define BEAT_PLAY_TASK_PRIORITY			BEAT_TASK_PRIORITY+1	
#define BEAT_PLAY_TASK_MAX_BLOCK_TICKS	pdMS_TO_TICKS(5000)		/* 5000ms max blocking before considering the beat machine is paused */
#define BEAT_TIMER_GROUP				TIMER_GROUP_0
#define BEAT_TIMER_IDX 					TIMER_1
#define BEAT_TIMER_DIVIDER				8000 	/* since TIMER_BASE_CLK = APB_CLK = 80MHz, Timer @10kHz - 0.1ms period */
#define BEAT_TIMER_PERIOD_100US 	TIMER_PERIOD_100US(TIMER_BASE_CLK, BEAT_TIMER_DIVIDER)

/* Generic
 * 
 */
#define BPM_TO_PERIOD_MS(bpm)	(uint32_t)(60000/bpm) 	/* adding 0.5 acts as a round function */
#define BPM_TO_PERIOD_100US(bpm)	(uint32_t)(600000/bpm)

/* the play beat task handler */
static TaskHandle_t xBeat_play_task_handle;

/* Buffer to store the beat sound data*/
DRAM_ATTR static uint8_t* sound_buf = NULL;

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
		.bits_per_sample = BEAT_I2S_BITS_PER_SAMPLE,		/* we only want 8 bits unsigned audio data */
		.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,		/* RIGHT -> DAC1 ie GPIO25; LEFT -> DAC2 ie GPIO26*/
		.communication_format = I2S_COMM_FORMAT_PCM,
		.intr_alloc_flags = 0,								/* default interrupt priority */
		.dma_buf_count = BEAT_I2S_DMA_BUF_COUNT,			/* should be between 2 and 128 */ 									
		.dma_buf_len = BEAT_I2S_DMA_BUF_LEN,				/* should be between 8 and 1024 */							
		.use_apll = true
	};

	i2s_driver_install(BEAT_I2S_PORT_NUM, &i2s_config, 0, NULL);   /* install and start i2s driver */
	i2s_set_dac_mode(I2S_DAC_CHANNEL_BOTH_EN);
	i2s_set_clk(BEAT_I2S_PORT_NUM, BEAT_AUDIO_SAMPLE_RATE, BEAT_I2S_BITS_PER_SAMPLE, 1);
}


/**
 * @brief Scale data to 16bit/32bit for I2S DMA output.
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

/**
 * Task which is waiting for a notification
 * to play the beat. The notification comes
 * from the beat_timer_isr() @bpm frequency
 */
static void beat_play_task(void* arg)
{
	TickType_t xLastBeatTime;
	BaseType_t ret;
	BeatMachine* bm = (BeatMachine*)arg;

	/* int offset = 0, tot_size = 0; */
	size_t bw = 0; /* bytes written to I2S */


	vTaskSuspend(NULL); /* wait for the first beat_start to be called */

	xLastBeatTime = xTaskGetTickCount();

	for(;;)
	{
		/* waiting for the next beat, stop the i2s driver and preload data */
		/*i2s_stop(CONFIG_I2S_PORT_NUM);
		  i2s_write(CONFIG_I2S_PORT_NUM, i2s_write_buf, i2s_wr_len, &bytes_written, portMAX_DELAY);
		*/

		ret = pdTRUE;

		/* use the direct notification system to block the task until next beat */
		if(bm->use_timer) /* wait the start signal from beat_timer_isr() to play the beat or timeout*/
		{ ret = xTaskNotifyWait(0, 0, NULL, BEAT_PLAY_TASK_MAX_BLOCK_TICKS); }	/* if no timeout, the metronome is running. Play the Beat */
		else
		{ vTaskDelayUntil(&xLastBeatTime, bm->period_ticks); }

		if(ret == pdTRUE)
		{
			// int offset = 0;
        	// int tot_size = sizeof(audio_table);
        	// while (offset < tot_size) {
            // 	int play_len = ((tot_size - offset) > (4 * 1024)) ? (4 * 1024) : (tot_size - offset);
            // 	int i2s_wr_len = example_i2s_dac_data_scale(i2s_write_buff, (uint8_t*)(audio_table + offset), play_len);
            // 	i2s_write(EXAMPLE_I2S_NUM, bm->sound_buf, bm->sound_buf_len, &bytes_written, portMAX_DELAY);
            // 	offset += play_len;
        	// }
			

			i2s_write(BEAT_I2S_PORT_NUM, bm->sound_buf, bm->sound_buf_len, &bw, portMAX_DELAY);
			ESP_LOGI(TAG, "beat played ! <%d> of <%d> bytes written to DMA", bw, bm->sound_buf_len );
			/*
			offset = 0; tot_size = 0;
			i2s_start(CONFIG_I2S_PORT_NUM);

			while (offset < tot_size) 
			{
				 	FAUT FINIR ICI !
					/!\   /!\   /!\
				
				i2s_write(CONFIG_I2S_PORT_NUM, i2s_write_buf, i2s_wr_len, &bytes_written, portMAX_DELAY);
				
			}
			*/
		}
		else /* if timeout */
		{
			ESP_LOGV(TAG, "metronome paused...");
			vTaskSuspend(NULL);
		}	
	}

	vTaskDelete(NULL); /* just in case of... */
} 


/************************************************
 *  			BEAT TIMER
 ***********************************************/

#if BEAT_USE_TIMER_ISR

/**
 * this ISR is used to send a task notification to the beat_play_task()
 * which is responsible for playing the beat at the tempo.
 * 
 */
static void IRAM_ATTR beat_timer_ISR(void* ptr)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	BeatMachine* bm = (BeatMachine*) ptr;

	timer_group_clr_intr_status_in_isr(bm->timer_group, bm->timer_idx);
	timer_group_enable_alarm_in_isr(bm->timer_group, bm->timer_idx);

	vTaskNotifyGiveFromISR(xBeat_play_task_handle, &xHigherPriorityTaskWoken);
	/* Force a context switch if xHigherPriorityTaskWoken is now set to pdTRUE.
    The macro used to do this is dependent on the port and may be called
    portEND_SWITCHING_ISR. */
	if(xHigherPriorityTaskWoken)
    { 
		portYIELD_FROM_ISR();
	}
}

/**
 * Timer initialization function. Select a timer between the 4
 * available.
 *
 * timer_group	The timer group number
 * timer_idx  The timer index
 */
static esp_err_t beat_timer_init(BeatMachine *bm)
{
	timer_group_t tgp = bm->timer_group;
	timer_idx_t tid = bm->timer_idx;

	esp_err_t ret;
	timer_config_t config = {
		.divider = BEAT_TIMER_DIVIDER,
		.counter_dir = TIMER_COUNT_UP,
		.counter_en = TIMER_PAUSE,
		.alarm_en = TIMER_ALARM_EN,
		.intr_type = TIMER_INTR_LEVEL,
		.auto_reload = TIMER_AUTORELOAD_EN,
	};

	ret = timer_init(tgp, tid, &config);
	ESP_ERROR_CHECK(ret);
	ret = timer_set_counter_value(tgp, tid, 0x0000000000000000ULL);
	ESP_ERROR_CHECK(ret);
	ret = timer_set_alarm_value(tgp, tid, 0xFFFFFFFFFFFFFFFFULL);
	ESP_ERROR_CHECK(ret);
	ret = timer_enable_intr(tgp, tid);
	ESP_ERROR_CHECK(ret);
	/* Register an ISR handler */
	timer_isr_register(tgp, tid, beat_timer_ISR, bm, ESP_INTR_FLAG_IRAM, NULL);
	
	return ret;
}

#endif /* end #ifdef M_USE_BEAT_TIMER_ISR */


/************************************************
 *  			BEAT MACHINE
 ***********************************************/

BeatMachine beat_create(void)
{
	BeatMachine bm = {
		.bpm = 0,
		.sound_buf = NULL,
		.sound_buf_len = 0,
		.status = PAUSE,
		.period_100us = 0,
		.period_ticks = 0,
		.alarm_val = 0,
		.timer_group = TIMER_GROUP_MAX,
		.timer_idx = TIMER_MAX,
		.use_timer = false
	};
	return bm;
}


esp_err_t beat_init(BeatMachine* bm)
{
	/* init the audio output I2S driver */
	beat_i2s_init();
	/* create the task which is unblocked periodically at bpm to play the beat */
	if(xTaskCreatePinnedToCore(beat_play_task, "Beat playback task", 2048, bm, BEAT_PLAY_TASK_PRIORITY, &xBeat_play_task_handle, 1) != pdPASS ) return ESP_FAIL;
	return ESP_OK;
}


esp_err_t beat_start(BeatMachine* bm)
{
	esp_err_t ret = ESP_OK;
	bm->status = PLAY;
	i2s_start(BEAT_I2S_PORT_NUM);
	if(bm->use_timer) { ret = timer_start(bm->timer_group, bm->timer_idx); }
	vTaskResume(xBeat_play_task_handle);
	return ret;
}


esp_err_t beat_stop(BeatMachine* bm)
{
	esp_err_t ret = ESP_OK;
	bm->status = PAUSE;
	
	/* stop i2s and don't forget to zero dma buffer
	if the stop happend during a beat */
	i2s_stop(BEAT_I2S_PORT_NUM);
	i2s_zero_dma_buffer(BEAT_I2S_PORT_NUM);

	if(bm->use_timer) { ret = timer_pause(bm->timer_group, bm->timer_idx); }
	vTaskSuspend(xBeat_play_task_handle);
	return ret;
}


esp_err_t beat_set_bpm(BeatMachine* bm, bpm_t bpm)
{
	/* set bpm and its corresponding value in ticks and 100us multiple
	an perform some check */
	bm->bpm = bpm > BEAT_MAX_BPM ? BEAT_MAX_BPM : (bpm < BEAT_MIN_BPM ? BEAT_MIN_BPM : bpm );
	bm->period_100us = BPM_TO_PERIOD_100US(bm->bpm);
	bm->period_ticks = pdMS_TO_TICKS(BPM_TO_PERIOD_MS(bm->bpm));

	ESP_LOGV(TAG, "<%s> bpm:%u period_100us: %u period_ticks: %u ", __ASSERT_FUNC, bm->bpm, bm->period_100us, bm->period_ticks);

	if(bm->use_timer)
	{	
		bm->alarm_val = ALARM_VAL_FROM_PERIOD(bm->period_100us , BEAT_TIMER_PERIOD_100US);
		ESP_LOGV(TAG, "<%s> timer alarm value: %llu", __ASSERT_FUNC, bm->alarm_val);
		return timer_set_alarm_value(bm->timer_group, bm->timer_idx, bm->alarm_val); 
	}

	return ESP_OK;
}


esp_err_t beat_register_sound(BeatMachine* bm, const char* filename)
{
    ESP_LOGV(TAG, "entering <%s()>", __ASSERT_FUNC);
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
		bm->sound_buf_len = (size_t) beat_i2s_dac_data_scale(bm->sound_buf, audio_table, sizeof(audio_table));
		ESP_LOGV(TAG, "audio_table is <%d> bytes long. sound_buf is <%d> bytes long.", sizeof(audio_table), bm->sound_buf_len);
	}
	return ESP_OK;
}


#if BEAT_USE_TIMER_ISR

esp_err_t beat_register_timer(BeatMachine* bm, timer_group_t timer_group, timer_idx_t timer_idx, bool use_timer)
{
	bm->timer_group = timer_group;
	bm->timer_idx = timer_idx;
	bm->use_timer = use_timer;
	if(bm->use_timer)
	{
		return beat_timer_init(bm);
	}
	return ESP_OK;
}

#endif