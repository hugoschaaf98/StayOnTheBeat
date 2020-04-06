#include "m_beat.h"

#include <sys/fcntl.h>
#include <unistd.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "beat";

/**
 * Audio 
 * Here we use the built in DAC with i2s driver.
 * We can drive the DAC in a DMA style for more comfortability
 */
#include "driver/i2s.h"
#include "freertos/queue.h"

#define M_MAX_SAMPLE_RATE 		44100			/* Hz, for mono audio. It hugely enough for a metronome.... */
#define M_AUDIO_SAMPLE_RATE		44100


/**
 * I2S with DMA configuration 
 * @44.1kHz - 8 bits - mono, 1024 samples represent 23 ms. We have two buffers so 46 ms to 
 * read another 1024 bytes from the filesystem
 */
#define I2S_DMA_BUF_LEN			512			/* Size of a single dma buffer */
#define I2S_DMA_BUF_COUNT		2				/* Count of dma buffers */
#define CONFIG_I2S_PORT_NUM 	I2S_NUM_0		/* Use I2S0 */
#define DATA_BUF_LEN			512			/* data read from file buffer equals one i2s buffer size */


/* Utility stuffs around timer
 * 
 */
#define TIMER_FREQ(tim_base, tim_div)				(tim_base / tim_div)
#define TIMER_PERIOD_100US(tim_base, tim_div)		(tim_div*10000UL/tim_base)	/* returns the period multiple of 100US */ 				 				
#define ALARM_VAL_FROM_FREQ(alarm_f, timer_tick_f)		(timer_tick_f / alarm_f)	/* the two members must share same unit	*/ 
#define ALARM_VAL_FROM_PERIOD(alarm_p, timer_tick_p) 	(alarm_p / timer_tick_p)	/* the two members must share same unit	*/

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

/**
 * task handlers
 */
static TaskHandle_t xBeat_play_task_handle;


/*** AUDIO OUTPUT ***/

/**
 * @brief      	Init the I2S driver to directly output audio data
 * 				via the 2 built-in DACs on GPIO25-GPIO26
 * 				@CONFIG_M_SAMPLE_RATE, configured in the project
 */
static void beat_i2s_init(void)
{
	i2s_config_t i2s_config = {
		.mode = I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN,
		.sample_rate = M_AUDIO_SAMPLE_RATE,
		.bits_per_sample = 16,											/* we only want 8 bits unsigned audio data */
		.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
		.communication_format = I2S_COMM_FORMAT_I2S_MSB,
		.intr_alloc_flags = 0,											/* default interrupt priority */
		.dma_buf_count = I2S_DMA_BUF_COUNT,												
		.dma_buf_len = I2S_DMA_BUF_LEN,											
		.use_apll = false
	};

	i2s_driver_install(CONFIG_I2S_PORT_NUM, &i2s_config, 0, NULL);   /* install and start i2s driver */
	i2s_set_dac_mode(I2S_DAC_CHANNEL_BOTH_EN);
}

/**
 * @brief   	Task which is waiting for a notification
 * 				to play the beat. The notification comes
 * 				from the beat_timer_isr() @bpm frequency
 * @see        	beat_timer_isr()
 */
static void beat_play_task(void* arg)
{
	TickType_t xLastBeatTime;
	BaseType_t ret;
	int offset = 0, tot_size = 0;
	BeatMachine_t* bm = (BeatMachine_t*)arg;

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
			ESP_LOGI(TAG, "beat played !");
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


/*** BEAT TIMER ***/

#if M_USE_BEAT_TIMER_ISR

/**
 * @brief      	this ISR is used to send a task notification to the beat_play_task()
 * 				which is responsible for playing the beat at the tempo.
 * 
 * @see beat_play_back_task()
 */
static void IRAM_ATTR beat_timer_ISR(void* ptr)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	BeatMachine_t *bm = (BeatMachine_t*) ptr;

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
 * @brief      Timer initialization function. Select a timer between the 4
 * 			   available.
 *
 * @param[in]  timer_group	The timer group number
 * @param[in]  timer_idx  The timer index
 */
static esp_err_t beat_timer_init(BeatMachine_t *bm)
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


/**
 * @brief      initialization of the beat machine. Then call
 * 			   the dedicated functions to set other parameters.
 * 			   By default, the beat machine is paused so no need to 
 * 			   call beat_stop()
 * 
 * @return     ESP_OK if beat machine initialized. ESP_FAIL else.
 */
esp_err_t beat_init(BeatMachine_t* bm)
{
	bm->bpm=0;
	bm->sound_buf=NULL;
	bm->sound_buf_len=0;
	bm->status=PAUSE;
	bm->period_100us=0;
	bm->period_ticks=0;
	bm->alarm_val=0;
	bm->timer_group=TIMER_GROUP_MAX;
	bm->timer_idx=TIMER_MAX;
	bm->use_timer=false;

	/* init the audio output I2S driver */
	beat_i2s_init();
	/* create the task which is unblocked periodically at bpm to play the beat */
	if(xTaskCreate(beat_play_task, "Beat playback task", 2048, bm, BEAT_PLAY_TASK_PRIORITY, &xBeat_play_task_handle) != pdPASS ) return ESP_FAIL;
	return ESP_OK;
}

/**
 * @brief      start the beat machine
 *
 * @return     ESP_OK if beat machine started. ESP_FAIL else.
 */
esp_err_t beat_start(BeatMachine_t* bm)
{
	esp_err_t ret = ESP_OK;
	bm->status = PLAY;
	if(bm->use_timer) { ret = timer_start(bm->timer_group, bm->timer_idx); }
	vTaskResume(xBeat_play_task_handle);
	return ret;
}

/**
 * @brief      stop the beat machine
 *
 * @return     ESP_OK if beat machine stopped. ESP_FAIL else.
 */
esp_err_t beat_stop(BeatMachine_t* bm)
{
	esp_err_t ret = ESP_OK;
	bm->status = PAUSE;
	if(bm->use_timer) { ret = timer_pause(bm->timer_group, bm->timer_idx); }
	vTaskSuspend(xBeat_play_task_handle);
	return ret;
}

/**
 * @brief      set the beat per minute of the beat machine. 
 * 			   Must be between M_MIN_BPM and M_MAX_BPM
 * 
 * @return     ESP_OK if bpm properly set. ESP_FAIL else.
 */
esp_err_t beat_set_bpm(BeatMachine_t* bm, bpm_t bpm)
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

/**
 * @brief      register a sound to be played as the beat
 *
 * @param[in]  sound_buf   pointer to the audio data buffer
 * @param[in]  sound_buf_size   size of the audio data buffer in bytes  	
 *
 * @return     ESP_OK if sound was proprely registered
 * 				
 */
esp_err_t beat_register_sound(BeatMachine_t* bm, const uint8_t* sound_buf, size_t sound_buf_len)
{
    ESP_LOGV(TAG, "<%s> : entering...", __ASSERT_FUNC);
	if(sound_buf != NULL)
	{
		bm->sound_buf = sound_buf;
		bm->sound_buf_len = sound_buf_len > BEAT_MAX_SOUND_BUF_LEN ? BEAT_MAX_SOUND_BUF_LEN:sound_buf_len;
	}
	else
	{ 
		ESP_LOGW(TAG, "<sound_buf> is NULL. Ignoring");
		return ESP_FAIL;
	}
	return ESP_OK;
}

#if M_USE_BEAT_TIMER_ISR
/**
 * @brief   set a timer to generate the beat period. This function must be
 * 			called after beat_init()
 * 
 */
esp_err_t beat_register_timer(BeatMachine_t* bm, timer_group_t timer_group, timer_idx_t timer_idx, bool use_timer)
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