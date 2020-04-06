#ifndef BEAT_H
#define BEAT_H 1

#include "sdkconfig.h"
#include "esp_err.h"
#include "driver/timer.h"


#include <assert.h>


/** @def
 *  @brief use or not a timer to generate the beat
 *  
 *  If it is set to 1 the delay between two beats wil be generated
 *  using a 64bits timer ISR. The timing precision is roughly 100us.
 *  Otherwise vTaskDelayUntil() function will be used, the timing accuracy
 *  is about 1ms. 
 */
#define M_USE_BEAT_TIMER_ISR	1
#define BEAT_MAX_BPM			350		/* beats per minute */
#define BEAT_MIN_BPM			50		/* beats per minute */
#define BEAT_MAX_SOUND_BUF_LEN	8192	/* max 8kio	*/

_Static_assert(BEAT_MIN_BPM <= BEAT_MAX_BPM, "The BEAT_MIN_BPM is upper than BEAT_MAX_BPM.");

typedef enum {PLAY, PAUSE} beatstatus_t;
typedef uint16_t	bpm_t;

/** @struct
 *  @brief      the metronome must be unique.
 * 
 */
struct BeatMachine_t
{
	bpm_t bpm;
	uint8_t* sound_buf;
	size_t sound_buf_len;
	beatstatus_t status;
	uint32_t period_100us;
	uint32_t period_ticks;
	uint64_t alarm_val;
	timer_group_t timer_group;
	timer_idx_t timer_idx;
	bool use_timer;
};
typedef struct BeatMachine_t BeatMachine_t;


/**
 * @brief      initialization of the beat machine. Then call
 * 			   the dedicated functions to set other parameters.
 * 			   By default, the beat machine is paused so no need to 
 * 			   call beat_stop()
 * 
 * @return     ESP_OK if beat machine initialized. ESP_FAIL else.
 */
esp_err_t beat_init(BeatMachine_t* bm);

/**
 * @brief      start the beat machine
 *
 * @return     ESP_OK if beat machine started. ESP_FAIL else.
 */
esp_err_t beat_start(BeatMachine_t* bm);

/**
 * @brief      stop the beat machine
 *
 * @return     ESP_OK if beat machine stopped. ESP_FAIL else.
 */
esp_err_t beat_stop(BeatMachine_t* bm);

/**
 * @brief      set the beat per minute of the beat machine. 
 * 			   Must be between M_MIN_BPM and M_MAX_BPM
 * 
 * @return     ESP_OK if bpm properly set. ESP_FAIL else.
 */
esp_err_t beat_set_bpm(BeatMachine_t* bm, bpm_t bpm);

/**
 * @brief      register a sound to be played as the beat
 *
 * @param[in]  sound_buf   pointer to the audio data buffer
 * @param[in]  sound_buf_size   size of the audio data buffer in bytes  	
 *
 * @return     ESP_OK if sound was proprely registered
 * 				
 */
esp_err_t beat_register_sound(BeatMachine_t* bm, const uint8_t* sound_buf, size_t sound_buf_len);

/**
 * @brief set a timer to generate the beat period 
 * 
 */
esp_err_t beat_register_timer(BeatMachine_t* bm, timer_group_t timer_group, timer_idx_t timer_idx, bool use_timer);

#endif