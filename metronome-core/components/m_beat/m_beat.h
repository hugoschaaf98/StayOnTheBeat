#ifndef BEAT_H
#define BEAT_H 1

#include "esp_err.h"
#include "driver/timer.h"

#include <assert.h>

/** @def
 *  @brief use or not a timer to generate the beat
 *  
 *  If it is set to 1 the delay between two beats wil be generated
 *  using a 64bits timer ISR. The timing precision is roughly 0.1ms .
 *  Otherwise vTaskDelayUntil() function will be used, the timing accuracy
 *  is about 1ms. 
 */
#define BEAT_USE_TIMER_ISR		1

#define BEAT_MAX_BPM			330		/* Max beats per minute. Modify it WITH CARE !*/
#define BEAT_MIN_BPM			50		/* Min beats per minute */
#define BEAT_MAX_SOUND_BUF_LEN	16*1024	/* Max 8kio sound file for the beat */

_Static_assert(BEAT_MIN_BPM <= BEAT_MAX_BPM, "The BEAT_MIN_BPM is upper than BEAT_MAX_BPM.");

typedef enum {
	PLAY, 		/**< BeatMachine running */
	PAUSE		/**< BeatMachine stopped */
} beatstatus_t;

typedef uint16_t	bpm_t;

/** @struct
 *  @brief      The BeatMachine structure. This should be 
 * 				created only once.
 * 
 */
struct BeatMachine
{
	bpm_t bpm; /**< Beats Per Minute at wich the beat machine plays the beat */
	uint8_t* sound_buf; /**< A pointer to the buffer which contains the beat sound data */
	size_t sound_buf_len; /**< The length in bytes of the beat sound data */
	beatstatus_t status; /**< Status of the BeatMachine. Either PLAY or PAUSE */
	uint32_t period_100us; /**< Time between two beats multiple of 100us */
	uint32_t period_ticks; /**< Time between two beats in systems ticks */
	uint64_t alarm_val; /**< Timer alarm value to trigger ISR (if timer ISR used) */
	timer_group_t timer_group; /**< Timer group identifier (if timer ISR used)*/
	timer_idx_t timer_idx; /**< Timer identifier (if timer ISR used)*/
	bool use_timer; /**< Set to true if timer ISR used, else false */
};
typedef struct BeatMachine BeatMachine;


/**
 * @brief	Create a BeatMachine.
 * 
 * @return	An instance of BeatMachine.
 */
BeatMachine beat_create(void);

/**
 * @brief      	Initialize a BeatMachine structure. Then call
 * 			  	the dedicated functions to set other parameters.
 * 			   	By default, the beat machine is paused so no need to 
 * 			   	call beat_stop() after initialization.
 * 
 * @param bm	The pointer to the BeatMachine.
 * 
 * @return     	ESP_OK if BeatMachine initialized. ESP_FAIL else.
 */
esp_err_t beat_init(BeatMachine* bm);

/**
 * @brief      	Start the BeatMachine.
 * 
 * @param bm	The pointer to the BeatMachine.
 * 
 * @return		ESP_OK if BeatMachine started. ESP_FAIL else.
 */
esp_err_t beat_start(BeatMachine* bm);

/**
 * @brief		Stop the BeatMachine.
 * 
 * @param bm	The pointer to the BeatMachine.
 * 
 * @return     	ESP_OK if BeatMachine stopped. ESP_FAIL else.
 */
esp_err_t beat_stop(BeatMachine* bm);

/**
 * @brief 		Set the beat per minute of the BeatMachine. 
 * 				Must be between M_MIN_BPM and M_MAX_BPM.
 * 
 * @param bm 	The pointer to the BeatMachine.
 * @param bpm 
 * 
 * @return		ESP_OK if bpm properly set. ESP_FAIL else.
 */
esp_err_t beat_set_bpm(BeatMachine* bm, bpm_t bpm);

/**
 * @brief 			Register a sound to be played as the beat.
 * 
 * @param bm		The pointer to the BeatMachine.
 * @param filename 	The sound file name. If null, the function does nothing.
 * 
 * @return			ESP_OK if sound was proprely registered 
 * 					or if filename is NULL.
 */
esp_err_t beat_register_sound(BeatMachine* bm, const char* filename);

/**
 * @brief				Set a timer to generate the beat period.
 * 
 * @param bm			The pointer to the BeatMachine.
 * @param timer_group 	Timer group identifier.
 * @param timer_idx 	Timer identifier.
 * @param use_timer 	If true, the specified timer will be used to generate
 * 						the delay between beats. If false, the timer will 
 * 						just be set but not used to generate the delay.
 * 
 * @return 				ESP_OK if the timer was properly registered. 
 * 						ESP_FAIL else. 
 */
esp_err_t beat_register_timer(BeatMachine* bm, timer_group_t timer_group, timer_idx_t timer_idx, bool use_timer);

#endif