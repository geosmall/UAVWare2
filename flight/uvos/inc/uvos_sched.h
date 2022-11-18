#ifndef UVOS_SCHED_H
#define UVOS_SCHED_H

// ------ Public data type declarations ----------------------------

// User-defined type to store required data for each task
struct uvos_sched_task {
	void ( *pTask ) ( void );  	/* Pointer to the task (must be a 'void (void)' function) */
	uint32_t Delay;  					 	/* Delay (ticks) until the task will (next) be run */
	uint32_t Period;  					/* Interval (ticks) between subsequent runs */
};

// ------ Public function prototypes -----------------------------------------

extern void UVOS_SCHED_init_hz( const uint32_t TICKhz );
extern void UVOS_SCHED_start( uint32_t * tim_id );
extern int32_t UVOS_SCHED_dispatch_tasks( void );
extern void UVOS_SCHED_tick_handler( uint32_t tim_id, uint32_t context, uint8_t chan_idx, uint16_t count );
extern int32_t UVOS_SCHED_add_task( void ( * pTask )(), const uint32_t DELAY, const uint32_t PERIOD );

// ------ Public constants -------------------------------------------------------------

// The maximum number of tasks required at any one time
// during the execution of the program
//
// MUST BE CHECKED FOR EACH PROJECT (*not* dynamic)
#define SCH_MAX_TASKS ( 5 )

// Usually set to 1, unless 'Long Tasks' are employed
#define SCH_TICK_COUNT_LIMIT ( 20 )

// Default value for pTask (no task at this location)
#define SCH_NULL_PTR ( ( void (*) ( void ) ) 0 )

#endif /* UVOS_SCHED_H */