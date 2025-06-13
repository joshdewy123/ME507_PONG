/*
 * fsm.h
 *
 *  Created on: Jun 9, 2025
 *      Author: NJ
 */

// fsm.h
#ifndef INC_FSM_H_
#define INC_FSM_H_

typedef enum {
    STATE_0,
    STATE_1,
    STATE_2,
    // Add more states
} State_t;

void FSM_Init(void);
void FSM_Run(void);
void FSM_SetState(State_t new_state);

State_t FSM_GetState(void);

#endif /* INC_FSM_H_ */
