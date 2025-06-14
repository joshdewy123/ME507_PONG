/**
 * @file fsm.h
 * @brief Finite State Machine (FSM) header for autonomous launcher behavior
 *
 * This module defines the interface for an FSM that coordinates different
 * states of the ping pong launcher system. These may include manual control,
 * auto-targeting, loading, launching, and safety handling.
 *
 * The FSM is run in a loop from `main.c`, and each state should define its
 * own entry actions and transitions.
 */
#ifndef INC_FSM_H_
#define INC_FSM_H_

/**
 * @brief Enumeration of system states
 *
 * Update the enum to reflect the actual robot states used in the project.
 */
typedef enum {
    STATE_0,
    STATE_1,
    STATE_2,
    // Add more states
} State_t;

/**
 * @brief Initializes FSM-related state and subsystems
 *
 * Call this during system startup to set initial state and configure variables.
 */
void FSM_Init(void);

/**
 * @brief Runs one cycle of the FSM
 *
 * Called repeatedly in `main.c` to progress through states based on conditions.
 */
void FSM_Run(void);


/**
 * @brief Set the FSM to a specific state
 * @param new_state Desired state to transition into
 */
void FSM_SetState(State_t new_state);


/**
 * @brief Get the current state of the FSM
 * @return Current FSM state
 */
State_t FSM_GetState(void);

#endif /* INC_FSM_H_ */
