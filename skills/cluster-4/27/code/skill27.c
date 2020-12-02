/*
  Skill27 code
  Paul Adan
*/

typdef enum {			// Set of states enumerated
       TITLE_SCREEN_STATE, //Title screen with text saying "Click to Start"
       START_TIMER_STATE, //starts a tasks for a game timer for 60 seconds, when the timer ends, the TIMER_END_EVENT occurs
       GENERATE_MOLES_STATE, //In this state, the game randomly generates moles
       SWING_HAMMER_STATE, //When player clicks, the hammer swings in the game
       INCREASE_SCORE_STATE, //In this state, the game increases the players score
       GAME_OVER_SCREEN_STATE, //In this state, the game displays the final score and "GAME OVER" until the player clicks
       MAX_STATES
       } state_e;

typedef enum {			// Set of events enumerated
	CLICK_EVENT, //a click from the player
	HIT_EVENT, //hammer hits a mole
  NO_HIT_EVENT, //hammer does not hit a mole
	TIMER_END_EVENT, //Game timer ends
  NEXT_TICK_EVENT, //this event occurs every tick of the game
	MAX_EVENTS
	} event_e;

state_e state = TITLE_SCREEN_STATE;	// Starting state
state_e next_state;
event_e event;
int score;

while(1)			// When event occurs, go to next state
{
	event = read_event();

	if (state == TITLE_SCREEN_STATE){
		if (event == CLICK_EVENT){
			next_state == START_TIMER_STATE;
		}
  }

  if (state == START_TIMER_STATE){ //Start timer for 60 secs then immedeatly go to next state
    if (event == NEXT_TICK_EVENT){
		  next_state == GENERATE_MOLES_STATE;
    }
  }

	else if (state == GENERATE_MOLES_STATE){
		if (event == CLICK_EVENT){
			next_state == SWING_HAMMER_STATE;
		}
    else if (event == TIMER_END_EVENT){
      next_state == GAME_OVER_SCREEN_STATE;
    }
  }

  else if (state == SWING_HAMMER_STATE){
    if (event == HIT_EVENT){
			next_state == INCREASE_SCORE_STATE;
		}
    else if (event == NO_HIT_EVENT){
      next_state == GENERATE_MOLES_STATE;
    }
  }

	else if (state == INCREASE_SCORE_STATE){
    if (event == NEXT_TICK_EVENT){
		  next_state == GENERATE_MOLES_STATE;
    }
	}

  else if (state == GAME_OVER_SCREEN_STATE){ //display score and "GAMEOVER" until player clicks
    if (event == CLICK_EVENT){
      next_state == TITLE_SCREEN_STATE;
    }
  }

	state = next_state;
}
