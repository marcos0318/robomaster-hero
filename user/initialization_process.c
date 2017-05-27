#include "initialization_process.h"





void Limit_Switch_init(){
	
	BUTTON_init(Left_front_LM);
	BUTTON_init(Left_back_LM);
	BUTTON_init(Right_front_LM);
	BUTTON_init(Right_back_LM);
	
}

void FL_init(){
	
}

void FR_init(){

}

void BL_init(){

}

void BR_init(){

}



void initialization_process_full_init(){
	FL_init();
	FR_init();
	BL_init();
	BR_init();



}



void initialization_process_front_init(){
	FL_init();
	FR_init();
}

void initialization_process_back_init(){
	BL_init();
	BR_init();

}



