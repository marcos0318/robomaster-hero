#include "helper_functions.h"
#include "judge.h"


int32_t convertToShootingSpeed(float v) {
	counter++;
	int base = 990;
  if (v >= 26.3) {
    return base;
  } 
  else if ( v > 26 ) {
    return (int)(-10 * (v - 26) + base + 3); 
  }
  else if ( v > 25.5 ) {
    return (int)(-2 * (v - 25.5) + base + 4);
  }
  else if ( v > 25 ) {
    return (int)(-4 * (v - 25) + base + 6);
  }
  else if ( v > 24.5 ) {
    return (int)(-2 * (v - 24.5) + base + 7);
  }
  else if ( v > 24 ) {
    return (int)(-40 * (v - 24) + base + 27 );
  }
  else if ( v > 20 ) {
    return (int)(-61.81 * (v - 21.8) + base + 163);
  }
  else return base;//for when the judging system return 0 V.. Normally wont happen

}

int32_t fixShootingSpeed ( int32_t speed, int32_t timeSinceLastShoot, int32_t increase, int32_t period) {
	if (timeSinceLastShoot > period) {
		return speed;
	}
	else {
		return (speed + increase * (period - timeSinceLastShoot) / period); 
	}
	
}


//The buffer functions 
int32_t buffer_out(int32_t* b, int32_t length, int32_t counter){
	int32_t b_output = 0;
	for (int i = 0 ; i< length ; i++){
		b_output+= b[i];
	}
	b_output = b_output/ length;
	return b_output;
}

void buffer_in(int32_t* b , int32_t length, int32_t counter, int32_t input ){
	int16_t index = counter%length ;
	b[index] = input;
}

//The adjustment of the set points
int32_t abs(int32_t x){
	if (x < 0){
			return -x;
	}
	return x;
}

int32_t max(int32_t a, int32_t b){

	if(a > b)
		return a;
	else
		return b;

}

int32_t min(int32_t a, int32_t b){

	if(a < b)
		return a;
	else
		return b;

}

void wheel_setpoints_adjust(int32_t * sp1, int32_t* sp2, int32_t* sp3, int32_t* sp4, int32_t limit){
	int32_t max = abs(*sp1);
	if (abs(*sp2) > max) max = abs(*sp2);
	if (abs(*sp3) > max) max = abs(*sp3);
	if (abs(*sp4) > max) max = abs(*sp4);

	if (max > limit){ 
			*sp1 = *sp1 * limit  / max;
			*sp2 = *sp2 * limit  / max;
			*sp3 = *sp3 * limit  / max;
			*sp4 = *sp4 * limit  / max;			
	}
}	

void windowLimit(int32_t* dst, int32_t upperLimit, int32_t lowerLimit) {

	if(*dst > upperLimit)
		*dst = upperLimit;
	else if(*dst < lowerLimit)
		*dst = lowerLimit;
}

void fwindowLimit(float* dst, int32_t upperLimit, int32_t lowerLimit) {

	if(*dst > upperLimit)
		*dst = upperLimit;
	else if(*dst < lowerLimit)
		*dst = lowerLimit;
}

uint8_t checkBrokenLine(uint32_t ticks, uint32_t counter)
{
	//if((ticks-counter)<80 && (ticks-counter)>-80) return 0;
	if((ticks-counter)<=80) return 0;
	else return 1;
}
