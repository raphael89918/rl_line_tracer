#include "dynamixel.h"

using namespace std;

int main(){

	Motor motor(2, 1, "/dev/ttyUSB1");
	
	int position = 90;
	int stateResult = motor.setServoState(ON);
	int speedResult = motor.setSpeed(30);
	int positionResult = motor.setPosition(position);

	motor.waitForIdle();
	int check = motor.checkPosition();

	return 0;
}
