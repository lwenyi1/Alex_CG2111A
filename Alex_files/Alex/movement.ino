/**
 * robotlib.ino
 * 
 * contains the functions to move the robot
 */

#include <AFMotor.h>


// Motor control
#define FRONT_LEFT 4   // M4 on the driver shield
#define FRONT_RIGHT 1  // M1 on the driver shield
#define BACK_LEFT 3    // M3 on the driver shield
#define BACK_RIGHT 2   // M2 on the driver shield

AF_DCMotor motorFL(FRONT_LEFT);
AF_DCMotor motorFR(FRONT_RIGHT);
AF_DCMotor motorBL(BACK_LEFT);
AF_DCMotor motorBR(BACK_RIGHT);

void move(float speed, int direction) {
  int speed_scaled = (speed / 100.0) * 255;
  motorFL.setSpeed(speed_scaled);
  motorFR.setSpeed(speed_scaled);
  motorBL.setSpeed(speed_scaled);
  motorBR.setSpeed(speed_scaled);

  switch (direction) {
    case BACK:
      motorFL.run(BACKWARD);
      motorFR.run(BACKWARD);
      motorBL.run(FORWARD);
      motorBR.run(FORWARD);
      break;
    case GO:
      motorFL.run(FORWARD);
      motorFR.run(FORWARD);
      motorBL.run(BACKWARD);
      motorBR.run(BACKWARD);
      break;
    case CW:
      motorFL.run(FORWARD);
      motorFR.run(BACKWARD);
      motorBL.run(BACKWARD);
      motorBR.run(FORWARD);
      break;
    case CCW:
      motorFL.run(BACKWARD);
      motorFR.run(FORWARD);
      motorBL.run(FORWARD);
      motorBR.run(BACKWARD);
      break;
    case STOP:
    default:
      motorFL.run(STOP);
      motorFR.run(STOP);
      motorBL.run(STOP);
      motorBR.run(STOP);
  }
}

void forward(float dist, float speed) {
  if (!mode) {
    if (dist > 0)
      deltaDist = dist;
    else
      deltaDist = 9999999;

    newDist = forwardDist + deltaDist;
    dir = (TDirection)FORWARD;
  }
  move(speed, FORWARD);
}

void backward(float dist, float speed) {
  if (!mode) {
    if (dist > 0)
      deltaDist = dist;
    else
      deltaDist = 9999999;

    newDist = reverseDist + deltaDist;
    dir = (TDirection)BACKWARD;
  }
  move(speed, BACKWARD);
}

void left(float ang, float speed) {
  if (!mode) {
    if (ang == 0)
      deltaTicks = 99999999;
    else
      deltaTicks = computeDeltaTicks(ang);

    targetTicks = leftReverseTicksTurns + deltaTicks;
    dir = (TDirection)LEFT;
  }
  move(speed, LEFT);
}

void right(float ang, float speed) {
  if (!mode) {
    if (ang == 0)
      deltaTicks = 99999999;
    else
      deltaTicks = computeDeltaTicks(ang);

    targetTicks = rightReverseTicksTurns + deltaTicks;
    dir = (TDirection)RIGHT;
  }
  move(speed, RIGHT);
}

void stop() {
  if (!mode) {
    dir = (TDirection)STOP;
  }
  move(0, STOP);
}

//This stops the robot based on the original controls, i.e. the robot
//will move the commanded distance as per what the operator sent over
//then stop upon reaching.
void original_movement() {
  if (deltaDist > 0) {
    if (dir == FORWARD) {
      if (forwardDist > newDist) {
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    } else if (dir == BACKWARD) {
      if (reverseDist > newDist) {
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    } else if (dir == (TDirection)STOP) {
      deltaDist = 0;
      newDist = 0;
      stop();
    }
  }

  if (deltaTicks > 0) {
    if (dir == LEFT) {
      if (leftReverseTicksTurns >= targetTicks) {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
    } else if (dir == RIGHT) {
      if (rightReverseTicksTurns >= targetTicks) {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
    } else if (dir == (TDirection)STOP) {
      deltaTicks = 0;
      targetTicks = 0;
      stop();
    }
  }
}