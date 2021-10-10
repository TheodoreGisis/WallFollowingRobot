#include <NewPing.h>
#define MAX_DISTANCE 300
#define MAX_DISTANCE_FRONT 200
#define TRIGGER_PIN 2
#define ECHO_PIN 3
#define ECHO_PIN_FRONT 4
#define TRIGGER_PIN_FRONT  5
#define setpoint 19
#define setpoint_front 50

NewPing Left(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
NewPing Front(TRIGGER_PIN_FRONT, ECHO_PIN_FRONT, MAX_DISTANCE_FRONT);


double distanceLeft, distance, error, integral, lasterror, derivative, distanceFront, distance1;
double error_front, derivative_front, last_error_front, integral_front;

float kp = 0.6;
float ki = 0.003;
float kd = 2.5;

float Kp_left = 0.2;
float Ki_left = 0;
float Kd_left = 0.3;
int totalLeft, totalRight;

int pid_value;

int rightspeed = 80;
int leftspeed = 95;

//left motor
const int a_pwm_pin = 9;
const int a_dir1_pin = 8;
const int a_dir2_pin = 7;

//right motor
const int b_pwm_pin = 10;
const int b_dir1_pin = 11;
const int b_dir2_pin = 12;


void setup() {
  pinMode(a_pwm_pin, OUTPUT);
  pinMode(a_dir1_pin, OUTPUT);
  pinMode(a_dir2_pin, OUTPUT);
  pinMode(b_pwm_pin, OUTPUT);
  pinMode(b_dir1_pin, OUTPUT);
  pinMode(b_dir2_pin, OUTPUT);
  Serial.begin(9600);
}

void loop() {

  distanceLeft = read_distance_left();
  distanceFront = read_distance_front();


  if (distanceFront > 45 || distanceFront == 0) 
{
    error = distanceLeft - setpoint;
    derivative = (error - lasterror);
    
    if (abs(error) < 10 && error != 0) { 
      
      integral = error + integral;
      
    }
    else {
      
      integral = 0;
      
    }

    if (error == 0) {
      
      derivative = 0;
      
    }
    if (error > 30) {
      
      error = 30;
    }

    pid_value = (kp * error + kd * derivative + ki * integral);

    if (error < 12)
    {
      if (pid_value > 80 && pid_value > 0) {
        pid_value = 80;
      }
      if (pid_value < -80 && pid_value < 0) {
        pid_value = -80;
      }

      totalLeft = leftspeed - (pid_value);
      totalRight = rightspeed + pid_value;
      forward(totalLeft, totalRight);
    }
    else if(error>12){
      int lefturn = Kp_left * error +  Kd_left *derivative + Ki_left * integral;
        if(lefturn >80 && lefturn >0){
          lefturn=80;
        }
        if(lefturn <-80 && lefturn <0){
          lefturn =-80;
        }
      totalLeft= 80-( lefturn);
      totalRight = 80  +(lefturn);  
      forward(totalLeft,totalRight);
    }
    
}
  else if (distanceFront < 45 && distanceFront != 0) {

    right_turn();
    delay(380);

   }

lasterror = error;
}


//-------FUNCTIONS---------------
void right_turn() {
  digitalWrite(a_dir1_pin, HIGH);
  digitalWrite(a_dir2_pin, LOW);
  analogWrite(a_pwm_pin, 85);

  digitalWrite(b_dir1_pin, HIGH);
  digitalWrite(b_dir2_pin, LOW);
  analogWrite(b_pwm_pin, 0);

}


void forward(int speed1, int speed2) {

  // Check to make sure speed is 0-255
  if ( speed1 < 0 ) {
    
    speed1 = 0;
    
  }
  if ( speed1 > 255 ) {
    
    speed1 = 255;
    
  }
  if (speed2 < 0) {
    
    speed2 = 0;
    
  }
  if (speed2 > 255) {
    
    speed2 = 255;
    
  }
  
  digitalWrite(a_dir1_pin, HIGH);
  digitalWrite(a_dir2_pin, LOW);
  analogWrite(a_pwm_pin, speed1);

  digitalWrite(b_dir1_pin, HIGH);
  digitalWrite(b_dir2_pin, LOW);
  analogWrite(b_pwm_pin, speed2);
}




void Stop() {
  
  digitalWrite(a_dir1_pin, HIGH);
  digitalWrite(a_dir2_pin, LOW);
  analogWrite(a_pwm_pin, 0);

  digitalWrite(b_dir1_pin, HIGH);
  digitalWrite(b_dir2_pin, LOW);
  analogWrite(b_pwm_pin, 0);

}




//-------DISTANCES---------------------

float read_distance_left() {
  int iterations = 5;
  float duration;
  duration = Left.ping_median(iterations);
  distance = (duration / 2) * 0.0343;
  return distance;
}




int read_distance_front() {
  int iterations1 = 1;
  float duration1;
  duration1 = Front.ping_median(iterations1);
  distance1 = (duration1 / 2) * 0.0343;
  return distance1;
}
