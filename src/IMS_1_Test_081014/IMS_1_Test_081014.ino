
//Left motor constants
const int leftMotorEn = 4;
const int leftMotorLpwm = 5;
const int leftMotorRpwm = 6;

//Right motor constants
const int rightMotorEn = 8;
const int rightMotorLpwm = 9;
const int rightMotorRpwm = 10;

int rmSpeed;
int lmSpeed;
int aRead;
int sendSpeed;

void setup() {                
  //Serial.begin(9600);
  //set pinmode for left motor
  pinMode(leftMotorLpwm, OUTPUT); 
  pinMode(leftMotorRpwm, OUTPUT);  
  pinMode(leftMotorEn, OUTPUT);
  //set pinmode for right motor
  pinMode(rightMotorLpwm, OUTPUT); 
  pinMode(rightMotorRpwm, OUTPUT);  
  pinMode(rightMotorEn, OUTPUT);

}

void loop() {

//read input from A1 and generate left motor speed value
aRead = analogRead(A1);
aRead = constrain(aRead, 0, 1023);
lmSpeed = map(aRead,0,1023, 0, 255);
lmSpeed = constrain(lmSpeed, 0, 255);


//read input from A0 and generate right motor speed value
aRead = analogRead(A0);
aRead = constrain(aRead, 0, 1023);
rmSpeed = map(aRead,0,1023, 0, 255);
rmSpeed = constrain(rmSpeed, 0, 255);

   
// ***** Left Motor Logic *****
    if (lmSpeed > 110 && lmSpeed < 140) {  // DO NOTHING
      drive(0, 255, 0);  // drive(int motorSel, int pwm, int reverse)
    }
    if (lmSpeed > 140){ // MOVE FORWARD
        sendSpeed = map(lmSpeed, 140, 255, 255, 0);
        sendSpeed = constrain(sendSpeed, 0, 255);
        drive(0, sendSpeed, 0); //left motor, motor speed, forward
    } 
    if (lmSpeed < 110){ // MOVE IN REVERSE
      sendSpeed = map(lmSpeed, 0, 110, 255, 0);
      sendSpeed = constrain(sendSpeed, 0, 255);
      sendSpeed = 255 - sendSpeed;
      drive(0, sendSpeed,1);
    } 
    
// ***** Right Motor Logic *****
    if (rmSpeed > 110 && rmSpeed < 140) {  // DO NOTHING
      drive(1, 255, 0);  // drive(int motorSel, int pwm, int reverse)
    }

    if (rmSpeed > 140){   // MOVE FORWARD
        sendSpeed = map(rmSpeed, 140, 255, 255, 0);
        sendSpeed = constrain(sendSpeed, 0, 255);
        drive(1, sendSpeed, 0); //right motor, motor speed, forward
    }     
    if (rmSpeed < 110){  // MOVE IN REVERSE
      sendSpeed = map(rmSpeed, 0, 110, 255, 0);
      sendSpeed = constrain(sendSpeed, 0, 255);
      sendSpeed = 255 - sendSpeed;
      drive(1, sendSpeed,1);
    }    
    delay(100);       //for 100ms
}


void drive(int motorSel, int pwm, int reverse){
  //motorSel - pass 0 for left motor, 1 for right
  //pwm - the speed (0 max, 255 stop)
  //dir - direction of motor (0 forward, 1 reverse)
  
  int lpwmPin;  //used for pin number storage
  int rpwmPin;  //used for pin number storage

   if(motorSel==0) {  // LEFT MOTOR SELECTED
    lpwmPin = leftMotorLpwm;  // MAP lpwmPin to LEFT Motor
    rpwmPin = leftMotorRpwm;  // MAP rpwmPin to LEFT Motor
    digitalWrite(leftMotorEn, HIGH);  //Enable Left Motor
  }

  if(motorSel==1){  //if right motor is selected
    lpwmPin = rightMotorLpwm;  //MAP LPWMPIN TO RIGHT MOTOR
    rpwmPin = rightMotorRpwm;  //MAP RPWMPIN TO RIGHT MOTOR
    digitalWrite(rightMotorEn, HIGH);  //Enable Right Motor
  } 

  if (reverse == 1){  //reverse = 1 for reverse, reverse = 0 for forward
    analogWrite(rpwmPin, pwm);
    analogWrite(lpwmPin, 255);
  } 
 if (reverse == 0) {
    analogWrite(lpwmPin, pwm);
    analogWrite(rpwmPin, 255);
  }
}

void disableAll(){
  digitalWrite(leftMotorEn, LOW);
  digitalWrite(rightMotorEn, LOW);
}

