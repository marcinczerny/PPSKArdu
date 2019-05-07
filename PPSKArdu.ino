#pragma region includes
#include <Fsm.h>
#include <mpu_wrappers.h>
#include <Servo.h>
#include <PCF8574.h>
#include <helper_3dmath.h>
#include <MPU6050_6Axis_MotionApps20.h>
//#include <MPU6050_6Axis_Motio"/home/marcin/Pobrane/arduino-1.8.9/tools/**",
               // "/home/marcin/Pobrane/arduino-1.8.9/hardware/arduino/avr/**",nApps20.h>

#include <I2Cdev.h>
#include <NewPing.h>
#include <Wire.h>
#pragma endregion includes

#pragma region defines
#define trigPin A2
#define echoPin A1
#define trigPinBack A0
#define trigPinFront 12
#define echoPinBack 3
#define echoPinFront 13
#define MAX_DISTANCE 600

#define BackRight 6
#define BackLeft 7
#define UpLeft 4
#define UpRight 5
#define expanderLeftUpWheel 0
#define expanderRightUpWheel 1
#define UpCenter 2
#pragma endregion defines

#pragma region globals
unsigned long timeOfLastStateSwitch;
PCF8574 expander;
MPU6050 mpu;
Servo left;
Servo right; 
int leftStop = 111;
int rightStop = 81;
NewPing sonarBack(trigPinBack, echoPinBack, MAX_DISTANCE);
NewPing sonarFront(trigPinFront,echoPinFront,MAX_DISTANCE);


byte g_ekspanderSensors;
byte g_limitSwitchesSensors;
unsigned int g_FrontUltraSondDistance;
unsigned int g_RearUltraSondDistance;

#pragma endregion globals

#pragma region FreeRtosTasks
void TaskFSM(void *pvParameters);
void TaskFrontUltasond(void *pvParameters);
void TaskRearUltrasond(void *pvParameters);
void TaskMTU(void *pvParameters);
void TaskExpander(void *pvParameters);
void TaskSwitches(void *pvParameters);
void TaskSerialRead(void *pvParameters);
#pragma endregion FreeRtosTasks

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
//VectorInt16 aa;         // [x, y, z]            accel sensor measurements
//VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
//VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
//float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

#pragma region Fsm
#define FSM_STOP 1
#define FSM_MOVEMENT 2
#define FSM_OBSTACLE 3
#define FSM_STAIRS 4

State state_movement(&on_movement_enter, &on_movement, &on_movement_exit);
State state_initialize(&on_initialize_enter, &on_initialize, &on_initialize_exit);
State state_stop(&on_stop_enter,&on_stop,&on_stop_exit);
State state_obstacle_detected(&on_obstacle_detected_enter,&on_obstacle,&on_obstacle_detected_exit);
State state_stairs_detected(&on_stairs_detected_enter,&on_stairs_detected,&on_stairs_detected_exit);

Fsm fsm(&state_initialize);


void on_movement_enter(){
    Serial.println("on_movement_enter");
}
void on_movement(){
    digitalWrite(8,LOW);
    left.write(leftStop + 10);
    right.write(rightStop - 10);
    if(Serial.available()>0){
    byte readChar = Serial.read();
        if (readChar = 'o'){
            fsm.trigger(FSM_STOP);

        }
    }
    bool noFloorDetected = false;
    for(int i = 0;i<7;i++){
        if(expander.digitalRead(i)==HIGH){
            Serial.print("Wykryto brak podlogi pod czujnikiem nr ");
            Serial.println(i);
            noFloorDetected = true;
        }
    }
    if(noFloorDetected & millis() - timeOfLastStateSwitch > 50){
        timeOfLastStateSwitch = millis();
        fsm.trigger(FSM_STAIRS);
    }
    bool obstacleDetected = false;
    for(int i = UpLeft;i<=BackLeft;i++){
        if(digitalRead(i)==LOW){
            Serial.print("Wykryto zderzenie z przeszkoda na czujniku nr ");
            Serial.println(i);
            obstacleDetected = true;
        }
    }
    if(obstacleDetected == true && millis() - timeOfLastStateSwitch > 50){
        timeOfLastStateSwitch = millis();
        fsm.trigger(FSM_OBSTACLE);
    }  
}
void on_movement_exit(){
    Serial.println("on_movement_exit");
}
void on_initialize_enter(){
    timeOfLastStateSwitch = millis();
    Serial.println("on_initialize_enter");
}
void on_initialize_exit(){
    Serial.println("on_initialize_exit");
}
void on_initialize(){
    delay(5000);
    fsm.trigger(FSM_MOVEMENT);
}
void on_stop_enter(){
    stopEngines();
    Serial.println("on_stop_enter");
}
void on_stop(){
    boolean noFloorDetected = false;
    if(Serial.available()>0){
    byte readChar = Serial.read();
        if (readChar = 'o'){
            fsm.trigger(FSM_MOVEMENT);
        }
    }
    for(int i = 0;i<7;i++){
        if(expander.digitalRead(i)==HIGH){
            Serial.print("Wykryto brak podlogi pod czujnikiem nr ");
            Serial.println(i);
            noFloorDetected = true;
        }
    }
    if(noFloorDetected && millis() - timeOfLastStateSwitch > 50){
        timeOfLastStateSwitch = millis();
        fsm.trigger(FSM_STAIRS);
    }
    bool obstacleDetected = false;
    for(int i = UpLeft;i<=BackLeft;i++){
        if(digitalRead(i)==LOW){
            Serial.print("Wykryto zderzenie z przeszkoda na czujniku nr ");
            Serial.println(i);
            obstacleDetected = true;
        }
    }

    if(obstacleDetected == true && millis() - timeOfLastStateSwitch > 50){
        timeOfLastStateSwitch = millis();
        fsm.trigger(FSM_OBSTACLE);
    }  
}
void on_stop_exit(){
    Serial.println("on_stop_exit");
}

void on_obstacle_detected_enter(){
    stopEngines();
    Serial.println("on_obstacle_detected_enter");
}
void on_obstacle(){
    bool obstacleDetected = false;
    for(int i = UpLeft;i<=BackLeft;i++){
        if(digitalRead(i)==LOW){
            obstacleDetected = true;
        }
    }
    if(obstacleDetected == false && millis()-timeOfLastStateSwitch > 50){
        timeOfLastStateSwitch = millis();
        fsm.trigger(FSM_OBSTACLE);
    }  
}
void on_obstacle_detected_exit(){
    Serial.println("on_obstacle_detected_exit");
}
void on_stairs_detected(){
    bool noFloorDetected = false;
    for(int i = 0;i<7;i++){
        if(expander.digitalRead(i)==HIGH){
            noFloorDetected = true;
        }
    }
    if(!noFloorDetected && millis() - timeOfLastStateSwitch > 50){
        timeOfLastStateSwitch = millis();
        fsm.trigger(FSM_STAIRS);
    }
}
void on_stairs_detected_enter(){
    stopEngines();
    Serial.println("on_stairs_detected_enter");
}
void on_stairs_detected_exit(){
    Serial.println("on_stairs_detected_exit");
}


#pragma endregion Fsm

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
void setupMPU(){

    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);
    mpu.setZAccelOffset(1688); // 1688 factory default for my test chip

	 // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}
}
void restartSonar(int echo, int trig){
    pinMode(echo, OUTPUT);
    delay(150);
    digitalWrite(echo, LOW);
    delay(150);
    pinMode(echo, INPUT);
    delay(150);
}
void stopEngines(){
    right.write(rightStop);//stop signal
    left.write(leftStop);//stop signal
}

void setup() {
	// join I2C bus (I2Cdev library doesn't do this automatically)
    Serial.begin(115200);

    fsm.add_transition(&state_initialize, &state_movement,FSM_MOVEMENT,NULL);
    fsm.add_transition(&state_movement,&state_stop,FSM_STOP,NULL);
    fsm.add_transition(&state_movement,&state_obstacle_detected,FSM_OBSTACLE,NULL);
    fsm.add_transition(&state_movement,&state_stairs_detected,FSM_STAIRS,NULL);
    fsm.add_transition(&state_stop,&state_movement,FSM_MOVEMENT,NULL);
    fsm.add_transition(&state_stop,&state_stairs_detected,FSM_STAIRS,NULL);
    fsm.add_transition(&state_stop,&state_obstacle_detected,FSM_OBSTACLE,NULL);
    fsm.add_transition(&state_stairs_detected,&state_stop,FSM_STAIRS,NULL);
    fsm.add_transition(&state_obstacle_detected,&state_stop,FSM_OBSTACLE,NULL);

    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
	
  setupMPU();
  // put your setup code here, to run once:
  pinMode(6,INPUT);
  pinMode(7,INPUT);
  pinMode(4,INPUT);
  pinMode(5,INPUT);
  pinMode(8,OUTPUT);
  

  expander.begin(0x20);
  expander.pinMode(0,INPUT_PULLUP);
  expander.pinMode(1,INPUT_PULLUP);
  expander.pinMode(2,INPUT_PULLUP);
  expander.pinMode(3,INPUT_PULLUP);
  expander.pinMode(4,INPUT_PULLUP);
  expander.pinMode(5,INPUT_PULLUP);
  expander.pinMode(6,INPUT_PULLUP);
  
  left.attach(10, 1000, 1800); //right servo motor
  right.attach(9, 100, 1800); //l//

  stopEngines();

  Serial.begin(115200);
}

void loop(){

	
	/*while(stopEngines == false){
    right.write(rightStop-10);
    left.write(leftStop + 10);
    delay(1500);
    right.write(rightStop+10);
    left.write(leftStop - 10);
    delay(1500);
    right.write(rightStop+10);
    left.write(leftStop + 10);
    delay(1500);
    right.write(rightStop-10);
    left.write(leftStop - 10);
    delay(1500);
    right.write(rightStop);//stop signal
left.write(leftStop);//stop signal
    stopEngines = true;
    }*/
	
	/*delay(150);*/
  //int uS = sonar.ping_cm();
//   if (uS==0)
//   {
//     Serial.println("MAX: resetting sensor");
//     pinMode(echoPin, OUTPUT);
//     delay(150);
//     digitalWrite(echoPin, LOW);
//     delay(150);
//     pinMode(echoPin, INPUT);
//     delay(150);
//   }
//   else
//   {
//   Serial.print(" ");
//   Serial.print("Ping: ");
//   Serial.print(uS);
//   Serial.println("cm");
//   }
}


void TaskMTU(){
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        if (mpuInterrupt && fifoCount < packetSize) {
          // try to get out of the infinite loop 
          fifoCount = mpu.getFIFOCount();
        }  
        // other program behavior stuff here
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        fifoCount = mpu.getFIFOCount();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;


            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
    }
}
void TaskFrontUltasond(  )  // This is a Task.
{
        int uS = sonarFront.ping_cm();
        if (uS==0)
        {
            restartSonar(echoPinFront,trigPinFront);
        }else{
            g_FrontUltraSondDistance = uS;
        }

}

void TaskRearUltrasond(void *pvParameters __attribute__((unused))){
    for(;;){
        int uS = sonarBack.ping_cm();
        if (uS==0)
        {
            restartSonar(echoPinBack,trigPinBack);
        }else{
            g_RearUltraSondDistance = uS;
        }

    }
}
void TaskSerialRead( void *pvParameters __attribute__((unused)) )  // This is a Task.
{

}
void TaskSwitches(void *pvParameters __attribute__((unused))){
    for(;;){
        g_limitSwitchesSensors = 1;
        for(int i = UpLeft;i<=BackLeft;i++){
            g_limitSwitchesSensors = g_limitSwitchesSensors & digitalRead(i); 
            g_limitSwitchesSensors <<=1;
        }
    }
}
void TaskExpander(){
    for(;;){
        g_ekspanderSensors = 1;
        for(int i = 0;i<7;i++){
            g_ekspanderSensors = g_ekspanderSensors & expander.digitalRead(i); 
            g_ekspanderSensors <<=1;
        }
    }
}
