#pragma region includes
#include <Fsm.h>
#include <mpu_wrappers.h>
#include <Servo.h>
#include <PCF8574.h>
#include <helper_3dmath.h>
#include <MPU6050_6Axis_MotionApps20.h>

#include <I2Cdev.h>
#include <NewPing.h>
#include <Wire.h>
#pragma endregion includes

#define DEBUG_MODE true

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

#define CONST_SERIAL_RPI_INITIALIZED 48
#define CONST_SERIAL_RPI_STOP 49
#define CONST_SERIAL_RPI_START 50
#define CONST_SERIAL_RPI_SPEED 51
#define CONST_SERIAL_RPI_DIRECTION 52

#define CONST_STATE_MOVEMENT 70
#define CONST_STATE_STOP 71
#define CONST_STATE_STAIRS 72
#define CONST_STATE_OBSTACLE 73
#define CONST_OBSTACLE_SWITCHES 74
#define CONST_OBSTACLE_SONAR_FRONT 75
#define CONST_OBSTACLE_SONAR_BACK 76
#define CONST_FLOOR 77
#define CONST_LIMIT 78
#define CONST_MPU 79
//TODO: Dobrac wspolczynniki
#define CONST_SPEED_FACTOR 0.315
#define CONST_DIRECT_FACTIOR 1.4173 //180/127
#define CONST_STEERING_FACTOR 0.5

#pragma endregion defines

#pragma region globals
unsigned long timeOfLastStateSwitch;
unsigned long timeOfSonarFrontRestart;
unsigned long timeOfSonarRearRestart;
unsigned long timeLastLowPriorityCycle;

PCF8574 expander;
MPU6050 mpu;
Servo left;
Servo right; 
int leftStop = 111;
int rightStop = 81;
unsigned int g_ultrasondTreshold;
NewPing sonarBack(trigPinBack, echoPinBack, MAX_DISTANCE);
NewPing sonarFront(trigPinFront,echoPinFront,MAX_DISTANCE);


byte g_ekspanderSensors;
byte g_limitSwitchesSensors;
byte g_FrontUltraSondDistance;
byte g_RearUltraSondDistance;
byte g_frontSonarState;
byte g_rearSonarState;
byte g_SetSpeed;
byte g_SetDirection;
float yawOffset;

//bool workingIRsensors[7];

#pragma endregion globals

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
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

#pragma region Fsm
#define FSM_STOP 1
#define FSM_MOVEMENT 2
#define FSM_OBSTACLE 3
#define FSM_STAIRS 4



void serialFlush(){
  while(Serial.available() > 0) {
    char t = Serial.read();
  }
} 

State state_movement(&on_movement_enter, &on_movement, &on_movement_exit);
State state_initialize(&on_initialize_enter, &on_initialize, &on_initialize_exit);
State state_stop(&on_stop_enter,&on_stop,&on_stop_exit);
State state_obstacle_detected(&on_obstacle_detected_enter,&on_obstacle,&on_obstacle_detected_exit);
State state_stairs_detected(&on_stairs_detected_enter,&on_stairs_detected,&on_stairs_detected_exit);

Fsm fsm(&state_initialize);


void on_movement_enter(){
    attachServo();
    stopEngines();
    #ifdef DEBUG_MODE
    //Serial.println("on_movement_enter");
    Serial.write(uint8_t(CONST_STATE_MOVEMENT));
    #else
    Serial.write(uint8_t(CONST_STATE_MOVEMENT));
    #endif
    //TODO: wysyłanie informacji o zmianie stanu
}
void on_movement(){
    //DEBUG: funkcja do sterowania serwami, bierze informacje o zadanej prędkosći i zadanym kierunku
    
    TaskMTU();
    #ifdef DEBUG_MODE
        g_SetSpeed = 180;
        g_SetDirection = 127;
        if(millis()-timeOfLastStateSwitch > 30000){

            g_SetDirection = 60;
        }
        if(millis()-timeOfLastStateSwitch > 70000){

            g_SetSpeed = 160;
        }
        //Serial.println((ypr[0]-yawOffset)*180/M_PI);
        //Serial.println((g_SetDirection - 128) * CONST_DIRECT_FACTIOR);
    #endif
    controlEngines((ypr[0]-yawOffset)*180/M_PI,g_SetSpeed,g_SetDirection);
    
    if(millis()-timeLastLowPriorityCycle > 100){
        if(Serial.available()>0){
        byte readChar = Serial.read();
            if (readChar == CONST_SERIAL_RPI_STOP){
                timeOfLastStateSwitch = millis();
                fsm.trigger(FSM_STOP);
            }else if(readChar == CONST_SERIAL_RPI_SPEED){
                //wait for data
                while(Serial.available()==0){}
                    g_SetSpeed = Serial.read();
                    //Serial.println(g_SetSpeed);   
            }else if(readChar == CONST_SERIAL_RPI_DIRECTION){
                //wait for data
                while(Serial.available()==0){}
                    g_SetDirection = Serial.read();
                    //Serial.println(g_SetDirection);   
            }
        }
    } 
    bool noFloorDetected = false;
    GetFloorSensors();
    if (g_ekspanderSensors != 0){
        noFloorDetected = true;
    }
    if(noFloorDetected && millis() - timeOfLastStateSwitch > 50){
        timeOfLastStateSwitch = millis();
        byte writePack[2] = {uint8_t(CONST_FLOOR), g_ekspanderSensors};
        Serial.write(writePack,2);
        fsm.trigger(FSM_STAIRS);
    }
    bool obstacleDetected = false;
    GetLimitSwitchSensors();
    if(g_limitSwitchesSensors < 15){
        byte writePack1[2] = {uint8_t(CONST_LIMIT), g_ekspanderSensors};
        Serial.write(writePack1,2);
        obstacleDetected = true;
    }
    // bool noFloorDetected = false;
    // for(int i = 0;i<7;i++){
    //     if(expander.digitalRead(i)==HIGH){//workingIRsensors[i] == true && expander.digitalRead(i)==HIGH){
    //         Serial.print("Wykryto brak podlogi pod czujnikiem nr ");
    //         Serial.println(i);
    //         noFloorDetected = true;
    //     }
    // }
    // if(noFloorDetected & millis() - timeOfLastStateSwitch > 50){
    //     timeOfLastStateSwitch = millis();
    //     fsm.trigger(FSM_STAIRS);
    // }
    // bool obstacleDetected = false;
    // for(int i = UpLeft;i<=BackLeft;i++){
    //     if(digitalRead(i)==LOW){
    //         Serial.print("Wykryto zderzenie z przeszkoda na czujniku nr ");
    //         Serial.println(i);
    //         obstacleDetected = true;
    //     }
    // }
    if(obstacleDetected == true && millis() - timeOfLastStateSwitch > 50){
        timeOfLastStateSwitch = millis();
        fsm.trigger(FSM_OBSTACLE);
    }
    if(millis()-timeLastLowPriorityCycle > 100){
        TaskFrontUltasond();

        //Send info to Rasp
        byte writePacket1[2] = {uint8_t(CONST_OBSTACLE_SONAR_FRONT),g_FrontUltraSondDistance};
        Serial.write(writePacket1,2);

        if(g_FrontUltraSondDistance < g_ultrasondTreshold && g_FrontUltraSondDistance != 0 && millis() - timeOfLastStateSwitch > 50){
            timeOfLastStateSwitch = millis();
            //Serial.println("Za maly dystans do przeszkody z przodu");
            //Serial.println(g_FrontUltraSondDistance);
            fsm.trigger(FSM_OBSTACLE);
        }
        TaskRearUltrasond();  

        //Send info to Rasp
        byte writePacket[2] = {uint8_t(CONST_OBSTACLE_SONAR_BACK),g_RearUltraSondDistance};
        Serial.write(writePacket,2);

        if(g_RearUltraSondDistance < g_ultrasondTreshold && g_RearUltraSondDistance != 0 && millis() - timeOfLastStateSwitch > 50){
            timeOfLastStateSwitch = millis();
            //Serial.println("Za maly dystans do przeszkody z tyłu");
            fsm.trigger(FSM_OBSTACLE);
        }
        timeLastLowPriorityCycle = millis();
    }
}
void on_movement_exit(){
    #ifdef DEBUG_MODE
    //Serial.println("on_movement_exit");
    #endif
}
void on_initialize_enter(){
    
    serialFlush();

    //uncomment when using workingIRsensors
    // for(int i = 0;i<7;i++){
    //         workingIRsensors[i] = true;
    // }
    timeOfLastStateSwitch = millis();
    //Serial.println("on_initialize_enter");
}
void on_initialize_exit(){
    yawOffset = ypr[0];
    //Serial.println("on_initialize_exit");
}
void on_initialize(){
    byte cInput;
    TaskMTU();
    //uncomment when using workingIRsensors
    // for(int i = 0;i<7;i++){
    //     if(expander.digitalRead(i)==HIGH){
    //         workingIRsensors[i] = false;
    //     }
    // }
    if(millis()-timeOfLastStateSwitch > 5000){
        if(Serial.available()>0){
            cInput = Serial.read();
            if(cInput == CONST_SERIAL_RPI_INITIALIZED){
                timeOfLastStateSwitch = millis();
                fsm.trigger(FSM_MOVEMENT);
            }
        }
    }

    #ifdef DEBUG_MODE
        if(millis()-timeOfLastStateSwitch > 10000){
            timeOfLastStateSwitch = millis();
            fsm.trigger(FSM_MOVEMENT);
        }
    #endif
    
}
void on_stop_enter(){

    stopEngines();
    detachServo();
    #ifdef DEBUG_MODE
    //Serial.println("on_stop_enter");
    Serial.write(uint8_t(CONST_STATE_STOP));
    #else
    Serial.write(uint8_t(CONST_STATE_STOP));
    #endif
}
void on_stop(){
    boolean noFloorDetected = false;
    #ifdef DEBUG_MODE
        if(millis() - timeOfLastStateSwitch > 30000){
            timeOfLastStateSwitch = millis();
            fsm.trigger(FSM_MOVEMENT);
        }
    #endif

    if(Serial.available()>0){
    byte readChar = Serial.read();
        if (readChar = CONST_SERIAL_RPI_START){
            timeOfLastStateSwitch = millis();
            fsm.trigger(FSM_MOVEMENT);
        }
    }

    //DEBUG: Odkomentować, jak poprawię czujniki
    GetFloorSensors();
    if(g_ekspanderSensors != 0 ){
        noFloorDetected = true;
    }
    // for(int i = 0;i<7;i++){
    //     if(expander.digitalRead(i)==HIGH){ //if(workingIRsensors[i] == true && expander.digitalRead(i)==HIGH){
    //         Serial.print("Wykryto brak podlogi pod czujnikiem nr ");
    //         Serial.println(i);
    //         noFloorDetected = true;
    //     }
    // }
    if(noFloorDetected && millis() - timeOfLastStateSwitch > 50){
        timeOfLastStateSwitch = millis();
        byte writePacket[2] = {uint8_t(CONST_FLOOR), g_ekspanderSensors};
        Serial.write(writePacket,2);
        fsm.trigger(FSM_STAIRS);
    }
    bool obstacleDetected = false;
    GetLimitSwitchSensors();
    if(g_limitSwitchesSensors < 15){
        byte writePacket1[2] = {uint8_t(CONST_LIMIT), g_ekspanderSensors};
        Serial.write(writePacket1,2);
        obstacleDetected = true;
    }
    // for(int i = UpLeft;i<=BackLeft;i++){
    //     if(digitalRead(i)==LOW){
    //         Serial.print("Wykryto zderzenie z przeszkoda na czujniku nr ");
    //         Serial.println(i);
    //         obstacleDetected = true;
    //     }
    // }
    TaskFrontUltasond();
    TaskRearUltrasond();
    if(g_FrontUltraSondDistance < g_ultrasondTreshold || g_RearUltraSondDistance < g_ultrasondTreshold)
        obstacleDetected = true;
    if(obstacleDetected == true && millis() - timeOfLastStateSwitch > 50){
        timeOfLastStateSwitch = millis();
        fsm.trigger(FSM_OBSTACLE);
    }  
}

void on_stop_exit(){
    attachServo();
    //Serial.println("on_stop_exit");
}

void on_obstacle_detected_enter(){
    stopEngines();
    //Serial.println("on_obstacle_detected_enter");
}
void on_obstacle(){
    //DEBUG: Sprawdzić cofanie
    if(Serial.available()>0){
        byte readChar = Serial.read();
        if(readChar == CONST_SERIAL_RPI_SPEED){
            byte speed;
            //wait for data
            while(Serial.available()==0){}
                speed = Serial.read();
                controlEngines(0,speed,128);  
        }
    }

    // #ifdef DEBUG_MODE
    //     if(millis()-timeOfLastStateSwitch > 10000){
    //         controlEngines(0,60,128);
    //         delay(2000);
    //         controlEngines(0,128,60);
    //         delay(2000);
    //     }
    // #endif
    bool obstacleDetected = false;
    TaskFrontUltasond();
    TaskRearUltrasond();  
    GetLimitSwitchSensors();
    if(g_limitSwitchesSensors < 15){
        obstacleDetected = true;
    }
    if(obstacleDetected == false && millis()-timeOfLastStateSwitch > 50 && g_FrontUltraSondDistance > g_ultrasondTreshold && g_RearUltraSondDistance > g_ultrasondTreshold){ 
        timeOfLastStateSwitch = millis();
        fsm.trigger(FSM_OBSTACLE);
    }
}
void on_obstacle_detected_exit(){
    //Serial.println("on_obstacle_detected_exit");
}
void on_stairs_detected(){
    //DEBUG: Sprawdzić cofanie
    if(Serial.available()>0){
        byte readChar = Serial.read();
        if(readChar == CONST_SERIAL_RPI_SPEED){
            byte speed;
            //wait for data
            while(Serial.available()==0){}
                speed = Serial.read();
                controlEngines(0,speed,128);  
        }
    }
    // #ifdef DEBUG_MODE
    //     if(millis()-timeOfLastStateSwitch > 3000){
    //         controlEngines(0,128,0);
    //         delay(2000);
    //         controlEngines(0,128,255);
    //         delay(2000);
    //     }
    // #endif
    
    bool noFloorDetected = false;
    // for( int i = 0;i<7;i++){
    //     if(expander.digitalRead(i)==HIGH){//workingIRsensors[i] == true && expander.digitalRead(i)==HIGH){
    //         noFloorDetected = true;
    //     }
    // }
    GetFloorSensors();
    if(g_ekspanderSensors != 0) {
        noFloorDetected = true;
    }
    if(!noFloorDetected && millis() - timeOfLastStateSwitch > 50){
        timeOfLastStateSwitch = millis();
        fsm.trigger(FSM_STAIRS);
    }
}
void on_stairs_detected_enter(){
    stopEngines();
    Serial.write(uint8_t(CONST_STATE_STAIRS));

    //Serial.println("on_stairs_detected_enter");
}
void on_stairs_detected_exit(){
    //Serial.println("on_stairs_detected_exit");
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

    //Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    //Serial.println(F("Testing device connections..."));
    //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    //Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);
    mpu.setZAccelOffset(1688); // 1688 factory default for my test chip

	 // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        //Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        // enable Arduino interrupt detection
        //Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        //Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        //Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        //Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        //Serial.print(F("DMP Initialization failed (code "));
        //Serial.print(devStatus);
        //Serial.println(F(")"));

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}
}
byte restartFrontSonar(byte previousState){
    byte state = 0;
    unsigned long currentTime = millis();
    if(previousState==4){
        pinMode(echoPinFront, OUTPUT);
        timeOfSonarFrontRestart = millis();
        state = 1;
        return state;
    }
    if(previousState == 1 && currentTime-timeOfSonarFrontRestart > 150){
        digitalWrite(echoPinFront, LOW);
        timeOfSonarFrontRestart = currentTime;
        state = 2;
        return state;
    }
    if(previousState == 2 && currentTime-timeOfSonarFrontRestart > 150){
        pinMode(echoPinFront, INPUT);
        timeOfSonarFrontRestart = currentTime;
        state = 3;
        return state;
    }
    if(previousState == 3 && currentTime-timeOfSonarFrontRestart > 150){
        timeOfSonarFrontRestart = currentTime;
        state = 0;
        return state;
    }
    return previousState;
}
byte restartRearSonar(byte previousState){
    byte state = 0;
    unsigned long currentTime = millis();
    // Serial.print("Prev state");
    // Serial.println(previousState);
    if(previousState==4){
        pinMode(echoPinBack, OUTPUT);
        timeOfSonarRearRestart = millis();
        state = 1;
        return state;
    }
    if(previousState == 1 && currentTime-timeOfSonarRearRestart > 150){
        digitalWrite(echoPinBack, LOW);
        timeOfSonarRearRestart = millis();
        state = 2;
        return state;
    }
    if(previousState == 2 && currentTime-timeOfSonarRearRestart > 150){
        pinMode(echoPinBack, INPUT);
        timeOfSonarRearRestart = millis();
        state = 3;
        return state;
    }
    if(previousState == 3 && currentTime-timeOfSonarRearRestart > 150){
        timeOfSonarRearRestart = millis();
        state = 0;
        return state;
    }
    return previousState;
}
void stopEngines(){
    if(right.attached()){
        right.write(rightStop);//stop signal
        left.write(leftStop);//stop signal
    }
}

void controlEngines(float yaw,byte speed, byte direction){
    /*
    CONST_SPEED_FACTOR 0.315
    CONST_DIRECT_FACTIOR 1.4173 //180/127
    CONST_STEERING_FACTOR 0.0511
    */
    int rightSpeed = rightStop - ((speed - 128)*CONST_SPEED_FACTOR) + CONST_SPEED_FACTOR*CONST_STEERING_FACTOR *
     (((direction - 128) * CONST_DIRECT_FACTIOR) - yaw);
    //Serial.println(rightSpeed);
    right.write(rightSpeed);

    

    int leftSpeed = leftStop + ((speed - 128)*CONST_SPEED_FACTOR )+ CONST_SPEED_FACTOR*CONST_STEERING_FACTOR *
     (((direction - 128) * CONST_DIRECT_FACTIOR) - yaw);
    left.write(leftSpeed);
    //Serial.println(leftSpeed);
}

void setup() {
	// join I2C bus (I2Cdev library doesn't do this automatically)
    Serial.begin(115200);
    g_frontSonarState = 0;
    g_rearSonarState = 0;
    g_ultrasondTreshold = 15;
    g_SetSpeed = 128;
    g_SetDirection = 128;

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
  


}

void attachServo(){
    if(!right.attached()){
        left.attach(10, 1000, 1800); //right servo motor
        right.attach(9, 100, 1800); //l//
    }
}

void detachServo(){
    if(right.attached()){
        left.detach();
        right.detach();
    }
}

void loop(){
    fsm.run_machine();
}


void TaskMTU(){
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    if (!mpuInterrupt && fifoCount < packetSize) {
        if (mpuInterrupt && fifoCount < packetSize) {
          // try to get out of the infinite loop 
          fifoCount = mpu.getFIFOCount();
        }  
        // other program behavior stuff here
        return;
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
        //Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
        // wait for correct available data length, should be a VERY short wait
        if (fifoCount < packetSize){
            fifoCount = mpu.getFIFOCount();
            return;
        } 

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;


            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            
            // Serial.print("ypr\t");
            // Serial.print(ypr[0] * 180/M_PI);
            // Serial.print("\t");
            // Serial.print(ypr[1] * 180/M_PI);
            // Serial.print("\t");
            // Serial.println(ypr[2] * 180/M_PI);
    }
}

void TaskFrontUltasond(  )  // This is a Task.
{
        if(g_frontSonarState != 0){
            g_frontSonarState = restartFrontSonar(g_frontSonarState);
            return;
        }
        int uS = sonarFront.ping_cm();
        
        if (uS==0)
        {
            //Serial.println("Reseutuje przedni sonar");
            g_frontSonarState = restartFrontSonar(4);
        }else{
            g_FrontUltraSondDistance = uS;
           
        }
}

void TaskRearUltrasond(){
    if(g_rearSonarState != 0){    
            g_rearSonarState = restartRearSonar(g_rearSonarState);
            return;
    }
    int uS = sonarBack.ping_cm();

    if (uS==0)
    {
        //Serial.println("Resetuje tylni sonar");
        g_rearSonarState = restartRearSonar(4);
    }else{
         g_RearUltraSondDistance = uS;
        
    }

}

void GetLimitSwitchSensors(){
    g_limitSwitchesSensors = 1;
    for(int i = UpLeft;i<=BackLeft;i++){
        g_limitSwitchesSensors = g_limitSwitchesSensors & digitalRead(i); 
        g_limitSwitchesSensors <<=1;
    }
}
void GetFloorSensors(){
    g_ekspanderSensors = 1;
    for(int i = 0;i<7;i++){
        g_ekspanderSensors = g_ekspanderSensors & expander.digitalRead(i); 
        g_ekspanderSensors <<=1;
    }
}
 
