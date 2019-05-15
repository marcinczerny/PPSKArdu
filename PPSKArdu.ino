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

//TODO: Dobrac wspolczynniki
#define CONST_SPEED_FACTOR 0.315
#define CONST_DIRECT_FACTIOR 1.4173 //180/127
#define CONST_STEERING_FACTOR 0.05

#pragma endregion defines

#pragma region globals
unsigned long timeOfLastStateSwitch;
unsigned long timeOfSonarFrontRestart;
unsigned long timeOfSonarRearRestart;
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
unsigned int g_FrontUltraSondDistance;
//unsigned int FrontUltrasondArray[7]={0,0,0,0,0,0,0};
//unsigned int RearUltrasondArray[7]={0,0,0,0,0,0,0};
unsigned int g_RearUltraSondDistance;
byte g_frontSonarState;
byte g_rearSonarState;
byte g_SetSpeed;
byte g_SetDirection;
float yawOffset;

bool workingIRsensors[7];

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
    Serial.println("on_movement_enter");
    //TODO: wysyłanie informacji o zmianie stanu
}
void on_movement(){
    //DEBUG: funkcja do sterowania serwami, bierze informacje o zadanej prędkosći i zadanym kierunku
    TaskMTU();
    controlEngines((ypr[0]-yawOffset)*180/M_PI,g_SetSpeed,g_SetDirection);
    
    
    if(Serial.available()>0){
    byte readChar = Serial.read();
        if (readChar == CONST_SERIAL_RPI_STOP){
            fsm.trigger(FSM_STOP);
        }else if(readChar == CONST_SERIAL_RPI_SPEED){
            //wait for data
            while(Serial.available()==0){}
                g_SetSpeed = Serial.read();
                Serial.println(g_SetSpeed);   
        }else if(readChar == CONST_SERIAL_RPI_DIRECTION){
            //wait for data
            while(Serial.available()==0){}
                g_SetDirection = Serial.read();
                Serial.println(g_SetDirection);   
        }
    }
    
    //TODO: Odkomentować, jak poprawimy czujniki IR
    // bool noFloorDetected = false;
    // for(int i = 0;i<7;i++){
    //     if(workingIRsensors[i] == true && expander.digitalRead(i)==HIGH){
    //         Serial.print("Wykryto brak podlogi pod czujnikiem nr ");
    //         Serial.println(i);
    //         noFloorDetected = true;
    //     }
    // }
    // if(noFloorDetected & millis() - timeOfLastStateSwitch > 50){
    //     timeOfLastStateSwitch = millis();
    //     //fsm.trigger(FSM_STAIRS);
    // }
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

    TaskFrontUltasond();
    if(g_FrontUltraSondDistance < g_ultrasondTreshold && g_FrontUltraSondDistance != 0 && millis() - timeOfLastStateSwitch > 50){
        timeOfLastStateSwitch = millis();
        Serial.println("Za maly dystans do przeszkody z przodu");
        Serial.println(g_FrontUltraSondDistance);
        fsm.trigger(FSM_OBSTACLE);
    }
    TaskRearUltrasond();  
    if(g_RearUltraSondDistance < g_ultrasondTreshold && g_RearUltraSondDistance != 0 && millis() - timeOfLastStateSwitch > 50){
        timeOfLastStateSwitch = millis();
        Serial.println("Za maly dystans do przeszkody z tyłu");
        fsm.trigger(FSM_OBSTACLE);
    }
}
void on_movement_exit(){
    Serial.println("on_movement_exit");
}
void on_initialize_enter(){
    
    serialFlush();

    for(int i = 0;i<7;i++){
            workingIRsensors[i] = true;
    }
    timeOfLastStateSwitch = millis();
    Serial.println("on_initialize_enter");
}
void on_initialize_exit(){
    yawOffset = ypr[0];
    Serial.println("on_initialize_exit");
}
void on_initialize(){
    byte cInput;
    TaskMTU();
    for(int i = 0;i<7;i++){
        if(expander.digitalRead(i)==HIGH){
            workingIRsensors[i] = false;
        }
    }
    if(millis()-timeOfLastStateSwitch > 5000){
        if(Serial.available()>0){
            cInput = Serial.read();
            if(cInput == CONST_SERIAL_RPI_INITIALIZED){
                fsm.trigger(FSM_MOVEMENT);
            }
        }
    }
    
}
void on_stop_enter(){
    stopEngines();
    Serial.println("on_stop_enter");
}
void on_stop(){
    boolean noFloorDetected = false;

    if(Serial.available()>0){
    byte readChar = Serial.read();
        if (readChar = CONST_SERIAL_RPI_START){
            fsm.trigger(FSM_MOVEMENT);
        }
    }

    for(int i = 0;i<7;i++){
        if(workingIRsensors[i] == true && expander.digitalRead(i)==HIGH){
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
    Serial.println("on_stop_exit");
}

void on_obstacle_detected_enter(){
    stopEngines();
    Serial.println("on_obstacle_detected_enter");
}
void on_obstacle(){
    bool obstacleDetected = false;
    TaskFrontUltasond();
    TaskRearUltrasond();  
    for(int i = UpLeft;i<=BackLeft;i++){
        if(digitalRead(i)==LOW){
            obstacleDetected = true;
        }
    }
    if(obstacleDetected == false && millis()-timeOfLastStateSwitch > 50 && g_FrontUltraSondDistance > g_ultrasondTreshold && g_RearUltraSondDistance > g_ultrasondTreshold){ 
        timeOfLastStateSwitch = millis();
        fsm.trigger(FSM_OBSTACLE);
    }
}
void on_obstacle_detected_exit(){
    Serial.println("on_obstacle_detected_exit");
}
void on_stairs_detected(){
    bool noFloorDetected = false;
    for( int i = 0;i<7;i++){
        if(workingIRsensors[i] == true && expander.digitalRead(i)==HIGH){
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
/*void setupMPU(){

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
}*/
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
    right.write(rightStop);//stop signal
    left.write(leftStop);//stop signal
}

void controlEngines(float yaw,byte speed, byte direction){
    int rightSpeed = rightStop + ((speed - 128)*CONST_SPEED_FACTOR + CONST_SPEED_FACTOR*CONST_STEERING_FACTOR *
     ((direction - 128) * CONST_DIRECT_FACTIOR) - yaw);
    
    right.write(rightSpeed);
    Serial.print("rightSpeed: ");
    Serial.println(rightSpeed);

    int leftSpeed = leftStop - ((speed - 128)*CONST_SPEED_FACTOR + CONST_SPEED_FACTOR*CONST_STEERING_FACTOR *
     ((direction - 128) * CONST_DIRECT_FACTIOR) - yaw);
    left.write(leftSpeed);
    Serial.print("leftspeed: ");
    Serial.println(leftSpeed);
}

void setup() {
	// join I2C bus (I2Cdev library doesn't do this automatically)
    Serial.begin(115200);
    g_frontSonarState = 0;
    g_rearSonarState = 0;
    g_ultrasondTreshold = 15;

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
  Serial.println("Jestem tu: ");
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
        Serial.println(F("FIFO overflow!"));

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
// int sort_desc(const void *cmp1, const void *cmp2)
// {
//   // Need to cast the void * to int *
//   int a = *((int *)cmp1);
//   int b = *((int *)cmp2);
//   // The comparison
//   //return a > b ? -1 : (a < b ? 1 : 0);
//   // A simpler, probably faster way:
//   return b - a;
// }
void TaskFrontUltasond(  )  // This is a Task.
{
        if(g_frontSonarState != 0){
            g_frontSonarState = restartFrontSonar(g_frontSonarState);
            return;
        }
        int uS = sonarFront.ping_cm();
        
        if (uS==0)
        {
            Serial.println("Reseutuje przedni sonar");
            g_frontSonarState = restartFrontSonar(4);
        }else{
            g_FrontUltraSondDistance = uS;
            // unsigned int tempArray[7];
            // for(int i =0;i<6;i++){
            //         FrontUltrasondArray[i]=FrontUltrasondArray[i+1];
            //         tempArray[i]=FrontUltrasondArray[i];
            // }
            // FrontUltrasondArray[6]=uS;
            // tempArray[6] = uS;
            // int array_length = sizeof(FrontUltrasondArray) / sizeof(FrontUltrasondArray[0]);
            // if(FrontUltrasondArray[0]!=0){
            //     qsort(tempArray,array_length,sizeof(tempArray[0]),sort_desc);
            //     g_FrontUltraSondDistance = tempArray[int(array_length/2) + 1];
            //     // Serial.print("FronSonar: ");
            //     // for(int j = 0;j<7;j++){
            //     //     Serial.print("tab[");
            //     //     Serial.print(j);
            //     //     Serial.print("]= ");
            //     //     Serial.print(FrontUltrasondArray[j]);
            //     // }
                
            //     // Serial.println(g_FrontUltraSondDistance);
            // }
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
        // unsigned int tempArray[7];
        //     for(int i =0;i<6;i++){
        //             RearUltrasondArray[i]=RearUltrasondArray[i+1];
        //             tempArray[i]=RearUltrasondArray[i];
        //     }
        //     RearUltrasondArray[6]=uS;
        //     tempArray[6]=uS;
        //     int array_length = sizeof(RearUltrasondArray) / sizeof(RearUltrasondArray[0]);
        //     if(RearUltrasondArray[0]!=0){
        //         qsort(tempArray,array_length,sizeof(tempArray[0]),sort_desc);
        //         g_RearUltraSondDistance = tempArray[int(array_length/2) + 1];
        //         // Serial.print("FronSonar: ");
        //         // for(int j = 0;j<7;j++){
        //         //     Serial.print("tab[");
        //         //     Serial.print(j);
        //         //     Serial.print("]= ");
        //         //     Serial.print(RearUltrasondArray[j]);
        //         // }
        //         // Serial.print("  DYSTNS:   ");
        //         // Serial.println(g_RearUltraSondDistance);
        //     }
    }

}

// void TaskSwitches(void *pvParameters __attribute__((unused))){
//     for(;;){
//         g_limitSwitchesSensors = 1;
//         for(int i = UpLeft;i<=BackLeft;i++){
//             g_limitSwitchesSensors = g_limitSwitchesSensors & digitalRead(i); 
//             g_limitSwitchesSensors <<=1;
//         }
//     }
// }
// void TaskExpander(){
//     for(;;){
//         g_ekspanderSensors = 1;
//         for(int i = 0;i<7;i++){
//             g_ekspanderSensors = g_ekspanderSensors & expander.digitalRead(i); 
//             g_ekspanderSensors <<=1;
//         }
//     }
// }
