#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <string.h>
#include <limits.h>

//스피드 세팅(0~255)
#define DEFAULT_SPEED 130

//핀 세팅

#define PIN_BLUETOOTH_RX 0
#define PIN_BLUETOOTH_TX 1
#define PIN_EEPROM_RESET_BTN 2
#define PIN_START_CAR_BTN 3
#define PIN_MOTOR_IN1 4
#define PIN_MOTOR_IN2 5
#define PIN_MOTOR_IN3 7
#define PIN_MOTOR_IN4 8
#define PIN_MOTOR_ENA 11
#define PIN_MOTOR_ENB 12
#define PIN_DOOR_SERVO 9
#define PIN_TRUNK_SERVO 10
#define PIN_START_CAR_OUT 13

class LCD{
    LiquidCrystal_I2C i2c;
    
    int flag;
    unsigned long startTime;
    unsigned long printingTime;
    char lastMessage[2][17];
    const char blank[17] = "                ";
    public:
    LCD():i2c(0x27, 16, 2), flag(0), startTime(0), printingTime(0){
        strcpy(lastMessage[0], blank);
        strcpy(lastMessage[1], blank);
        i2c.begin();
    }
    void clear(){
        i2c.setCursor(0,0); //첫번째 줄의 0번째 부터 출력
        i2c.print(blank);
        i2c.setCursor(0,1); //첫번째 줄의 0번째 부터 출력
        i2c.print(blank);
    }
    void rollback(){
        i2c.setCursor(0,0); //첫번째 줄의 0번째 부터 출력
        i2c.print(lastMessage[0]);
        i2c.setCursor(0,1); //첫번째 줄의 0번째 부터 출력
        i2c.print(lastMessage[1]);
    }
    void update(){
        if(flag == 1 && millis()-startTime > printingTime){
            clear();
            rollback();
            flag = 0;
        }
    }
    void print(const char *line1, const char *line2){
        clear();
        flag = 0;
        if(line1 != NULL){
            i2c.setCursor(0,0); //첫번째 줄의 0번째 부터 출력
            i2c.print(line1);
        }
        if(line2 != NULL){
            i2c.setCursor(0,1); //두번째 줄의 0번째 부터 력출
            i2c.print(line2);
        }
        strcpy(lastMessage[0], line1);
        strcpy(lastMessage[1], line2);
    } 
    void print(const char *line1, const char *line2, unsigned int time){
        clear();
        flag = 1;
        startTime = millis();
        printingTime = time;
        if(line1 != NULL){
            i2c.setCursor(0,0); //첫번째 줄의 0번째 부터 출력
            i2c.print(line1);
        }
        if(line2 != NULL){
            i2c.setCursor(0,1); //두번째 줄의 0번째 부터 력출
            i2c.print(line2);
        }
    }
    void print(const char *line1){
        print(line1, "");
    }
    void print(const char *line1, unsigned time){
        print(line1, "", time);
    }
};

class Motor{
  //모터드라이버 매크로
  #define MOTOR_LEFT 16
  #define MOTOR_RIGHT 32
  #define MOTOR_FORWARD 8
  #define MOTOR_BACK 4
  #define MOTOR_STOP 2
  private:
  int in1, in2, in3, in4, ena, enb;
  public:
  Motor(unsigned char defaultSpeed, int pinIn1, int pinIn2, int pinIn3, int pinIn4, int pinEna, int pinEnb)
  :in1(pinIn1), in2(pinIn2), in3(pinIn3), in4(pinIn4), ena(pinEna), enb(pinEnb){
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
    pinMode(ena, OUTPUT);
    pinMode(enb, OUTPUT);
    analogWrite(ena, defaultSpeed);
    analogWrite(enb, defaultSpeed);
  }

  void move(int side, int how){
    if(side & MOTOR_LEFT){
      if(how == MOTOR_FORWARD){
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
      } else if(how == MOTOR_BACK) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
      } else if(how == MOTOR_STOP){
        digitalWrite(in1, HIGH);
        digitalWrite(in2, HIGH);
      }
    }
    if(side & MOTOR_RIGHT){
      if(how == MOTOR_FORWARD){
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
      } else if(how == MOTOR_BACK) {
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
      } else if(how == MOTOR_STOP){
        digitalWrite(in3, HIGH);
        digitalWrite(in4, HIGH);
      }
    }
  }

  void speed(int side, unsigned char speed){
    if(side & MOTOR_LEFT){
      analogWrite(ena, speed);
    }
    if(side & MOTOR_RIGHT){
      analogWrite(enb, speed);
    }
  }
};

class TimeManager(){
    void doAfter(int time){
        
    }
} TimeManager;

class Button{
    #define UNPUSHED 0
    #define PUSHED 1
    #define COMPLETED 2

    int pin;
    unsigned long pushTime;
    unsigned long duringTime;
    int pushFlag;

    public:
    Button(int pin, unsigned long duringTime=ULONG_MAX):pin(pin), pushTime(0), duringTime(duringTime), pushFlag(UNPUSHED){
        pinMode(pin, INPUT);
    }
    protected:
    virtual bool check(){
        return digitalRead(pin) == HIGH;
    }
    virtual void onDeepPushed(){ }
    virtual void onShortPushed(){ }
    public:
    virtual void update(){
        if(check()){ //누른 상태
            if(pushFlag == UNPUSHED){ //처음 누른 순간
                pushTime = millis(); //시간 기록
                pushFlag = PUSHED;
            }
            else if(pushFlag == PUSHED && millis() - pushTime > duringTime){ //정해진 시간이 지났다면
                onDeepPushed();
                pushFlag = COMPLETED;
            }
        }
        else{ //뗀 상태
            if(pushFlag == PUSHED){ 
                onShortPushed();
                pushFlag = UNPUSHED;
            }
            else if(pushFlag == COMPLETED){
                pushFlag = UNPUSHED;
            }
        }
    }
};

class EepromResetButton : public Button{
    LCD *lcd;
    protected:
    virtual void onDeepPushed(){
        //EEPROM 리셋 코드를 작성해야함
        lcd->print("eeprom", "reset complete", 5000);
    }
    virtual void onShortPushed(){
        lcd->print("short", "pushed", 2000);
    }
    public:
    EepromResetButton(int pinResetBtn, unsigned long duringTime, LCD *lcd)
    :Button(pinResetBtn, duringTime), lcd(lcd){}
};

class Car{
    int pinStartOut;
    int started;
    LCD *lcd;
    Motor *motor;
    public:
    Car(int pinStartOut, LCD *lcd, Motor *motor)
    : pinStartOut(pinStartOut), started(0), lcd(lcd), motor(motor){
        pinMode(pinStartOut, OUTPUT);
    }
    void turnOnLight(){
        digitalWrite(pinStartOut, HIGH);
        delay(100);
        digitalWrite(pinStartOut, LOW);
        delay(100);
        digitalWrite(pinStartOut, HIGH);
    }
    void turnOffLight(){
        digitalWrite(pinStartOut, LOW);
    }
    void on(){
        started = 1;
        lcd->print("Start Car...", 5000);
        turnOnLight();
        motor->move(MOTOR_LEFT | MOTOR_RIGHT, MOTOR_FORWARD);
    }
    void off(){
        started = 0;
        lcd->print("Engine off...", 5000);
        turnOffLight();
        motor->move(MOTOR_LEFT | MOTOR_RIGHT, MOTOR_STOP);
    }
    int isStarted(){ return started; }
    void update(){

    }
};

class StartButton : public Button{
    Car *car;
    private:
    void onDeepPushed(){
        if(!car->isStarted())
            car->on();
        else{

        }
    }
    void onShortPushed(){
        if(car->isStarted())
            car->off();
    }
    public:
    StartButton(int pinStartBtn, unsigned long duringTime, Car *car)
    :Button(pinStartBtn, duringTime), car(car){}
};



Motor *motor;
EepromResetButton *eepromResetBtn;
StartButton *startBtn;
Car *car;
LCD *lcd;

void setup() {
    Serial.begin(9600);
    lcd = new LCD();
    motor = new Motor(DEFAULT_SPEED, PIN_MOTOR_IN1, PIN_MOTOR_IN2, PIN_MOTOR_IN3, PIN_MOTOR_IN4, PIN_MOTOR_ENA, PIN_MOTOR_ENB);
    car = new Car(PIN_START_CAR_OUT, lcd,motor);

    eepromResetBtn = new EepromResetButton(PIN_EEPROM_RESET_BTN, 5000, lcd);
    startBtn = new StartButton(PIN_START_CAR_BTN, 2000, car);
    lcd->print("System on", 10000);
}

void loop() {
    eepromResetBtn->update();
    startBtn->update();
    lcd->update();
}

