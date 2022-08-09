#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <Servo.h> 
#include <EEPROM.h>
#include <SoftwareSerial.h>
#include <string.h>
#include <limits.h>

//eeprom에 쓸 데이터 크기(바이트 단위)
#define CAR_ID_LENGTH 64

//모터 스피드 세팅. 실질적으로 130~255 사이의 값을 줘야 움직임
#define DEFAULT_SPEED 255
#define HIGH_SPEED 255
#define LOW_SPEED 255

//블루투스 프로토콜 헤더
#define DEFAULT_HEADER_LENGTH 2
#define MAX_INFORMATION_LENGTH 128
const unsigned char defaultHeader[DEFAULT_HEADER_LENGTH] = {
    //'B', 'C'
    0xA6, 0x12
};

#define RS_CARCTL 25
#define R_REQON 57
#define R_START 59
#define R_REQBC 65
#define R_ASSIGN_ID 68
#define R_DELETE_ID 72
#define R_REQLOCK 74
#define R_LOCK 76

#define S_REQON_AVAIL 58
#define S_SEND_STATE 60
#define S_SEND_OFF 63
#define S_SUCBC 66
#define S_ASSIGN_ID_OK 69
#define S_DELETE_OK 73
#define S_REQLOCK_AVAIL 75

#define C_STOP 10
#define C_REQ 105
#define C_REQACK 106
#define C_OPEN_TRUNK 103
#define C_CLOSE_TRUNK 104
#define C_OPEN_DOOR 101
#define C_CLOSE_DOOR 102
#define C_CAR_OFF 100


//핀 세팅
#define PIN_BLUETOOTH_RX A2
#define PIN_BLUETOOTH_TX A3
#define PIN_EEPROM_RESET_BTN 3
#define PIN_START_CAR_BTN 4
#define PIN_MOTOR_IN1 7
#define PIN_MOTOR_IN2 8
#define PIN_MOTOR_IN3 12
#define PIN_MOTOR_IN4 13
#define PIN_MOTOR_ENA 5
#define PIN_MOTOR_ENB 6

#define PIN_DOOR_SERVO 9
#define PIN_TRUNK_SERVO 10
#define PIN_START_CAR_OUT 11

class Rom {
    #define ROM_SET_CAR_ID 1
    #define ROM_NEW_DEVICE 128
public:
    void begin(){
        if(EEPROM.read(0) & ROM_NEW_DEVICE)
            EEPROM.write(0, 0);
    }
    unsigned char getRomState(){
        return EEPROM.read(0);
    }
    void getCarID(unsigned char *dst){
        for(int i = 0 ; i < CAR_ID_LENGTH ; i++)
            dst[i] = EEPROM.read(1 + i);
    }
    void updateCarId(unsigned char* data){
        EEPROM.update(0, EEPROM.read(0) | ROM_SET_CAR_ID);
        for(int i = 0 ; i < CAR_ID_LENGTH ; i++)
            EEPROM.update(1+i, data[i]);
    }
    void format(){
        EEPROM.update(0, 0);
    }
} rom;

class Reserver {
    /*
        일정 시간 이후에 작업을 수행하도록 할 때 사용하는 클래스
        인스턴스는 reserver이며 아두이노의 loop에서 매번 update를 수행해준다.
        이 때 예약시간이 된 루틴이 실행된다.
        delay 대신 이것을 사용하자.
    */
private:
    //람다 함수를 저장하기 위한 베이스 클래스
    struct CallableBase {
        virtual void operator()() = 0;
        virtual ~CallableBase() {}
    };
    //람다 함수를 저장하기 위한 derived 탬플릿 클래스
    template <typename F>
    struct Callable : CallableBase {
        F functor;
        Callable(F functor) : functor(functor) {}
        virtual void operator()() { return functor(); }
    };
    //람다 함수와 예약 시간 정보를 저장하는 데이터 클래스
    struct Data {
        CallableBase* func;
        long long startTime;
        long long term;
        Data(CallableBase* func, unsigned long term)
            :func(func), startTime(millis()), term(term) {}
        ~Data() {
            delete func;
        }
    };
    //링크드 리스트 노드
    struct Node {
        Data* data;
        Node* prev;
        Node* next;
        Node(Data* data, Node* prev, Node* next)
            :data(data), prev(prev), next(next) {}
        Node()
            :data(NULL), prev(NULL), next(NULL) {}
        ~Node() {
            if (data != NULL)
                delete data;
        }
    };
    Node* head;
    Node* tail;
public:
    Reserver()
        :head(new Node()), tail(new Node()) {
        head->prev = head;
        head->next = tail;
        tail->prev = head;
        tail->next = tail;
    }
    ~Reserver() {
        Node* cur = head, * del;
        while (cur != tail) {
            del = cur;
            cur = cur->next;
            delete del;
        }
        delete tail;
    }

    //void()형 람다 함수를 func에 넘겨서 실행한다.
    template <typename T>
    void doAfter(T func, unsigned long term) {
        Node* newNode = new Node(new Data(new Callable<T>(func), term), head, head->next);
        head->next->prev = newNode;
        head->next = newNode;
    }

    //loop 함수에서 실행한다.
    void update() {
        unsigned long nowTime = millis();
        Node* curNode = head->next;
        Node* del;
        while (curNode != tail) {
            if ((curNode->data->startTime + curNode->data->term) <= nowTime) {
                (*(curNode->data->func))();
                curNode->prev->next = curNode->next;
                curNode->next->prev = curNode->prev;
                del = curNode;
                curNode = curNode->next;
                delete del;
            }
            else
                curNode = curNode->next;
        }
    }
} reserver;

class Repeater {
    /*
        Reserver에 의해 반복 호출될 작업을 수행하는 클래스
    */
private:
    bool _isStarted; 
    unsigned long term; // 반복 주기
    int count; // onRepeat에 전달되는 start 이후에 onRepeat을 호출한 횟수
    //반복 함수. 람다 함수는 friend 처리되므로 private
    void repeat() {
        if (!isStarted())
            return;
        onRepeat(count++);
        reserver.doAfter([&] {
            this->repeat();
            }, term);
    }
protected:
    //콜백 함수
    virtual void onStop() {}
    virtual void onStart() {}
    virtual void onRepeat(int count)=0;

public:
    Repeater(unsigned long term) :_isStarted(false), term(term), count(0) { }
    bool isStarted() { return _isStarted; }
    void stop() {
        if (isStarted()) {
            _isStarted = false;
            onStop();
        }
    }
    void start() {
        if (!isStarted()) {
            _isStarted = true;
            count = 0;
            onStart();
            repeat();
        }
    }
};

class LCD{
private:
    LiquidCrystal_I2C* i2c;
    int flag;
    unsigned long startTime;
    unsigned long printingTime;
    char defaultMessage[2][17];
    const char blank[17] = "                ";
    void clear(){
        i2c->setCursor(0,0); //첫번째 줄의 0번째 부터 출력
        i2c->print(blank);
        i2c->setCursor(0,1); //첫번째 줄의 0번째 부터 출력
        i2c->print(blank);
    }
    void rollback(){
        i2c->setCursor(0,0); //첫번째 줄의 0번째 부터 출력
        i2c->print(defaultMessage[0]);
        i2c->setCursor(0,1); //첫번째 줄의 0번째 부터 출력
        i2c->print(defaultMessage[1]);
    }
public:
    LCD():i2c(NULL), flag(0), startTime(0), printingTime(0){
        strncpy(defaultMessage[0], blank, sizeof(defaultMessage[0]));
        strncpy(defaultMessage[1], blank, sizeof(defaultMessage[1]));
    }
    ~LCD(){
        if (i2c != NULL)
            delete i2c;
    }
    void begin(){
        if(i2c != NULL)
            delete i2c;
        
        i2c = new LiquidCrystal_I2C(0x27, 16, 2);
        i2c->begin();
    }
    void setDefaultMessage(const char *line1, const char *line2){
        strncpy(defaultMessage[0], line1, sizeof(defaultMessage[0]));
        strncpy(defaultMessage[1], line2, sizeof(defaultMessage[1]));
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
            i2c->setCursor(0,0); //첫번째 줄의 0번째 부터 출력
            i2c->print(line1);
        }
        if(line2 != NULL){
            i2c->setCursor(0,1); //두번째 줄의 0번째 부터 력출
            i2c->print(line2);
        }
        strncpy(defaultMessage[0], line1, sizeof(defaultMessage[0]));
        strncpy(defaultMessage[1], line2, sizeof(defaultMessage[1]));
    } 
    void print(const char *line1, const char *line2, unsigned int time){
        clear();
        flag = 1;
        startTime = millis();
        printingTime = time;
        if(line1 != NULL){
            i2c->setCursor(0,0); //첫번째 줄의 0번째 부터 출력
            i2c->print(line1);
        }
        if(line2 != NULL){
            i2c->setCursor(0,1); //두번째 줄의 0번째 부터 력출
            i2c->print(line2);
        }
    }
    void print(const char *line1){
        print(line1, "");
    }
    void print(const char *line1, unsigned time){
        print(line1, "", time);
    }
} lcd;

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
  Motor():in1(-1), in2(-1), in3(-1), in4(-1), ena(-1), enb(-1){ }

  void attach(unsigned char defaultSpeed, int pinIn1, int pinIn2, int pinIn3, int pinIn4, int pinEna, int pinEnb){
    in1 = pinIn1;
    in2 = pinIn2;
    in3 = pinIn3;
    in4 = pinIn4;
    ena = pinEna;
    enb = pinEnb;
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
    if(side & MOTOR_RIGHT){
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

class Door{
    int pin;
    Servo servo;
    int openingAngle;
    int closingAngle;
    bool opened;
    public:
    Door(int openingAngle, int closingAngle)
    :openingAngle(openingAngle), closingAngle(closingAngle), opened(false) {}
    void attach(int pin){
        this->pin = pin;
        servo.attach(pin);
    }
    void open(){
        opened=true;
        servo.write(openingAngle);
    }
    void close(){
        opened=false;
        servo.write(closingAngle);
    }
    bool isOpened(){
        return opened;
    }
};


class Button{
private:
    #define UNPUSHED 0
    #define PUSHED 1
    #define COMPLETED 2

    int pin;
    unsigned long pushedTime;
    unsigned long duringTime;
    int pushFlag;
    
    bool check(){
        return digitalRead(pin) == HIGH;
    }

public:
    Button():pin(-1), pushedTime(0), duringTime(ULONG_MAX), pushFlag(UNPUSHED){}
    virtual ~Button(){}
    void attach(int pin, unsigned long duringTime=ULONG_MAX){
        this->pin = pin;
        this->duringTime = duringTime;
        pinMode(pin, INPUT);
    }
    void update(){
        if(check()){ //누른 상태
            if(pushFlag == UNPUSHED){ //처음 누른 순간
                pushedTime = millis(); //시간 기록
                pushFlag = PUSHED;
            }
            else if(pushFlag == PUSHED && millis() - pushedTime > duringTime){ //정해진 시간이 지났다면
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
    
protected:
    virtual void onDeepPushed() { }
    virtual void onShortPushed() { }
};

class EepromResetButton : public Button{
protected:
    virtual void onDeepPushed() override{
        rom.format();
        lcd.print("eeprom", "reset complete", 5000);
    }
    virtual void onShortPushed() override{
        if(rom.getRomState() & ROM_SET_CAR_ID)
            lcd.print("ID assigned!", "", 2000);
        else
            lcd.print("ID not assigned", "...", 2000);
    }
};

class Bluetooth;

class Car{
private:
    int pinStartOut;
    bool started;
    Motor *motor;
    Door *door, *trunk;
    Bluetooth *bluetooth;
    
    void lightOnStart(){
        digitalWrite(pinStartOut, HIGH);
        delay(100);
        digitalWrite(pinStartOut, LOW);
        delay(100);
        digitalWrite(pinStartOut, HIGH);
    }

    void turnOnLight(){
        digitalWrite(pinStartOut, HIGH);
    }
    void turnOffLight(){
        digitalWrite(pinStartOut, LOW);
    }


public:
    Car()
    : pinStartOut(-1), started(false), motor(NULL), door(NULL), trunk(NULL), bluetooth(NULL){ }

    void attach(int pinStartOut, Motor *motor, Door *door, Door *trunk, Bluetooth *bluetooth){
        this->door = door;
        this->trunk = trunk;
        this->pinStartOut = pinStartOut;
        this->motor = motor;
        this->bluetooth = bluetooth;
        pinMode(pinStartOut, OUTPUT);
    }

    void on(){
        started = true; // 시동 상태 기록
        lcd.print("Start Car...", 5000);
        lightOnStart();
    }

    void off(){
        started = false; // 시동 상태 기록
        lcd.print("Engine off...", 5000);
        turnOffLight();
    }

    enum class Speed{
        STOP, SLOW, FAST
    };

    enum class Direction{
        FORWARD_LEFT, FORWARD_RIGHT, FORWARD_STRAIGHT,
        BACK_LEFT, BACK_RIGHT, BACK_STRAIGHT, STOP
    };

    bool steer(Direction direction, Speed speed){
        if(started){
            if(speed==Speed::STOP || direction == Direction::STOP){
                motor->move(MOTOR_LEFT | MOTOR_RIGHT, MOTOR_STOP);
                return true;
            }
            else if(speed == Speed::SLOW)
                motor->speed(MOTOR_LEFT | MOTOR_RIGHT, LOW_SPEED);
            else if(speed == Speed::FAST)
                motor->speed(MOTOR_LEFT | MOTOR_RIGHT, HIGH_SPEED);
            else
                return false;
            if(direction == Direction::FORWARD_LEFT){
                motor->move(MOTOR_LEFT, MOTOR_STOP);
                motor->move(MOTOR_RIGHT, MOTOR_FORWARD);
            }
            else if(direction == Direction::FORWARD_RIGHT){
                motor->move(MOTOR_LEFT, MOTOR_FORWARD);
                motor->move(MOTOR_RIGHT, MOTOR_STOP);
            }
            else if(direction == Direction::FORWARD_STRAIGHT){
                motor->move(MOTOR_LEFT, MOTOR_FORWARD);
                motor->move(MOTOR_RIGHT, MOTOR_FORWARD);
            }
            else if(direction == Direction::BACK_LEFT){
                motor->move(MOTOR_LEFT, MOTOR_STOP);
                motor->move(MOTOR_RIGHT, MOTOR_BACK);
            }
            else if(direction == Direction::BACK_RIGHT){
                motor->move(MOTOR_LEFT, MOTOR_BACK);
                motor->move(MOTOR_RIGHT, MOTOR_STOP);
            }
            else if(direction == Direction::BACK_STRAIGHT){
                motor->move(MOTOR_LEFT, MOTOR_BACK);
                motor->move(MOTOR_RIGHT, MOTOR_BACK);
            }
            else
                return false;
            return true;
        }
        return false;
    }

    enum class LockingState{
        OPEN, CLOSE
    };

    void controlDoor(LockingState open){
        if(open == LockingState::OPEN){
            door->open();
        }
        else if(open == LockingState::CLOSE){
            door->close();
        }
    }

    LockingState getDoorState(){
        if(door->isOpened())
            return LockingState::OPEN;
        else
            return LockingState::CLOSE;
    }

    void controlTrunk(LockingState open){
        if(open == LockingState::OPEN){
            trunk->open();
        }
        else if(open == LockingState::CLOSE){
            
            trunk->close();
        }
    }

    LockingState getTrunkState(){
        if(trunk->isOpened())
            return LockingState::OPEN;
        else
            return LockingState::CLOSE;
    }

    int isStarted(){ return started; }
        
};

class StartButton : public Button{
private:
    Car *car;
    unsigned long doorTime = 0UL;
protected:
    void onDeepPushed(){
        if(!car->isStarted())
            car->on();
        else{
            doorTime = millis();
            car->off();
        }
    }
    void onShortPushed(){
        if(car->isStarted()){
            doorTime = millis();
            car->off();
        }
        else{
            if(doorTime + 300 > millis()){
                return;
            }
            doorTime = millis();
            if(car->getDoorState() == Car::LockingState::OPEN){
                lcd.print("Close the door.", 3000);
                car->controlDoor(Car::LockingState::CLOSE);
                car->controlTrunk(Car::LockingState::CLOSE);
            }
            else{
                lcd.print("Open the door.", 3000);
                car->controlDoor(Car::LockingState::OPEN);
                car->controlTrunk(Car::LockingState::OPEN);
            }
        }
    }
public:
    void setCar(Car *car){
        this->car = car;
    }
};

class BluetoothSerial{
private:
    SoftwareSerial *bluetoothSerial;
public:
    BluetoothSerial()
    :bluetoothSerial(NULL)
    {}
    ~BluetoothSerial(){
        if(bluetoothSerial != NULL)
            delete bluetoothSerial;
    }
    void begin(int tx, int rx, int frequency){
        bluetoothSerial = new SoftwareSerial(tx, rx);
        bluetoothSerial->begin(frequency);
    }
    int available(){
        return bluetoothSerial->available();
    }
    int read(){
        int data = bluetoothSerial->read();
        Serial.print("bt-read:");
        Serial.println(data);
        return data;
    }
    int write(unsigned byte){
        int cnt = bluetoothSerial->write(byte);
        Serial.print("bt-write:");
        Serial.println(byte);
        return cnt;
    }
};

class Bluetooth{
private:
    BluetoothSerial btSerial;
    Car *car;
    bool remoteControlMode;
    int interpret();
    void car_control(unsigned char code);
public:
    Bluetooth():btSerial(), car(NULL), remoteControlMode(false) {}
    void begin(int pinTx, int pinRx, Car *car){
        btSerial.begin(pinTx, pinRx, 9600);
        this->car = car;
    }
    void sendData(unsigned char headerCode, unsigned char *dataArray=NULL);
    void receiveData(){
        if(btSerial.available()){
            unsigned char data;
            int i = 0;
            for(; i < DEFAULT_HEADER_LENGTH ; i++){
                data = btSerial.read();
                if(data != defaultHeader[i]) //잘못된 데이터는 버린다.
                    return;
            }
            if(i < DEFAULT_HEADER_LENGTH)
                return;
            //올바른 데이터인 경우
            interpret();
        }
    }
};

void Bluetooth::car_control(unsigned char code){
    if(!remoteControlMode)
        return;
    Car::Speed speed;
    Car::Direction direction;
    switch(code){
        case C_STOP:
            car->steer(Car::Direction::STOP, Car::Speed::STOP);
            break;
        case C_CAR_OFF:
            car->off();
            break;
        case C_OPEN_DOOR:
            if(car->getDoorState() != Car::LockingState::OPEN)
                car->controlDoor(Car::LockingState::OPEN);
            break;
        case C_CLOSE_DOOR:
            if(car->getDoorState() != Car::LockingState::CLOSE)
                car->controlDoor(Car::LockingState::CLOSE);
            break;
        case C_OPEN_TRUNK:
            if(car->getTrunkState() != Car::LockingState::OPEN)
                car->controlTrunk(Car::LockingState::OPEN);
            break;
        case C_CLOSE_TRUNK:
            if(car->getTrunkState() != Car::LockingState::CLOSE)
                car->controlTrunk(Car::LockingState::CLOSE);
            break;
        default:
            switch(code/10){
                case 1:
                    car->steer(Car::Direction::STOP, Car::Speed::STOP);
                    return;
                case 2:
                    speed = Car::Speed::SLOW;
                    break;
                case 3:
                    speed = Car::Speed::FAST;
                    break;
                default:
                    return;
            }
            switch(code % 10){
                case 1:
                    direction = Car::Direction::FORWARD_RIGHT;
                    break;
                case 2:
                    direction = Car::Direction::FORWARD_STRAIGHT;
                    break;
                case 3:
                    direction = Car::Direction::FORWARD_LEFT;
                    break;
                case 5:
                    direction = Car::Direction::BACK_LEFT;
                    break;
                case 6:
                    direction = Car::Direction::BACK_STRAIGHT;
                    break;
                case 7:
                    direction = Car::Direction::BACK_RIGHT;
                    break;
                default:
                    return;
            }
            car->steer(direction, speed);
            break;
    }
}

int Bluetooth::interpret(){
    unsigned char headerNumber = btSerial.read();
    unsigned char next;
    switch(headerNumber){
        case RS_CARCTL:
            next = btSerial.read();
            if(next == C_REQ){
                next = C_REQACK;
                sendData(RS_CARCTL, &next);
                Serial.println("Connected.");
                lcd.print("remote control", "connected!", 5000);
                remoteControlMode = true;
            }
            else{
                car_control(next);
            }
			break;
        case R_REQON:
        //시동 요청
			break;
		case R_START:
        //시동 허가
			break;
		case R_REQBC:
        //블루투스 연결 요청
			break;
		case R_ASSIGN_ID:
        //자동차 ID 할당
			break;
		case R_DELETE_ID:
        //자동차 ID 제거 명령
			break;
        case R_REQLOCK:
        //락/언락 요청
			break;
        case R_LOCK:
        //락/언락 수행
			break;
        default:
            return -1;
    }
    return 0;
}

void Bluetooth::sendData(unsigned char headerCode, unsigned char *dataArray){
    int i;
    unsigned char data;
    int dataArrayLength = 0;
    switch(headerCode){
        case RS_CARCTL:
            dataArrayLength = 1;
			break;
        case S_REQON_AVAIL:
        //시동 요청이 올바른가
            break;
        case S_SEND_STATE:
        //시동상태 전송 요청
            break;
        case S_SEND_OFF:
        //시동꺼짐정보 전송 요청
            break;
        case S_SUCBC:
        //블루투스 연결 성공
            break;
        case S_ASSIGN_ID_OK:
        //자동차 ID 할당 성공
            break;
        case S_DELETE_OK:
        //자동차 ID 제거 성공
            break;
        case S_REQLOCK_AVAIL:
        //락/언락 요청이 올바른가
            break;
        default:
            return;
    }
    for(i = 0; i < DEFAULT_HEADER_LENGTH ; i++){
        btSerial.write(defaultHeader[i]);
    }
    btSerial.write(headerCode);
    for(i = 0 ; i < dataArrayLength ; i++){
        btSerial.write(dataArray[i]);
    }
}

Bluetooth bluetooth;
Motor motor;
EepromResetButton eepromResetBtn;
StartButton startBtn;
Car car;
Door door(0, 90), trunk(0, 90);

void setup() {
    rom.begin();
    Serial.begin(9600);
    lcd.begin();

    motor.attach(
        DEFAULT_SPEED, 
        PIN_MOTOR_IN1, PIN_MOTOR_IN2, PIN_MOTOR_IN3, PIN_MOTOR_IN4, 
        PIN_MOTOR_ENA, PIN_MOTOR_ENB
        );
    door.attach(PIN_DOOR_SERVO);
    trunk.attach(PIN_TRUNK_SERVO);
    car.attach(PIN_START_CAR_OUT, &motor, &door, &trunk, &bluetooth);

    eepromResetBtn.attach(PIN_EEPROM_RESET_BTN, 5000);
    startBtn.attach(PIN_START_CAR_BTN, 2000);
    startBtn.setCar(&car);
    bluetooth.begin(PIN_BLUETOOTH_RX, PIN_BLUETOOTH_TX, &car);
    
    lcd.setDefaultMessage("Carflix", "is now ready");
    lcd.print("Device Boot", "Complete!", 10000);
}

void loop() {
    eepromResetBtn.update();
    startBtn.update();
    lcd.update();
    reserver.update();
    bluetooth.receiveData();
    delay(15);
}

