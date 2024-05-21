#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WDT.h>
#include <PMU.h>
#include <SPI.h>
#include <RF24.h>

#define ANALOG_MAX 3072
#define SCREEN_WIDTH 128 // OLED 显示宽度，以像素为单位
#define SCREEN_HEIGHT 64 // OLED 显示高度，以像素为单位
#define OLED_I2C_ADDRESS 0x3C

// OLED 的 RESET 引脚，如果不需要可以设置为 -1
#define OLED_RESET     -1
#define RADIO_CHANNEL 7
#define RADIO_IRQ 3

#define HORN_PIN 7
#define HORN_TONE 230

#define POWER_VOLTAGE_PIN A0
#define POWER_VOLTAGE_RATIO 0.2424
#define WHEEL_SENSOR_PIN 2
#define STEERING_SENSOR_PIN A2
#define SIGNAL_PIN A1

#define LED_LEFT_PIN 5
#define LED_RIGHT_PIN 6
#define LED_FORWARD_PIN 10
#define LED_BACKWARD_PIN 9

#define FRONT_DISTANCE_TRIG_PIN A6
#define FRONT_DISTANCE_ECHO_PIN A7

#define EEPROM_ADDRESS 0x0
#define RADIO_ADDRESS { '0', '0', '0', '1', '0', '1' }

enum bot_power_t {
	BOT_SLEEPING = 0,
	BOT_WORKING = 1,
	BOT_GOING_SLEEP = 2,
	BOT_JUST_WAKEUP = 3
};

enum driving_direction_t {
  DRIVE_FORWARD = 1,
  DRIVE_BACKWARD = -1,
  STOPPED = 0,
};

enum steering_direction_t {
  TURNING_LEFT = 1,
  TURNING_RIGHT = -1,
  STRAIGHT = 0,
};

volatile unsigned long pulseCount = 0;
volatile unsigned long totalPulseCount = 0;
unsigned long lastPulseCount = 0;
float wheelDiameter = 6.0;
float wheelCircumference = 18.84954;
int circlePulses = 3;
float currentRPM = 0;
unsigned long lastTime = 0;

steering_direction_t steeringState = STRAIGHT;
driving_direction_t drivingState = STOPPED;

// 创建一个 OLED 显示对象
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
RF24 radio(4, 8); // CE, CSN

void setup() { 
  display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS);
  display.clearDisplay(); // 清除显示缓存  
  display.setTextSize(1); // 文本大小
  display.setTextColor(SSD1306_WHITE); // 文本颜色
  display.setCursor(2, 28); // 文本起始位置
  
  const byte radioAddress[] = RADIO_ADDRESS;
  radio.begin();
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(RADIO_CHANNEL);
  radio.openReadingPipe(1, radioAddress);
  radio.setDataRate(RF24_250KBPS);
  radio.enableAckPayload();  
  radio.startListening();  
  radio.setCRCLength(RF24_CRC_8);
  
  cli(); // 关闭中断
  // setup PWM for pin 9 and 10 to 8bits
  // // 设置Timer1
  TCCR1A = 0; // 将寄存器设置为0
  TCCR1B = 0; // 将寄存器设置为0
  TCNT1 = 0; // 将计数器重置为0

  // // 设置为模式1（相位正确PWM）
  TCCR1A |= (1 << WGM10);
  // TCCR1A |= (1 << COM1A1);

  // 设置非反相PWM输出模式  
  TCCR1A |= (1 << COM1A1) | (1 << COM1B1);
  // // 设置预分频为64 (从而获得~490 Hz的频率在16MHz Arduino上)
  TCCR1B |= (1 << CS11) | (1 << CS10);
  sei();

  // display heartbeat signal  
  pinMode(SIGNAL_PIN, OUTPUT);
  analogWrite(SIGNAL_PIN, 0);

  pinMode(LED_LEFT_PIN, OUTPUT);
  pinMode(LED_RIGHT_PIN, OUTPUT);
  pinMode(LED_FORWARD_PIN, OUTPUT);
  pinMode(LED_BACKWARD_PIN, OUTPUT);
  analogWrite(LED_LEFT_PIN, 0);
  analogWrite(LED_RIGHT_PIN, 0);
  analogWrite(LED_FORWARD_PIN, 0);
  analogWrite(LED_BACKWARD_PIN, 0);

  pinMode(HORN_PIN, OUTPUT);
  pinMode(STEERING_SENSOR_PIN, INPUT);
  pinMode(WHEEL_SENSOR_PIN, INPUT_PULLUP);
  pinMode(RADIO_IRQ, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(RADIO_IRQ), handleRadioInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(WHEEL_SENSOR_PIN), handleWheelPulses, FALLING);

  // setup for voltage reading
  analogReference(INTERNAL4V096); 
  analogReadResolution(12); 

  loadMileage();
  Serial.begin(9600);
  Serial.println("Bot Recevier Ready.");  
  display.clearDisplay();    
  display.println("Bot Recevier Ready.");
  display.display();    
  wdt_enable(WTO_2S);
}

void prepare_sleep() {
  digitalWrite(LED_LEFT_PIN, 0);
  digitalWrite(LED_RIGHT_PIN, 0);
  digitalWrite(LED_FORWARD_PIN, 0);
  digitalWrite(LED_BACKWARD_PIN, 0);
  digitalWrite(HORN_PIN, 0);
  digitalWrite(SIGNAL_PIN, 0);
  
  //disable INT0 not get wake up from wheel
  EIMSK &= ~(1 << INT0);   
}

void handleWheelPulses() {
  pulseCount++;
  totalPulseCount++;
}

// return distance in centimetre(cm)
long getFrontDistance() {
  digitalWrite(FRONT_DISTANCE_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(FRONT_DISTANCE_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(FRONT_DISTANCE_TRIG_PIN, LOW);
  long duration = pulseIn(FRONT_DISTANCE_ECHO_PIN, HIGH);  
  long distance = duration * 0.034 / 2;
  return distance;
}

float getCurrentWheelSpeed() {
  if (millis() - lastTime >= 500) {
    // 计算过去0.5秒的脉冲数
    unsigned long currentPulseCount = pulseCount;
    unsigned long pulses = currentPulseCount - lastPulseCount;
    lastPulseCount = currentPulseCount;
    // 计算每分钟转数（RPM），每转3个脉冲
    currentRPM = (pulses / circlePulses) * 120.0; // 因为是0.5秒，所以乘以120（2 * 60）
    // 记录当前时间
    lastTime = millis();
  }
  return currentRPM;
}

float getBatteryVoltage() {
  const float R1 = 51000.0; // 100K欧姆
  const float R2 = 16800.0; // 16K欧姆
  int sensorValue = analogRead(POWER_VOLTAGE_PIN);    
  float sensorVoltage = sensorValue * (4.096 / 4095.0);
  float voltageIn = sensorVoltage * ((R1 + R2) / R2);  
  return voltageIn;
}

int getSteeringAngle() {
  int v_min = 850, v_middle = 2400, v_max = 4000;
  int deadzone_min = 2200,  deadzone_max = 2600;
  float steeringValue = analogRead(STEERING_SENSOR_PIN);
  steeringValue = max(steeringValue, v_min);
  steeringValue = min(steeringValue, v_max);
  if (steeringValue >= deadzone_min && steeringValue <= deadzone_max) {
    steeringValue = v_middle;
  }
  float steerAngle = (steeringValue - v_min) / (v_max - v_min) - 0.5;  
  int intSteerAngle = int(steerAngle * 100);
  return intSteerAngle;
}

unsigned long lastSavedValue = 0;
void loadMileage() {
  totalPulseCount = EEPROM.read32(EEPROM_ADDRESS);    
  if (totalPulseCount == 0xFFFFFFFF) {
    totalPulseCount = 0;
    EEPROM.write32(EEPROM_ADDRESS, totalPulseCount);
  }
  lastSavedValue = totalPulseCount;  
}
void saveMileage(bool forceSave = false) {
  static unsigned long lastSaveTime = 0;
  if ((forceSave) ||
    (millis() - lastSaveTime >= 10000 && totalPulseCount - lastSavedValue >= 100)) {
    EEPROM.write32(EEPROM_ADDRESS, totalPulseCount); // 存储在地址 10 开始的地方
    Serial.println("total pulses saved.");
    lastSaveTime = millis();
    lastSavedValue = totalPulseCount;
  }
}

char botStatus[32] = "";
void fetchBotStatus() {
  float batteryVoltage = getBatteryVoltage();  
  float steeringAngle = getSteeringAngle();
  int currentWheelSpeed = getCurrentWheelSpeed();
  sprintf(botStatus, "%04d %+03d %d %d %d" , 
    int(batteryVoltage * 100), int(steeringAngle), int(currentWheelSpeed), int(pulseCount),int(totalPulseCount));   
}

volatile char recvCmd[32] = "";    
void handleRadioInterrupt() {
    bool tx_ds, tx_df, rx_dr;
    radio.whatHappened(tx_ds, tx_df, rx_dr); // resets the IRQ pin to HIGH
    if (rx_dr) {      
      radio.read(recvCmd, 32);      
    }    
}

unsigned long lastSignalTime = 0;
void handleSignleStatus() {
  unsigned long sinceLastSignal = millis() - lastSignalTime;
  if (sinceLastSignal >= 500) { 
    digitalWrite(SIGNAL_PIN, 0);
  }
}

bot_power_t bot_p = BOT_WORKING;
bool handleBotPowerStatus() {
  wdt_reset();  
  unsigned long sinceLastSignal = millis() - lastSignalTime;
  if ((sinceLastSignal >= 2460) && (bot_p == BOT_WORKING)) {    
    bot_p = BOT_GOING_SLEEP;
    prepare_sleep();
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(2,28);
    display.println("Going to sleep...");    
    display.display();     
    return true;
  }
  if ((sinceLastSignal >= 3500) && (bot_p == BOT_GOING_SLEEP)) {        
    wdt_disable(); 
    saveMileage(true);
    display.clearDisplay();
    display.display();      
    bot_p = BOT_SLEEPING;      
    Serial.println("going to sleep");
    PMU.sleep(PM_POFFS1, SLEEP_FOREVER);      
    return true;
  }
  if (bot_p == BOT_SLEEPING) {      
    wdt_enable(WDTO_15MS);
    while (1);        
  }
  if (bot_p == BOT_WORKING) return false; 
  return true;
}

void loop() {
  if (handleBotPowerStatus()) {     
    return; 
  }
  handleSignleStatus();
  if (recvCmd[0] != '\0') {        
    handleCommand(recvCmd);
    displayLog(recvCmd);
    strcpy(recvCmd, "\0");       
  }
  saveMileage();
}

#define MAX_LOGS 8
#define MAX_LOG_LEN 10
char logs[MAX_LOGS][MAX_LOG_LEN]; // Array to store log messages
void displayLog(const char *message) {
  if (strlen(message) == 1) return; 
  
  Serial.println(message);
  // Shift logs up
  for (int i = 0; i < MAX_LOGS - 1; i++) {
    strncpy(logs[i], logs[i + 1], MAX_LOG_LEN);
  }
  // Add new message at the bottom
  strncpy(logs[MAX_LOGS - 1], message, MAX_LOG_LEN);
  // logs[MAX_LOGS - 1] = message;

  // Clear the display
  display.clearDisplay();

  // Display all log messages
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  for (int i = 0; i < MAX_LOGS; i++) {      
    display.setCursor(0, i * (8)); // Assuming each line is 8 pixels tall
    display.print(logs[i]);      
  }
  display.display();
}


void handleCommand(const char *cmd) {  
  // 处理车辆移动命令
  if (cmd[0] == 'M') {
    int x, y;
    sscanf(cmd, "M %03d %03d", &x, &y);
    if (x < 0) {      
      analogWrite(LED_LEFT_PIN,  min(map(abs(x), 0, 18, 0, 255), 255));
      analogWrite(LED_RIGHT_PIN, 0);
      steeringState = TURNING_LEFT;
    }else if (x > 0 ){
      analogWrite(LED_LEFT_PIN,  0);
      analogWrite(LED_RIGHT_PIN, min(map(abs(x), 0, 18, 0, 255), 255));
      steeringState = TURNING_RIGHT;
    }else{
      analogWrite(LED_LEFT_PIN,  0);
      analogWrite(LED_RIGHT_PIN, 0);
      steeringState = STRAIGHT;
    }    
    if (y > 0) {      
      analogWrite(LED_FORWARD_PIN,  min(map(abs(y), 0, 18, 0, 255), 255));
      analogWrite(LED_BACKWARD_PIN, 0);
      drivingState = DRIVE_FORWARD;
    }else if (y < 0 ){
      analogWrite(LED_FORWARD_PIN,  0);
      analogWrite(LED_BACKWARD_PIN, min(map(abs(y), 0, 18, 0, 255), 255));      
      drivingState = DRIVE_BACKWARD;
    }else{
      analogWrite(LED_FORWARD_PIN,  0);
      analogWrite(LED_BACKWARD_PIN, 0);      
      drivingState = STOPPED;
    }    
  }
  // 处理喇叭命令
  else if (cmd[0]== 'H') {
    int state;
    sscanf(cmd, "H %01d", &state);    
    if (state == HIGH) {
      tone(HORN_PIN, HORN_TONE);
    } else {
      noTone(HORN_PIN);
    }        
  }
  else if (cmd[0]== 'S') {
    // report status    
    digitalWrite(SIGNAL_PIN, 1);
    fetchBotStatus();
    Serial.println(botStatus);
    radio.writeAckPayload(1, botStatus, sizeof(botStatus));
  }
  // reset the signal detector pin
  unsigned long sinceLastSignal = millis() - lastSignalTime;
  if (sinceLastSignal>=200){
    digitalWrite(SIGNAL_PIN, 1);
    fetchBotStatus();
    radio.writeAckPayload(1, botStatus, sizeof(botStatus));
  }
  lastSignalTime = millis();
}