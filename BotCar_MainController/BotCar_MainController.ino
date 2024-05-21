#include <avr/interrupt.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WDT.h>
#include <SPI.h>
#include <RF24.h>

#define SCREEN_WIDTH 128 // OLED 显示宽度，以像素为单位
#define SCREEN_HEIGHT 64 // OLED 显示高度，以像素为单位
#define OLED_I2C_ADDRESS 0x3C

// OLED 的 RESET 引脚，如果不需要可以设置为 -1
#define OLED_RESET     -1
#define RADIO_CHANNEL 7
#define RADIO_IRQ 3

#define REARWHEEL_HALL_PIN 2

#define JOYSTICK_X A1
#define JOYSTICK_Y A0
#define JOYSTICK_SW 4

#define RADIO_CMD_DELAY 20
// 创建一个 OLED 显示对象
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
RF24 radio(9, 10); // CE, CSN

#define RADIO_ADDRESS { '0', '0', '0', '1', '0', '1' }


void setup() { 
  if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  display.clearDisplay(); // 清除显示缓存  
  display.setTextSize(1); // 文本大小
  display.setTextColor(SSD1306_WHITE); // 文本颜色
  display.setCursor(10, 28); // 文本起始位置

  // // 显示文本
  display.println("Controller Ready.");
  display.display();
  
  const byte radioAddress[] = RADIO_ADDRESS;
  radio.begin();
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(RADIO_CHANNEL);
  radio.openWritingPipe(radioAddress);  
  radio.setDataRate(RF24_250KBPS);
  radio.enableAckPayload();  
  radio.stopListening();
  radio.setCRCLength(RF24_CRC_8);

  pinMode(REARWHEEL_HALL_PIN, INPUT_PULLUP); // 设置引脚为输入并启用内部上拉电阻
  attachInterrupt(digitalPinToInterrupt(REARWHEEL_HALL_PIN), handleWheelInterrupt, FALLING); // 当引脚从高电平变为低电平时触发中断

  pinMode(JOYSTICK_X, INPUT);
  pinMode(JOYSTICK_Y, INPUT);
  pinMode(JOYSTICK_SW, INPUT_PULLUP);
  // 设置定时器中断
  cli(); // 关闭中断
  TCCR1A = 0; // 清除控制寄存器A
  TCCR1B = 0; // 清除控制寄存器B
  TCNT1 = 0; // 初始化计数器值

  // 设置比较匹配寄存器，使其在 1000 Hz 下中断
  OCR1A = 781;// = 16MHz / (1024 * 20Hz) - 1 (必需计算的值)

  // CTC 模式
  TCCR1B |= (1 << WGM12);
  // 1024 分频器
  TCCR1B |= (1 << CS12) | (1 << CS10);
  // 允许比较匹配中断
  TIMSK1 |= (1 << OCIE1A);
  sei(); // 允许中断
  
  pinMode(JOYSTICK_SW, INPUT); 
  // 使能PCINT2中断，这适用于引脚8到14
  PCICR |= (1 << PCIE2);
  // 使能PCINT20（对应引脚4）
  PCMSK2 |= (1 << PCINT20);

  pinMode(RADIO_IRQ, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(RADIO_IRQ), handleRadioInterrupt, FALLING);

  Serial.begin(9600);
  Serial.println("Controller Ready.");  

  wdt_enable(WTO_2S);
}

// 处理轮胎转动里程
volatile long wheelCounter = 0; // 计数器变量，使用volatile以便在中断中使用
void handleWheelInterrupt() {
  wheelCounter++; // 计数器加1  
}

volatile char recv_status[32] = "";    
void handleRadioInterrupt() {    
  bool tx_ds, tx_df, rx_dr;
  radio.whatHappened(tx_ds, tx_df, rx_dr); // resets the IRQ pin to HIGH
  digitalWrite(RADIO_IRQ, HIGH);    
}

// 处理游戏手柄了代码
volatile int joystickX = 0;
volatile int joystickY = 0;
volatile bool joystickChanged = false;
volatile bool joystickButtonChanged = false;

volatile int deadZoneMin = 16;
volatile int deadZoneMax = 21;
volatile int joystickIdle = 19;
volatile int statusTimerCounter = 0;
volatile bool needFetchStatus = true;
ISR(TIMER1_COMPA_vect) {  
  int currentX = analogRead(JOYSTICK_X) / 100;
  int currentY = analogRead(JOYSTICK_Y) / 100;  
  // 设置中间死区位 16-18
  if (currentX >= deadZoneMin && currentX <= deadZoneMax ) currentX = joystickIdle;
  if (currentY >= deadZoneMin && currentY <= deadZoneMax) currentY = joystickIdle;

  currentX = 0 - (currentX - joystickIdle);
  currentY = 0 - (currentY - joystickIdle);
  if (currentX != joystickX || currentY != joystickY) {
      joystickX = currentX;
      joystickY = currentY;      
      joystickChanged = true;
  }

  statusTimerCounter++;
  if (statusTimerCounter >= 5) {
    needFetchStatus = true;
    statusTimerCounter = 0;
  }
}

unsigned long lastSignalTime = 0;
void handleSignleStatus() {
  if (!lastSignalTime) return;  // if never had connection, then ignore it 
  unsigned long sinceLastSignal = millis() - lastSignalTime;
  if (sinceLastSignal >= 1000) { 
    displayLostSignal();
  }
}

char messageToSend[32];
char ackMessage[32];
void loop() {
  handleSignleStatus();
  if (joystickChanged) {    
    //send the message to nrf24    
    sprintf(messageToSend, "M %+03d %+03d", joystickX, joystickY);
    radio.write(&messageToSend, sizeof(messageToSend), 1);
    delay(RADIO_CMD_DELAY);
    if ((joystickX == joystickY) && (joystickX = 0)) {      
      radio.write(&messageToSend, sizeof(messageToSend), 1);
      delay(RADIO_CMD_DELAY);
    }    
    Serial.println(String(messageToSend));   
    joystickChanged = false;
    delay(RADIO_CMD_DELAY);
  }  
  if (needFetchStatus) {    
    sprintf(messageToSend, "S");
    radio.write(&messageToSend, sizeof(messageToSend), 1);    
    needFetchStatus = false;
    delay(RADIO_CMD_DELAY);    
  }
  if (radio.available()) {
    lastSignalTime = millis();
    char status[32];
    radio.read(status, sizeof(status));    
    displayBotStatus(status);    
    delay(RADIO_CMD_DELAY);
  }    
  if (joystickButtonChanged) {        
    bool buttonState = !digitalRead(JOYSTICK_SW);
    sprintf(messageToSend, "H %01d", buttonState);
    radio.write(&messageToSend, sizeof(messageToSend));
    joystickButtonChanged = false;
    delay(RADIO_CMD_DELAY);
    radio.write(&messageToSend, sizeof(messageToSend)); //send twice
    delay(RADIO_CMD_DELAY);
    Serial.println(String(messageToSend));    
  }
  wdt_reset();
}


ISR(PCINT2_vect) {        
  volatile static bool lastButtonState = HIGH;
  bool currentButtonState = digitalRead(JOYSTICK_SW);
  if (currentButtonState != lastButtonState){
    joystickButtonChanged = true;
    lastButtonState = currentButtonState;
  }
}

void displayLostSignal() {
  display.clearDisplay();  
  display.setTextColor(SSD1306_WHITE); // 文本颜色
  display.setTextSize(1); // 文本大小
  display.setCursor(10, 28); // 文本起始位置
  display.print("Connection Lost!");
  display.display();
}

void displayBotStatus(const char* status) {
  float wheelDiameter = 6.0;
  float wheelCircumference = 18.84954;
  int circlePulses = 3;
  float pulsesDistance = wheelCircumference / circlePulses;
  // parse data  
  int battery, steeringAngle, currentWheelSpeed;
  unsigned int pulseCount, totalPulseCount;  
  sscanf(status, "%d %d %d %u %u", 
    &battery, &steeringAngle, &currentWheelSpeed, &pulseCount, &totalPulseCount);

  //draw UI
  display.clearDisplay();  
  display.setTextColor(SSD1306_WHITE); // 文本颜色

  int steeringMarginTop = 4;
  int steering = (0-steeringAngle) + 51;
  display.setTextSize(1); // 文本大小
  display.setCursor(0, 0 + steeringMarginTop); // 文本起始位置
  display.print("STR: ");
  display.drawFastHLine(26, 4 + steeringMarginTop, 101, SSD1306_WHITE);
  display.drawFastVLine(26, 0 + steeringMarginTop, 8, SSD1306_WHITE);
  display.drawFastVLine(26 + 101, 0 + steeringMarginTop, 8, SSD1306_WHITE);
  if (steeringAngle<0) {
    display.fillRoundRect(26 + 51, 0 + steeringMarginTop, abs(steeringAngle), 8, 0, SSD1306_WHITE);
    display.drawFastVLine(26 + 51 + abs(steeringAngle), 0 + steeringMarginTop -1, 10, SSD1306_WHITE);
  } else if (steeringAngle>0) {
    display.fillRoundRect(26 + 51 - abs(steeringAngle), 0 + steeringMarginTop, abs(steeringAngle), 8, 0,SSD1306_WHITE);
    display.drawFastVLine(26 + 51 - abs(steeringAngle), 0 + steeringMarginTop -1, 10, SSD1306_WHITE);
  } 
  display.drawFastVLine(26 + 51, 0 + steeringMarginTop , 8, SSD1306_WHITE);

  String displaySpeed = String(((float)currentWheelSpeed  * pulsesDistance)/100 * 0.06);
  int speedMarginTop = 20;
  display.setTextSize(1); // 文本大小
  display.setCursor(0, 3 + speedMarginTop); // 文本起始位置
  display.print("SPD:");
  display.setCursor(28 + 4 + displaySpeed.length() * 12, 0 + speedMarginTop); // 文本起始位置
  display.print("km");
  display.setCursor(28 + 4 + displaySpeed.length() * 12, 6 + speedMarginTop); // 文本起始位置
  display.print("/hr");
  display.setTextSize(2); // 文本大小
  display.setCursor(28, 0 + speedMarginTop); // 文本起始位置  
  display.print(displaySpeed.c_str());

  int batteryMarginTop = 42;
  display.setTextSize(1); // 文本大小
  display.setCursor(0, batteryMarginTop); // 文本起始位置
  display.print("BAT: "+ String((float)battery/100) +"v");
  
  int tripMarginTop = 56;
  display.setCursor(74, batteryMarginTop); // 文本起始位置
  display.print("TOTAL: ");

  display.setCursor(0, tripMarginTop); // 文本起始位置
  display.print("TRP: "+ String(((float)pulseCount * pulsesDistance)/100) +"m");
  display.setCursor(70, tripMarginTop); // 文本起始位置
  display.print(" "+ String(((float)totalPulseCount * pulsesDistance)/100,1) +"m");
  display.display();
}

