
//
//   Пропеллер
//   Протасов, 26.11.2018
//   30.11.2018
//   17 12 2018
//   24 03 2019
//   06 04 2019
//    08.04.2019
#define VER "Protasov AA, 12.06.2019"

//Для управления
//#define ventPin A0  //выход 0 - вкл вентилятора  -- не нужен
#define ten0Pin A1  //выход 0 - вкл ТЭН0
#define ten1Pin A2  //выход 0 - вкл ТЭН1
#define ten2Pin A3  //выход 0 - вкл ТЭН2
#define alarmPin A7  //вход 0 - сработало аварийное отключение тэнов по температуре

//диммер
#define dimPin 4                           //выход на диммер
#define zeroPin 3                          //вход прерывания, сигнал прохождения нуля с диммера
#include <CyberLib.h>                      //шустрая библиотека для таймера, от алекса гайвера
const int ticDelay = 100;                  //период тика в микросекундах
const int DimmerMax = 100;       //максимальное значение задержки включения диммера в таках 0.01 сек - целый полупериод для 50Гц, вентилятор выключен
volatile int Dimmer = 50;         //мощность вентилятора 0..100 
volatile unsigned long tic = 0,
                       lastticDimmer = 0,
                       lastticSpeed = 0;
                                             
//датчик скорости вращения вентилятора
#define speedPin 2  //вход прерывания 1 для датчика лопастей вентилятора
const long VentPaddle = 2;               //количество лопастей вентилятора
const long minPeriod = 5;              //нивелирование помех
volatile long VentPeriod = 600000,          //период между лопастями в тиках
              VentPeriods[16] = {1200000,1200000,1200000,1200000, 1200000,1200000,1200000,1200000, 1200000,1200000,1200000,1200000, 1200000,1200000,1200000,1200000},
              idxVentPeriods = 0,
              VentPeriodTimer = 50000,      //период смены картинок вентилятора на дисплее в мс, от 40мс, пропорционально VentPeriod
              VentSpeed = 0;                //скорость в оборотах в минуту

              
//датчик DS18B20
#include <OneWire.h>
OneWire ds(7); // Объект OneWire
volatile int TempOut = 0;                           // Глобальная переменная для хранения значение температуры с датчика DS18B20
volatile unsigned long lastticTemp = 0;             // Переменная для хранения времени последнего считывания с датчика
const int updatetimeTemp = 10000;          // Определяем периодичность проверок в тиках = 1 сек

//сенсоры давления и температуры BMP280
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP0_CS 10
#define BMP1_CS 9
//Adafruit_BMP280 bme; // I2C
Adafruit_BMP280 bme0(BMP0_CS); // hardware SPI
Adafruit_BMP280 bme1(BMP1_CS); // hardware SPI
//Adafruit_BMP280 bme(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);
const long correctPressure = 45;     //коррекция разности измерения давления

/* на hardware serial 18 05 2019
//дисплей Nextion
#include       <SoftwareSerial.h>                                                              // Подключаем библиотеку SoftwareSerial для работы с программным UART
const uint8_t  pinRX   = 6;                                                                    // Определяем константу хранящую номер вывода Arduino RX программного UART, подключается к выводу TX дисплея
const uint8_t  pinTX   = 5;                                                                    // Определяем константу хранящую номер вывода Arduino TX программного UART, подключается к выводу RX дисплея
SoftwareSerial Serial(pinRX, pinTX);
*/

//переменные состояния
bool Enab = false;                              //состояние включено/выключено
bool Alarm = false;                             //наличие аварии
//unsigned long lastticDisable = 0;               //тик выключения
//const unsigned long timeoutDisable = 100000;    //таймаут 10 сек работы вентилятора после выключения
int TempSet = 24;                               //температура выставленная с дисплея
const long picTenOff = 14;                      //картинка выключенного тэна
const long picTenOn = 15;                       //картинка включенного тэна

//константы
const int PressureVentSpeed = 1000;              //минимальное количество оборотов в минуту при котором детектируем работоспособность фильтра и разрешаем включать нагрев
const int PressureDeltaMin = 3;               //при наличии фильтра минимальная разность давлений
const int PressureDeltaMax = 200;             //при наличии фильтра максимальная разность давлений
const int TempMin1 = 1;                         //разность температур от установленной для включения одного ТЭН`а
const int TempMin2 = 2;                         //разность температур от установленной для включения двух ТЭН`ов
const int TempMin3 = 4;                         //разность температур от установленной для включения всех ТЭН`ов
const int GraphID = 29;                         //id объекта waveform на nextion

//----------------------ОБРАБОТЧИКИ ПРЕРЫВАНИЙ--------------------------
void timer_interrupt() {       // прерывания таймера срабатывают каждые 100 мкс
  tic++;                       // счетчик, 1 тик = 100 мкс
  // если настало время и включено
  if (((tic - lastticDimmer) > (DimmerMax-Dimmer))and Enab) digitalWrite(dimPin, 1);   // врубить ток
}

void  detect_zero() {  // обработка внешнего прерывания от диммера
  digitalWrite(dimPin, 0);                  // вырубить ток
  lastticDimmer = tic;
}

void  detect_speed() {  // обработка внешнего прерывания от датчика лопасти
  if ((tic - lastticSpeed) > minPeriod) {
    long v = tic - lastticSpeed;
    lastticSpeed = tic;
    VentPeriods[idxVentPeriods] = v;
    idxVentPeriods++;
    if (idxVentPeriods>15) idxVentPeriods=0;
  }
}
//----------------------ОБРАБОТЧИКИ ПРЕРЫВАНИЙ--------------------------

//получить значение температуры на выходе с bs18d20
//в переменную TempOut
int detectTemperature() {
  byte data[2];
  ds.reset();
  ds.write(0xCC);
  ds.write(0x44);
  if (tic - lastticTemp > updatetimeTemp)
  {
    lastticTemp = tic;
    ds.reset();
    ds.write(0xCC);
    ds.write(0xBE);
    data[0] = ds.read();
    data[1] = ds.read();
    // Формируем значение
    TempOut = (data[1] << 8) + data[0]; TempOut = TempOut >> 4;
  }
  return TempOut;
}

//получить значение скорости вентилятора в оборотах в минуту
//в переменную VentSpeed
void CalcSpeed() {
  VentPeriod=0;
  for (int i = 0 ; i<= 15; i++) {
    VentPeriod = VentPeriod + VentPeriods[i];
  }
  VentPeriod = VentPeriod >> 4;  //делим на 16, получим среднее значение за последние 16 измерений
  VentSpeed = (60 * 1000000) / (VentPaddle * VentPeriod * ticDelay); //здесь вычисление скорости в оборотах в минуту
  VentPeriodTimer = (VentPaddle * VentPeriod * ticDelay / 500); //период смены картинок вентилятора на дисплее, 40мс  для 3000 оборотов в минуту - максимум
  if (VentPeriodTimer < 50) VentPeriodTimer = 50;
  if (VentPeriodTimer > 50000) VentPeriodTimer = 50000;
}

//отправить текст в текстовое поле на nextion
void nextionSendText(String element, String data, String page = "0") {
  Serial.print("page" + page + "." + element + ".txt=\"");
  Serial.print(data);
  Serial.print("\"");
  Serial.write(0xFF);
  Serial.write(0xFF);
  Serial.write(0xFF);
}

//отправить число в цифровое поле на nextion
void nextionSendVal(String element, long data, String page = "0") {
  Serial.print("page" + page + "." + element + ".val=");
  Serial.print(String(data));
  Serial.write(0xFF);
  Serial.write(0xFF);
  Serial.write(0xFF);
}

//отправить период таймера в nextion
void nextionSendTim(String element, long data, String page = "0") {
  Serial.print("page" + page + "." + element + ".tim=");
  Serial.print(String(data));
  Serial.write(0xFF);
  Serial.write(0xFF);
  Serial.write(0xFF);
}

//отправить ИД картинки у element на nextion
void nextionSendPic(String element, long data, String page = "0") {
  Serial.print("page" + page + "." + element + ".pic=");
  Serial.print(String(data));
  Serial.write(0xFF);
  Serial.write(0xFF);
  Serial.write(0xFF);
}

//отправить на график занчения
void nextionSendGraph(int id, int canal, int data) {
  String comando  = "add ";
  comando += id;
  comando += ",";
  comando += canal;
  comando += ",";
  comando += data;
  comando += "\xFF\xFF\xFF";
  Serial.print(comando);
}

//получить значение val для указаного element из nextion
long nextionGetVal(String element) {
  unsigned long r = 0;
  Serial.print("print " + element + ".val");
  Serial.print("\"");
  Serial.write(0xFF);
  Serial.write(0xFF);
  Serial.write(0xFF);
  delay(15);  //подождем 15мс ответа
  while (Serial.available()) {
    r = (r << 8) | Serial.read(); // младший байт вначале, 4 байта поймать
  }
  return r;
}



//парсим команду полученую от nextion
int parseCommand(String command) {
  int r=0;
  if (command.indexOf("ON")>=0) {
//    Serial.println("Resolve command ON");
    nextionSendText("tmessage", "ON");
    allEnable();
    r=1;
  }
  if (command.indexOf("OFF")>=0) {
//    Serial.println("Resolve command OFF");
    nextionSendText("tmessage", "OFF");
    allDisable();
    r=1;
  }
  if (command.indexOf("TEMPUP")>=0) {
//    Serial.println("Resolve command TEMPUP");
    TempSet++;
    if (TempSet>50) TempSet=50;
    r=1;
  }  
  if (command.indexOf("TEMPDOWN")>=0) {
//    Serial.println("Resolve command TEMPDOWN");
    TempSet--;
    if (TempSet<0) TempSet=0;
    r=1;
  }    
  if (command.indexOf("VENTUP")>=0) {
 //   Serial.println("Resolve command VENTUP");
    Dimmer=Dimmer+5;
    if (Dimmer<20) Dimmer=20;
    if (Dimmer>100) Dimmer=100;
    r=1;
  }  
  if (command.indexOf("VENTDOWN")>=0) {
 //   Serial.println("Resolve command VENTDOWN");
    Dimmer=Dimmer-5;
    if (Dimmer<20) Dimmer=0;
    r=1;
  }   
  return r;
  /*
  if (command.indexOf("VentilatorSpeed=")>=0 && command.length()==20) {
    Serial.print("Resolve command VentilatorSpeed, set:");
    Serial.println((command.substring(command.indexOf("=")+1)));
    Dimmer = DimmerMax - (command.substring(command.indexOf("=")+1)).toInt();
    //nextionSendText("tmessage", "Power "+command.substring(command.indexOf("=")+1)+" %");
    nextionSendText("tmessage", "Power "+String((command.substring(command.indexOf("=")+1)).toInt())+" %");
  }
  if (command.indexOf("TempSet=")>=0 && command.length()==12) {
      Serial.print("Resolve command TempSet, set:");
      Serial.println(command.substring(command.indexOf("=")+1));
      nextionSendVal("ntempset", command.substring(command.indexOf("=")+1).toInt() );
      nextionSendText("tmessage", "Temperature "+String((command.substring(command.indexOf("=")+1)).toInt())+" gr");      
      TempSet = (command.substring(command.indexOf("=")+1)).toInt();
      nextionSendVal("ntempset", TempSet);    //для обратной связи, полученная выставляемая температура из nextion отправляем обратно на nextion для отображения 
  }
  */
}

//при наличии текста в буфере последовательного порта пришедшего из nextion читаем текст и выделяем из него команды. команды между символами #
int getCommand() {
  int r=0;
  char c = ' ';
  while (Serial.available())                //чтото в буфере есть
    if (c == '#') {                             //поймали начало текстовой команды
      String command = "";
      c = ' ';
      delay(100);                               //подождем 0.1 сек, самая длинная команда будет вся в буфере
      while (Serial.available()) {          //читаем всю команду пока буфер не опустеет либо не начнется следующая команда
        c = Serial.read();
        if (c == '#') break;                    //пошла следующая команда
        command += c;
      }
//      Serial.print("Get message from nextion: ");
//      Serial.println(command);
      r=parseCommand(command);                     //Отправляем команду на разбор
    } else {
      c = Serial.read();
    }
  return r;
}


void toggleTEN(bool enab, int ten) {
  if (enab) {
    if (ten==0){
        digitalWrite(ten0Pin, 1);
        nextionSendPic("ten0", picTenOn); 
    } else if (ten==1){
        digitalWrite(ten1Pin, 1);
        nextionSendPic("ten1", picTenOn); 
    } else if (ten==2){
        digitalWrite(ten2Pin, 1);
        nextionSendPic("ten2", picTenOn);
    }   
  } else { 
    if (ten==0){
        digitalWrite(ten0Pin, 0);
        nextionSendPic("ten0", picTenOff); 
    } else if (ten==1){
        digitalWrite(ten1Pin, 0);
        nextionSendPic("ten1", picTenOff); 
    } else if (ten==2){
        digitalWrite(ten2Pin, 0);
        nextionSendPic("ten2", picTenOff);
    }     
  }
}

//Всё включить
void allEnable() {
  Enab = true;
  Alarm = false;
//  digitalWrite(ventPin, 1); //вкл вентилятора
  nextionSendText("tmessage", "ON");
  nextionSendVal("ventenable", 1);
//  Serial.println("All enable.");
}

//Всё выключить
void allDisable() {
  Enab = false;
  //выкл всех ТЭН`ов
  toggleTEN(false, 0);
  toggleTEN(false, 1);
  toggleTEN(false, 2);
  //выкл вентилятора
//  Dimmer=0;
//  digitalWrite(ventPin, 0);
  nextionSendVal("btenable", 0);
  nextionSendVal("ventenable", 0);
//  nextionSendText("tmessage", "OFF");
//  Serial.println("All disable.");
}

//процесс поддержания температуры
void heating() {
  int DeltaTemp = TempSet - TempOut;
  if (not Enab or (DeltaTemp <= 0) or (VentSpeed < PressureVentSpeed)) {
    //выключить все ТЭН`ы если все выключено либо температура на выходе больше установленой либо вентилятор не гонит воздух
    toggleTEN(false, 0);
    toggleTEN(false, 1);
    toggleTEN(false, 2);
  } else if (DeltaTemp >= TempMin3) {
    toggleTEN(true, 0);
    toggleTEN(true, 1);
    toggleTEN(true, 2);
  } else if (DeltaTemp >= TempMin2) {
    toggleTEN(true, 0);
    toggleTEN(true, 1);
    toggleTEN(false, 2);
  } else if (DeltaTemp >= TempMin1) {
    toggleTEN(true, 0);
    toggleTEN(false, 1);
    toggleTEN(false, 2);
  }
}

void checkAlarm() {
  if (analogRead(alarmPin) > 500) {
    Alarm = true;
    nextionSendText("tmessage", "Overheating !");
    allDisable();
  }
}

//отрисовываем на графике температура снаружи, температура на выходе, выставленая температура, включенные нагреватели
void drawGraph() {
  int tin,tout,tset,ten;
  ten=0;
  if (digitalRead(ten0Pin)==1) ten+=3;
  if (digitalRead(ten1Pin)==1) ten+=3;
  if (digitalRead(ten2Pin)==1) ten+=3;
  tin = constrain(bme0.readTemperature(),0,40);
  tout = constrain(TempOut,0,40);
  tset = constrain(TempSet,0,40);
  tin = map(tin,0,40,0,100);
  tout = map(tout,0,40,0,100);
  tset = map(tset,0,40,0,100);
  nextionSendGraph(GraphID,0,tin);
  nextionSendGraph(GraphID,1,tout);
  nextionSendGraph(GraphID,2,tset);
  nextionSendGraph(GraphID,3,ten);
}


void setup() {
  
  //датчик перегрева, подтянут к плюсу
  pinMode(alarmPin, INPUT_PULLUP);  
  
  // порты вкл выкл тэнов
  //  pinMode(ventPin, OUTPUT);
  //  digitalWrite(ventPin, 0);
  pinMode(ten0Pin, OUTPUT);
  digitalWrite(ten0Pin, 0);
  pinMode(ten1Pin, OUTPUT);
  digitalWrite(ten1Pin, 0);
  pinMode(ten2Pin, OUTPUT);
  digitalWrite(ten2Pin, 0);


  //диммер
  pinMode(dimPin, OUTPUT);
  digitalWrite(dimPin, 0);                  // вырубить ток на семисторе
  pinMode(zeroPin, INPUT);                  // настраиваем порт на вход для отслеживания прохождения сигнала через ноль
  StartTimer1(timer_interrupt, ticDelay);        // время для одного разряда ШИМ  полупериод 50Гц = 0.01сек = 100*100 мкс  = 100 тиков
  attachInterrupt(digitalPinToInterrupt(zeroPin), detect_zero, RISING);  // настроить срабатывание прерывания interrupt0 на pin 2 на смену уровня

  //датчик скорости вентилятора
  pinMode(speedPin, INPUT);                 // настраиваем порт на вход для отслеживания прохождения сигнала через ноль
  attachInterrupt(digitalPinToInterrupt(speedPin), detect_speed, FALLING);  // настроить срабатывание прерывания interrupt1 на pin 3 на переход в верхнего на низкий уровень , прохождение лопасти

  //дисплей nextion
/*  Serial.begin(9600);
  
  Serial.print("baud=57600");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);
  delay(100);
  Serial.end();*/
  Serial.begin(115200);

  //если датчики BMP280 не подключены - зависнуть
//  Serial.begin(9600);
//  Serial.println("PROPELLER");
//  Serial.println("Version Beta");
//  Serial.println("Protasov AA 2018 12 04");
  if (!bme0.begin()) {
//    Serial.println("Could not find BMP0 a valid BMP280 sensor, check wiring!");
    nextionSendText("tmessage", "No sensor 0 !");
    while (1);
  }
  if (!bme1.begin()) {
//    Serial.println("Could not find BMP1 a valid BMP280 sensor, check wiring!");
    nextionSendText("tmessage", "No sensor 1 !");
    while (1);
  }

  //экран привести в начальное состояние
  allDisable();   //Всё выключить
  nextionSendVal("nvent", Dimmer);
//  nextionSendVal("htemp", TempSet);
  nextionSendVal("ntempset", TempSet);
  nextionSendText("tmessage", VER);
//  Serial.println(VER);
//  Serial.println("I am ready !");
}

void loop() {
  for (int i=0; i < 10; i++){
    detectTemperature();
    CalcSpeed();
  //проверяем сигналы аварий
    checkAlarm();
    delay(100);    
  //ловим команды от nextion и выполняем их
    if (getCommand()>0) break;  
  }  
//греем
  heating();
//передаем значения в nextion
  nextionSendVal("btenable", Enab);
  nextionSendVal("ventspeed", VentSpeed);
  nextionSendTim("venttimer", VentPeriodTimer);               //вращение вентилятора
  nextionSendVal("ntempreal", TempOut);
  nextionSendVal("nvent", Dimmer);
  nextionSendVal("ntempset", TempSet);
  nextionSendVal("ntempin", bme0.readTemperature());
  nextionSendVal("ntempout", bme1.readTemperature());
  long pressin = bme0.readPressure();
  long pressout = pressin - bme1.readPressure() + correctPressure;
  nextionSendVal("npressurein", pressin);
  nextionSendVal("npressureout", pressout);
  if (VentSpeed > PressureVentSpeed) {  ///если вентилятор вращается
    if ( (bme0.readPressure() - bme1.readPressure() + correctPressure) > PressureDeltaMax ) {
      nextionSendText("tmessage", "Filter      clugged.");
    }
    if ( (bme0.readPressure() - bme1.readPressure() + correctPressure) < PressureDeltaMin ) {
      nextionSendText("tmessage", "Filter not  available.");
    }
  }
  drawGraph();

  //за 2 секунды небыло прерываний от датчика лопасти - значит вентилятор остановился.
  if ((tic - lastticSpeed) > 20000) for (int i = 0 ; i<= 15; i++) VentPeriods[i]=1200000;

}
