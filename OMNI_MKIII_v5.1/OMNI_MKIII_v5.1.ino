#include <avr/sleep.h>
#include <avr/wdt.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include "LoRaWAN_TLM922S.h"
// #include <LoRaWAN_TLM922S_SoftwareSerial.h>

//*************************************************//
// for a PCB of OMNI version 0.1.0
#define CONSOLE_BAUD  9600

//**************OMNI Setting***********//
const int durationOfSleep = 0; //sleep minutes
const int numOfGPSTrial = 6; 
const int durationOfRecievingGPS = 50 ;// second
const int durationOf2ndGPS = 50; //second
const int durationOfLoRaConecctionTrial = 120;

//******** select SLEEP Time ******************************************************//
bool bSleep = true;
const unsigned long SLEEP_INTERVAL = (durationOfSleep * 60) / 8 ; // OMNI sleeps for interval * 8sec.
//30min 1800s / 8 = 225, 10min 600s / 8 =  75, 1h 510
const int maxgpstry = durationOfRecievingGPS * 50; //time for getting GPS signal / 100 sec (9000 = 90sec ))
const int LoRatry = durationOfLoRaConecctionTrial;  // try LoRa connection for 60 times (120 sec)

//******** pin assign ******************************************************//
const uint8_t  TemperatureOutput1Pin = A0;
const uint8_t  TemperatureOutput2Pin = A1;
const uint8_t  TemperatureOutput3Pin = A2;
const uint8_t  TemperatureEnablePin = 10; // 3-pins package does not have SHUTDOWN PIN, Use Power Pin for enable
const uint8_t  GPSpower = 8;
const uint8_t  GPSrxPin = 9;
const uint8_t  GPGStxPin = 2;
const uint8_t  LoRaTX = 7;
const uint8_t  LoRaRX = 6;
const uint8_t  onBoardLED = 13;

//******** LoRaWAN senseway ******************************************************//
#define LORAWAN_BAUD  9600
#define SET_ADR     false        // adavtipe datarate mode
// or
#define SET_DR      3           // manual datarate
#define SET_FPORT   1
#define SET_TX      TX_CNF
#define SET_JOIN    JOIN_OTAA
#define SET_LCHK    false
LoRaWAN_TLM922S LoRaWAN(LoRaRX, LoRaTX);
// LoRaWAN_TLM922S_SoftwareSerial LoRaWAN(LoRaRX, LoRaTX);
#define LORA_WAKE_PIN    3
//device detail
//https://github.com/askn37/LoRaWAN_TLM922S

//******** gps ******************************************************//
SoftwareSerial gpsSS(GPSrxPin, GPGStxPin);
TinyGPSPlus gps;
const uint32_t gpsBaudRate = 9600;

uint32_t numSatellites = 0;;
float lat = 0.f;
float lng = 0.f;
float meters = 0.f;
//device detail
//https://www.yuden.co.jp/ut/product/category/module/GYSFDMAXB.html

//******** temperature ******************************************************//
////////////// Constant and variable for TMP36 sensor
const float baselineTemp = 25.0; // factory calibrate 750mV at 25oC
const float baselineVol = 750.0; // factory calibrate 750mV at 25oC
const float refVol = 5000.0; // ADC ref 5V
const float tempScale = 10.0; //10mV/oC
const int numSample = 3; // number of average sample
const int TEMPSENSOR_SETTLEDELAY = 1000; // delay for settle time
const int TEMPSENSOR_READ_INTERVAL = 100; // delay between samples
int count = 0; // counting variable for taking average
float temp1 = 0.0; // temperature sensor 1 readings
float temp2 = 0.0; // temperature sensor 2 readings
float temp3 = 0.0; // temperature sensor 3 readings
const float OFFSET1 = 3.0; // offset to calibrate sensor 1
const float OFFSET2 = 3.0; // offset to calibrate sensor 2
const float OFFSET3 = 3.0; // offset to calibrate sensor 3


//******** restart ******************************************************//
void software_reset() {
   Serial.println("reset");
  wdt_disable();Serial.println("diabeled");
  wdt_enable(WDTO_15MS);
  while (1) {}
}

//******** sleep ******************************************************//
bool bIsGPSUpdated = false;
bool bNeedToSendData = false;

//sleep watchdog
int wakePin = 1; 
bool initFlg = true;
volatile int wdt_cycle = 0; 
volatile int wdt_counter = 0; 

// work around http://milkandlait.blogspot.com/2014/06/avr_3.html
uint8_t mcusr_mirror __attribute__ ((section (".noinit")));
void get_mcusr(void) __attribute__((naked)) __attribute__((section(".init3")));
void get_mcusr(void)
{
  mcusr_mirror = MCUSR;
  MCUSR = 0;
  wdt_disable();
}

//---------------------------------------------------
void setup()
{
  //serial setting
  Serial.begin(CONSOLE_BAUD);
  delay(100);

  //LED setting
  pinMode(onBoardLED, OUTPUT);
  
   //sleep setting
  pinMode(wakePin, INPUT);

  //start
  Serial.println("This is OMNI");
  digitalWrite(onBoardLED, HIGH);
  
  //---------sensor---------------
  Serial.println("preparing temperature.");
  initTemperatureSensors();
  delay(100);
 // while(1){ //debug
  getTemperature();
  Serial.println("Temperatures -----------");
  Serial.print("Temp sensor 1: ");  Serial.println(temp1, 2);
  Serial.print("Temp sensor 2: ");  Serial.println(temp2, 2);
  Serial.print("Temp sensor 3: ");  Serial.println(temp3, 2);
  delay(100);
  delay(500);
  //}

  blinkLED(2);
  digitalWrite(onBoardLED, HIGH);
  
  Serial.println("preparing GPS.");
  initGPS();
  digitalWrite(GPSpower, LOW);
  gpsSS.begin(gpsBaudRate);
  
  Serial.println("Network -----------");
  Serial.println("initializing LoRa.");

  LoRaWAN.begin(LORAWAN_BAUD);
  LoRaWAN.setEchoThrough(ECHO_ON);
    while (!LoRaWAN.getReady())     delay(1000);
    while (!LoRaWAN.factoryReset()) delay(1000);
    Serial.println(F("try join otaa"));
    while (!(LoRaWAN.join(JOIN_OTAA) &&
             LoRaWAN.joinResult())) delay(1000);
    Serial.println(F("accepted"));
    
  pinMode(LORA_WAKE_PIN, OUTPUT);
  digitalWrite(LORA_WAKE_PIN, LOW);
  delay(1000);
  

  blinkLED(3);
  digitalWrite(onBoardLED, HIGH);
  
  getGPS();
  
  Serial.println("Send first Data."); 
  sendDataLoRa();
  delay(200);
  //Serial.println("Sleep LoRa-----------");
  //LoRaWAN.sleep(0);

  delay(1000);

}

//---------------------------------------------------
void loop()
{
  //-----------sleep Arduino-----------------------
  if(bSleep){
    wdt_counter++;
   //if wdt_counter is below SLEEP_INTERVAL, OMNI sleep again for 8 sec.
   if( wdt_counter <= SLEEP_INTERVAL ){ 
   // LoRaWAN.sleep(8)
     Serial.println(wdt_counter);
     delay(10);
      delayWDT(9); // 8sec 
      return; 
    } 
    if(wdt_counter > SLEEP_INTERVAL ){
       wdt_counter = 0;
    }
  }
  
 //------- start sensing -----------

Serial.println("GPS -----------");
 getGPS();
 delay(500);

 
 Serial.println("Temperatures -----------");
 getTemperature();
 delay(500);
 Serial.print("Temp sensor 1: ");  Serial.println(temp1, 2);
 Serial.print("Temp sensor 2: ");  Serial.println(temp2, 2);
 Serial.print("Temp sensor 3: ");  Serial.println(temp3, 2);
 Serial.println("\n");
 delay(100);
 
 //--------------- start network connection -----------
  Serial.println("LoRa -----------");
// digitalWrite(NetworkPower, LOW);
 LoRaWAN.begin(LORAWAN_BAUD);
 //while (!LoRaWAN.wakeUp());  // PS_OK
 delay(100);
 sendDataLoRa();
 
 if(bSleep){
    
   // Serial.println("Network power down.");  
   // digitalWrite(LORA_WAKE_PIN, LOW);
    delay(200);  
    //LoRaWAN.sleep(0);
    delay(1000);  
  }
}

/*****************************************/
/*
 * watch dog http://radiopench.blog96.fc2.com/
 */
/*****************************************/
void delayWDT_setup(unsigned int ii) { // ウォッチドッグタイマーをセット。
 // 引数はWDTCSRにセットするWDP0-WDP3の値。設定値と動作時間は概略下記
 // 0=16ms, 1=32ms, 2=64ms, 3=128ms, 4=250ms, 5=500ms
 // 6=1sec, 7=2sec, 8=4sec, 9=8sec
 byte bb;
 if (ii > 9 ){ // 変な値を排除
 ii = 9;
 }
 bb =ii & 7; // 下位3ビットをbbに
 if (ii > 7){ // 7以上（7.8,9）なら
 bb |= (1 << 5); // bbの5ビット目(WDP3)を1にする
 }
 bb |= ( 1 << WDCE );
 
 MCUSR &= ~(1 << WDRF); // MCU Status Reg. Watchdog Reset Flag ->0
 // start timed sequence
 WDTCSR |= (1 << WDCE) | (1<<WDE); // ウォッチドッグ変更許可（WDCEは4サイクルで自動リセット）
 // set new watchdog timeout value
 WDTCSR = bb; // 制御レジスタを設定
 WDTCSR |= _BV(WDIE);
} 
 
ISR(WDT_vect) { // WDTがタイムアップした時に実行される処理
 // wdt_cycle++; // 必要ならコメントアウトを外す
}
 
 
void delayWDT(unsigned long t) { // パワーダウンモードでdelayを実行
 
 delayWDT_setup(t); // ウォッチドッグタイマー割り込み条件設定
 ADCSRA &= ~(1 << ADEN); // ADENビットをクリアしてADCを停止（120μA節約）
 set_sleep_mode(SLEEP_MODE_PWR_DOWN); // パワーダウンモード
 sleep_enable();
 
 sleep_mode(); // ここでスリープに入る
 
 sleep_disable(); // WDTがタイムアップでここから動作再開 
 ADCSRA |= (1 << ADEN); // ADCの電源をON (|=が!=になっていたバグを修正2014/11/17)
 
}
/*****************************************/
/* END WATCH DOG / SLEEP */
/*****************************************/

//---------------------------------------------------
//--------------------Sensors----------------------------




void initTemperatureSensors()
{
  digitalWrite(TemperatureEnablePin, LOW); // make sure all temperature are OFF
	delay(TEMPSENSOR_SETTLEDELAY);
}


//---------------------------------------------------
void getTemperature()
{
  
  temp1 = 0.0;
	temp2 = 0.0;
	temp3 = 0.0;

	digitalWrite(TemperatureEnablePin, HIGH); // turn on all temperature sensor
  delay(TEMPSENSOR_SETTLEDELAY);
	
	analogRead(TemperatureOutput1Pin);
	delay(TEMPSENSOR_READ_INTERVAL);//trick to let ADC to accommodate high impedance sensor: https://forums.adafruit.com/viewtopic.php?f=25&t=11597
	sampleTemperature(TemperatureOutput1Pin, &temp1, OFFSET1);

	analogRead(TemperatureOutput2Pin);
	delay(TEMPSENSOR_READ_INTERVAL);//trick to let ADC to accommodate high impedance sensor: https://forums.adafruit.com/viewtopic.php?f=25&t=11597
	sampleTemperature(TemperatureOutput2Pin, &temp2, OFFSET2);


	analogRead(TemperatureOutput3Pin);
	delay(TEMPSENSOR_READ_INTERVAL);//trick to let ADC to accommodate high impedance sensor: https://forums.adafruit.com/viewtopic.php?f=25&t=11597
	sampleTemperature(TemperatureOutput3Pin, &temp3, OFFSET3);

	digitalWrite(TemperatureEnablePin, LOW); // turn on all temperature sensor
	delay(TEMPSENSOR_SETTLEDELAY);
  
}

void sampleTemperature(uint8_t pin, float* temp_read, float offset){
	for (count = 0; count < numSample; count++ ){
		*temp_read = *temp_read + baselineTemp + (analogRead(pin)/1023.0*refVol-baselineVol)/tempScale;
		delay(TEMPSENSOR_READ_INTERVAL);
	}
	*temp_read = *temp_read/numSample + offset;
}

void initGPS(){
  pinMode(GPSpower, OUTPUT);
  digitalWrite(GPSpower, LOW);
}


//---------------------------------------------------
void getGPS(){
  digitalWrite(GPSpower, HIGH);
  Serial.println("turn on GPS");
  delay(500);
  gpsSS.listen();
  
  if(initFlg){
    Serial.println("waiting for GPS");
    for(int i = 0 ; i < durationOf2ndGPS ; i++ ){ 
     delay(1000);
     //Serial.println(i);
    }
    initFlg = false;
  }

  Serial.println("getting GPS signal...");
  
  gpsSS.listen();
  
  int gpstry = maxgpstry;
  recieveGPS(maxgpstry);
  int trial=1;
  
  while(lat==0.0){
    trial++;
    if(trial > numOfGPSTrial){
      break;
    }
    Serial.print(trial);
    Serial.println("try...");
    recieveGPS(durationOf2ndGPS*100);
     Serial.print("LAT="); Serial.println(gps.location.lat(), 6);
     Serial.print("LONG="); Serial.println(gps.location.lng(), 6);
  }

  
 //GPS down
   digitalWrite(GPSpower, LOW);
   delay(1000);
 
 
 Serial.print("LAT="); Serial.println(gps.location.lat(), 6);
 Serial.print("LONG="); Serial.println(gps.location.lng(), 6);
 Serial.print("ALT="); Serial.println(gps.altitude.meters());
 Serial.print("SAT="); Serial.println(gps.satellites.value());
  //LED
  if(lat >0){
     digitalWrite(onBoardLED, LOW);
  }

  
}

void recieveGPS( int gpstry ){
  while(gpstry > 0) {
   while (gpsSS.available() > 0)
    { 
    char c = gpsSS.read();
    //Serial.print(c);
     if( gps.encode(c)){
        lat = gps.location.lat();
        lng = gps.location.lng();
        meters = gps.altitude.meters();
        numSatellites = gps.satellites.value();
       // Serial.print(lat);
       // Serial.print(" ");
       // Serial.println(lng);
     }
   }  
    gpstry--;
    delay(10);
  } 
}




//---------------------------------------------------
//--------------------Senseway----------------------------

void sendDataLoRa(){

  initLoRa();
  if(LoRaJoin()){
  
 uint32_t v;
    bool f;

    if (SET_LCHK) {
        // リンクチェックを要求する
        // 成功なら真
        f = LoRaWAN.setLinkCheck();
        Serial.print(F("=setLinkCheck:")); Serial.println(f);
        printResult();
    }

    // txコマンドを準備する
    // 成功すれば真
    f = LoRaWAN.tx(SET_TX, SET_FPORT);
    Serial.print(F("=tx:")); Serial.println(f);
    printResult();
    
    // 送信データを準備する

     Serial.println("Temperatures -----------");
     Serial.println(temp1, 2);
  	 Serial.println(temp2, 2);
  	 Serial.println(temp3, 2);
     Serial.println("\n");

    char str[32] = "";
    float2ByteString(lat, str);
    LoRaWAN.write(str);  delay(10);   

    float2ByteString(lng, str);
    LoRaWAN.write(str); delay(10);  

    float2ByteString(meters, str);
    LoRaWAN.write(str); delay(10);     

    float2ByteString(temp1, str);
    LoRaWAN.write(str); delay(10); 

    float2ByteString(temp2, str);
    LoRaWAN.write(str); delay(10); 

    float2ByteString(temp3, str);
    LoRaWAN.write(str); delay(10); 

    float2ByteString(numSatellites, str);
    LoRaWAN.write(str); delay(10); 


    // 送信を実行
    // 第1プロンプト結果が Okなら真
    int txtry = 120;
    f = LoRaWAN.txRequest();
    Serial.print(F("=txq:")); Serial.println(f);
    printResult();
    Serial.println("try to send data.");  
    while(!f && txtry >0 ){
        Serial.println(txtry);
          f = LoRaWAN.txRequest();
          Serial.print(F("=txq:")); Serial.println(f);
          printResult();
          txtry--;
          delay(500);
    }
    
    if (f) {
        // 第2プロンプトを待つ
        // 結果が tx_okなら真
        f = LoRaWAN.txResult();
        Serial.print(F("=txResult:")); Serial.println(f);
        printResult();

        // リンクチェックが得られたなら真（ucnfでもダウンリンク発生）
        Serial.print(F("=isLinkCheck:")); Serial.println(LoRaWAN.isLinkCheck());
        if (LoRaWAN.isLinkCheck()) {
            // マージン値とゲートウェイ数を取得
            Serial.print(F("=getMargin:")); Serial.println(LoRaWAN.getMargin());
            Serial.print(F("=getGateways:")); Serial.println(LoRaWAN.getGateways());
        }

        // rxダウンリンクデータが得られたなら真
        Serial.print(F("=isRxData:")); Serial.println(LoRaWAN.isRxData());
        if (LoRaWAN.isRxData()) {
            // rxポート番号とバイナリデータを取得
            Serial.print(F("=getRxPort:")); Serial.println(LoRaWAN.getRxPort());
            Serial.print(F("=getRxData:["));
            Serial.write(LoRaWAN.getRxData(), LoRaWAN.isRxData());
            Serial.println(']');
        }
    }
  }
  
}

void initLoRa(){
  
   bool f;
  Serial.println(F("LoRa Startup"));
  LoRaWAN.begin(LORAWAN_BAUD);
   LoRaWAN.setEchoThrough(ECHO_ON);
    while (!LoRaWAN.getReady())     delay(1000);
    while (!LoRaWAN.factoryReset()) delay(1000);
    Serial.println(F("try join otaa"));
    while (!(LoRaWAN.join(JOIN_OTAA) &&
             LoRaWAN.joinResult())) delay(1000);
    Serial.println(F("accepted"));

    // ファームウェアバージョン文字列取得
    // 成功すれば真
    f = LoRaWAN.getVersion();
    Serial.print(F("=getVersion:")); Serial.println(f);
    printResult();
    if (f) {
        Serial.print(F("=isData:")); Serial.println(LoRaWAN.isData());
        if (LoRaWAN.isData()) {
            Serial.print(F("=getData:["));
            Serial.print(LoRaWAN.getData());
            Serial.println(']');
        }
    }

    // DevEUI文字列取得
    // 成功すれば真
    f = LoRaWAN.getDevEUI();
    Serial.print(F("=getDevEUI:")); Serial.println(f);
    printResult();
    if (f) {
        Serial.print(F("=isData:")); Serial.println(LoRaWAN.isData());
        if (LoRaWAN.isData()) {
            Serial.print(F("=getData:["));
            Serial.print(LoRaWAN.getData());
            Serial.println(']');
        }
    }
    
}

bool  LoRaJoin(){
  
   bool f;
   int loraCount = 0;
    // joinが成功するまでループ
    do {
        if (!f) delay(2000);

        loraCount++;
        Serial.println(loraCount);
        if(loraCount  > LoRatry){
          return false;
        }
        // joinする
        // 第1プロンプト結果が Okなら真
        f = LoRaWAN.join(SET_JOIN);
        Serial.print(F("=join:")); Serial.println(f);
        if (f) {
            // 第2プロンプトを待つ
            // 結果が acceptedなら真
            f = LoRaWAN.joinResult();
            Serial.print(F("=joinResult:")); Serial.println(f);
        }
        printResult();
    } while (!f);

    // join後のDevAddr文字列取得（OTAAで成功すれば以前と変わっている）
    // 成功すれば真
    f = LoRaWAN.getDevAddr();
    Serial.print(F("=getDevAddr:")); Serial.println(f);
    printResult();
    if (LoRaWAN.isData()) {
        Serial.print(F("=getData:["));
        Serial.print(LoRaWAN.getData());
        Serial.println(']');
    }

    if (SET_ADR) {
        // ADRをオンにする
        // 成功なら真
        f = LoRaWAN.setAdr(ADR_ON);
        Serial.print(F("=setAdr:")); Serial.println(f);
        printResult();
    }
    else {
        // DR値を設定する
        // 成功なら真
        f = LoRaWAN.setDataRate(SET_DR);
        Serial.print(F("=setDataRate:")); Serial.println(f);
        printResult();
    }

    // lorawan設定を保存する（DR値など）
    // 成功なら真
    f = LoRaWAN.loraSave();
    Serial.print(F("=loraSave:")); Serial.println(f);
    printResult();

    // module設定を保存する（baudrateなど）
    // 成功なら真
    f = LoRaWAN.modSave();
    Serial.print(F("=modSave:")); Serial.println(f);
    printResult();

    return true;
 
}



// リザルトを取得して番号と名前を表示する
void printResult (void) {
    tlmps_t ps = LoRaWAN.getResult();
    Serial.print(F("=getResult:"));
    Serial.print(ps, DEC);
    Serial.write(':');
    switch (ps) {
        case PS_NOOP       : Serial.println(F("PS_NOOP")); return;
        case PS_READY      : Serial.println(F("PS_READY")); return;
        case PS_PREFIX     : Serial.println(F("PS_PREFIX")); return;
        case PS_ENDRESET   : Serial.println(F("PS_ENDRESET")); return;
        case PS_MODRESET   : Serial.println(F("PS_MODRESET")); return;
        case PS_DEMODMARG  : Serial.println(F("PS_DEMODMARG")); return;
        case PS_NBGATEWAYS : Serial.println(F("PS_NBGATEWAYS")); return;
        case PS_RX         : Serial.println(F("PS_RX")); return;
        case PS_OK         : Serial.println(F("PS_OK")); return;
        case PS_TXOK       : Serial.println(F("PS_TXOK")); return;
        case PS_ACCEPTED   : Serial.println(F("PS_ACCEPTED")); return;
        case PS_JOINED     : Serial.println(F("PS_JOINED")); return;
        case PS_ON         : Serial.println(F("PS_ON")); return;
        case PS_OFF        : Serial.println(F("PS_OFF")); return;
        case PS_INVALID    : Serial.println(F("PS_INVALID")); return;
        case PS_UNKOWN     : Serial.println(F("PS_UNKOWN")); return;
        case PS_ERR        : Serial.println(F("PS_ERR")); return;
        case PS_UNSUCCESS  : Serial.println(F("PS_UNSUCCESS")); return;
        case PS_UNJOINED   : Serial.println(F("PS_UNJOINED")); return;
        case PS_INVDLEN    : Serial.println(F("PS_INVDLEN")); return;
        case PS_KEYNOTINIT : Serial.println(F("PS_KEYNOTINIT")); return;
        case PS_NOFREECH   : Serial.println(F("PS_NOFREECH")); return;
        case PS_BUSY       : Serial.println(F("PS_BUSY")); return;
        case PS_NOTJOINED  : Serial.println(F("PS_NOTJOINED")); return;
        default            : Serial.println(F("???"));
    }
}

void float2ByteString(float val, char buffer[]){
    unsigned char bytes[4];
    float2Bytes(val,&bytes[0]);
    array_to_string(bytes, 4, buffer);  
}

void float2Bytes(float val,byte* bytes_array){
  // Create union of shared memory space
  union {
    float float_variable;
    byte temp_array[4];
  } u;
  // Overite bytes of union with float variable
  u.float_variable = val;
  // Assign bytes to input array
  memcpy(bytes_array, u.temp_array, 4);
}

void array_to_string(byte array[], unsigned int len, char buffer[])
{
    for (unsigned int i = 0; i < len; i++)
    {
        byte nib1 = (array[i] >> 4) & 0x0F;
        byte nib2 = (array[i] >> 0) & 0x0F;
        buffer[i*2+0] = nib1  < 0xA ? '0' + nib1  : 'A' + nib1  - 0xA;
        buffer[i*2+1] = nib2  < 0xA ? '0' + nib2  : 'A' + nib2  - 0xA;
    }
    buffer[len*2] = '\0';
}

void blinkLED(int num){
  digitalWrite(onBoardLED, LOW);
  delay(1000);
  
  for(int i =0 ; i < num ; i++){
      digitalWrite(onBoardLED, HIGH);
      delay(300);
      digitalWrite(onBoardLED, LOW);
      delay(300);
  }
  delay(1000);
}
