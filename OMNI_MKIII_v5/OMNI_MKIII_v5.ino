#include <avr/sleep.h>
#include <avr/wdt.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include "LoRaWAN_TLM922S.h"
// #include <LoRaWAN_TLM922S_SoftwareSerial.h>



//*************************************************//
// for a PCB of OMNI version 0.1.0
#define CONSOLE_BAUD  9600

//**************OMNI Setting***********//
const int durationOfSleep = 25; //sleep minutes
const int numOfSalinityTrial = 10; 
const int durationOfSalinityInterval = 5; 
const int durationOfRecievingGPS = 60 ;// second
const int durationOf2ndGPS = 30; //second
const int durationOfLoRaConecctionTrial = 120;

//******** Network ******************************************************//
// int network = 2;  //no network (debug): 0, LTE:1, LoRa:2 
const int NETWORK_DEBUG = 0;
const int NETWORK_LTE = 1;
const int NETWORK_LORA = 2;
int network = NETWORK_LORA;
//******** select SLEEP Time ******************************************************//
bool bSleep = true;
const unsigned long SLEEP_INTERVAL = (durationOfSleep * 60) / 8 ; // OMNI sleeps for interval * 8sec.
//30min 1800s / 8 = 225, 10min 600s / 8 =  75, 1h 510
const int maxgpstry = durationOfRecievingGPS * 50; //time for getting GPS signal / 100 sec (9000 = 90sec ))
const int LTEtry = 200; // try LTE connection for 120 times (120 sec)
const int LoRatry = durationOfLoRaConecctionTrial;  // try LoRa connection for 60 times (120 sec)

//******** pin assign ******************************************************//
const uint8_t  NetworkPower = 12;
const uint8_t	 TemperatureOutput1Pin = A0;
const uint8_t	 TemperatureOutput2Pin = A1;
const uint8_t	 TemperatureOutput3Pin = A2;
const uint8_t  TemperatureEnablePin = 10; // 3-pins package does not have SHUTDOWN PIN, Use Power Pin for enable
const uint8_t  GPSpower = 8;
const uint8_t  GPSrxPin = 9;
const uint8_t  GPGStxPin = 2;
const uint8_t  LoRaTX = 7;
const uint8_t  LoRaRX = 6;
const uint8_t  ECRX = 4;
const uint8_t  ECTX = 5;
const uint8_t  onBoardLED = 13;

//******** sakura LTE ******************************************************//
//SakuraIO_I2C sakuraio;

//******** LoRaWAN senseway ******************************************************//
#define LORAWAN_BAUD  9600
#define SET_ADR     false        // adaptive datarate mode
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
struct GPSInfo
{
	float lat = 0.f;
	float lng = 0.f;
	float meters = 0.f;
};

SoftwareSerial gpsSS(GPSrxPin, GPGStxPin);
TinyGPSPlus gps;

String mCommand = "";
boolean bIsValidCommand = false;

uint8_t result = 0;

const uint32_t gpsBaudRate = 9600;
const unsigned int GPS_BUFFER_SIZE = 100;
char rawGPSInfo[GPS_BUFFER_SIZE] = "";
int gpsInfoCursor = 0;
uint32_t numSatellites = 0;;
float lat = 0.f;
float lng = 0.f;
float meters = 0.f;

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

//******** EC (electrical conductivity)******************************************************//
SoftwareSerial ECserial(ECRX, ECTX);   
String inputstring = "";                              //a string to hold incoming data from the PC
String sensorstring = "";                             //a string to hold the data from the Atlas Scientific product
boolean input_string_complete = false;                //have we received all the data from the PC
boolean sensor_string_complete = false;               //have we received all the data from the Atlas Scientific product
float psu = 0.0;
float ecValue = 0.0;

//******** restart ******************************************************//
void software_reset() {
	Serial.println("reset");
	wdt_disable();
	Serial.println("diabeled");
	wdt_enable(WDTO_15MS);
	while (1) {}
}
 

//******** sleep ******************************************************//
bool bIsECUpdated = false;
bool bIsGPSUpdated = false;
bool bNeedToSendData = false;

//スリープ・ウォッチドッグ用
int wakePin = 1; //割り込み番号指定(実際は0→1ピンを指定/1→2ピンを指定) 
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
	//serial
	Serial.begin(CONSOLE_BAUD);
	delay(100);


	//LED
	pinMode(onBoardLED, OUTPUT);
	
	//sleep
	pinMode(wakePin, INPUT);

	//temperature sensor enable pin
	pinMode(TemperatureEnablePin, OUTPUT);
	
	Serial.println("This is OMNI");
	digitalWrite(onBoardLED, HIGH);
	
	//---------sensor---------------
	Serial.println("preparing temperature.");
	initTemperatureSensors();
	Serial.println("preparing EC.");
	initEC();
	delay(100);
	getTemperature();
	Serial.println("Temperatures -----------");
	// Serial.print("onBoard");  Serial.println(temperatureValueOnboard, 6);
	// Serial.print("outside");  Serial.println(temperatureValueOutside, 6);
	Serial.print("Temp sensor 1: ");  Serial.println(temp1, 2);
	Serial.print("Temp sensor 2: ");  Serial.println(temp2, 2);
	Serial.print("Temp sensor 3: ");  Serial.println(temp3, 2);
	delay(100);
	Serial.println("Salinity -----------");
	getECValue();

	blinkLED(2);
	digitalWrite(onBoardLED, HIGH);
	
	Serial.println("preparing GPS.");
	initGPS();
	digitalWrite(GPSpower, LOW);
	gpsSS.begin(gpsBaudRate);
	
	Serial.println("Network -----------");


 pinMode(NetworkPower, OUTPUT);
 digitalWrite(NetworkPower, HIGH);
 if(network == NETWORK_LTE ){//LTE 
		Serial.println("initilzing LTE.");
		delay(100);
		digitalWrite(NetworkPower, LOW);
		delay(2000);
		digitalWrite(NetworkPower, HIGH);
 }
 else if(network == NETWORK_LORA){//LoRa 
	 Serial.println("initializing LoRa.");
	 digitalWrite(NetworkPower, LOW);
	 LoRaWAN.begin(LORAWAN_BAUD);
	 LoRaWAN.setEchoThrough(ECHO_ON);

	 pinMode(LORA_WAKE_PIN, INPUT_PULLUP);
	 digitalWrite(LORA_WAKE_PIN, HIGH);
	 delay(1000);

		bool f;
		while (!LoRaWAN.getReady()) {
				printResult();
				Serial.println(F("=getReady:0"));
				delay(1000);
		}
		printResult();

		// 工場出荷時リセットを試す
		// 成功すれば真
		f = LoRaWAN.factoryReset();
		Serial.print(F("=factoryReset:")); Serial.println(f);
		printResult();

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

		// joinが成功するまでループ
		do {
				if (!f) delay(2000);

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

	 bool a = LoRaJoin();
	 Serial.println(a);
	 delay(1000);
	 digitalWrite(NetworkPower, HIGH);

 }
	blinkLED(3);
	digitalWrite(onBoardLED, HIGH);
	
	getGPS();

	
	Serial.println("Send first Data."); 
	digitalWrite(NetworkPower, LOW);
	sendDataLoRa();
	Serial.println("Network power down.");  
	digitalWrite(NetworkPower, HIGH);
	
}

//---------------------------------------------------
void loop()
{

	//-----------sleep-----------------------
	if(bSleep){
	 // Serial.println("Sleep.");  
		wdt_counter++;
	 //カウンターがSLEEP_INTERVAL以下なら再度スリープ状態へ
	 if( wdt_counter <= SLEEP_INTERVAL ){ 
		 Serial.println(wdt_counter);
		 delay(10);
			delayWDT(9); // 8sec 
			return; 
		} 
		if(wdt_counter > SLEEP_INTERVAL ){
			 wdt_counter = 0;
		}
	}

 //  software_reset() ;


	//------- start sensing -----------
 
 getGPS();
 delay(500);
 getTemperature();
 delay(500);
 getECValue();
 delay(500);
 
 Serial.println("Temperatures -----------");
//  Serial.println(temperatureValueOnboard, 6);
//  Serial.println(temperatureValueOutside, 6);
 Serial.print("Temp sensor 1: ");  Serial.println(temp1, 2);
 Serial.print("Temp sensor 2: ");  Serial.println(temp2, 2);
 Serial.print("Temp sensor 3: ");  Serial.println(temp3, 2);
 Serial.println("Salinity -----------");
 Serial.println(psu, 6);
 Serial.println("\n");


 //--------------- start network connection -----------
 digitalWrite(NetworkPower, LOW);
 if(network == NETWORK_LTE ){
	 // sendDataLTE();
 }else if (network == NETWORK_LORA){  //senseway lora
	sendDataLoRa();
 }

	if(bSleep){

				//LTE or LoRa down
			 Serial.println("Network power down.");  
			 digitalWrite(NetworkPower, HIGH);
			 delay(1000);
			 
	}
}

/*****************************************/
/*
 * ウォッチドッグ処理の参考元:2014/11/17 ラジオペンチさん http://radiopench.blog96.fc2.com/
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

void initEC(){
	ECserial.begin(9600);                               //set baud rate for the software serial port to 9600
	inputstring.reserve(10);                            //set aside some bytes for receiving data from the PC
	sensorstring.reserve(30);                           //set aside some bytes for receiving data from Atlas Scientific product
	Serial.println("setting up EC module");
	delay(200);      
	ECserial.print("i");  
	ECserial.print("\r"); 
	getECserial();
	
	ECserial.print("L,0"); 
	ECserial.print("\r"); 
	getECserial();

	ECserial.print("C,0");
	ECserial.print("\r"); 
	getECserial();

	ECserial.print("O,EC,1");
	ECserial.print("\r"); 
	getECserial();

	ECserial.print("O,TDS,0");
	ECserial.print("\r"); 
	getECserial();
	
	ECserial.print("O,S,1");
	ECserial.print("\r"); 
	getECserial();

	ECserial.print("O,SG,0");
	ECserial.print("\r"); 
	getECserial();
	
}

void getECValue(){
	ECserial.listen();
	ECserial.print("\r"); 
	getECserial();
 // delay(500);  
 // ECserial.print("R");  //no temperature compensation
 // ECserial.print("\r"); 
 // getECserial();
	delay(500); 
	float ecv =0;
	float sal =0;
	for(int i =0 ; i < numOfSalinityTrial ; i++){
//		String ECCommand = "T," + String(temperatureValueOutside);
    String ECCommand = "T," + String(temp1);
		Serial.println(ECCommand);
		ECserial.print(ECCommand);
		ECserial.print("\r"); 
		getECserial();
		ECCommand = "R" ; 
		Serial.println(ECCommand);
		ECserial.print(ECCommand);
		ECserial.print("\r"); 
		getECserial();
		sal += psu;
		ecv += ecValue;
		delay(durationOfSalinityInterval * 1000);  
	}
	psu = sal / numOfSalinityTrial;
	ecValue = ecv /numOfSalinityTrial;
	Serial.println("average value");
	Serial.print("EC:");
	Serial.println(ecValue);
	Serial.print("Salinity:");
	Serial.println(psu);

	
	//EC sleep
	ECserial.print("Sleep");
	ECserial.print("\r"); 
	getECserial();
}

void getECserial(){
	delay(1000); 
	while (ECserial.available() > 0) {                     //if we see that the Atlas Scientific product has sent a character
		char inchar = (char)ECserial.read();              //get the char we just received
		sensorstring += inchar;                           //add the char to the var called sensorstring
		if (inchar == '\r') {                             //if the incoming character is a <CR>
			sensor_string_complete = true;                  //set the flag
		}
	}
	if (sensor_string_complete == true) {               //if a string from the Atlas Scientific product has been received in its entirety
		if (isdigit(sensorstring[0]) == false) {          //if the first character in the string is a digit
			Serial.println(sensorstring);                   //send that string to the PC's serial monitor
		}
		else                                              //if the first character in the string is NOT a digit
		{
			print_EC_data();                                //then call this function 
		}
		sensorstring = "";                                //clear the string
		sensor_string_complete = false;                   //reset the flag used to tell if we have received a completed string from the Atlas Scientific product
	}

}

void print_EC_data(void) {                            //this function will pars the string  

	char sensorstring_array[30];                        //we make a char array
	char *EC;                                           //char pointer used in string parsing
	char *TDS;                                          //char pointer used in string parsing
	char *SAL;                                          //char pointer used in string parsing
	char *GRAV;                                         //char pointer used in string parsing
	float f_ec;                                         //used to hold a floating point number that is the EC
	
	sensorstring.toCharArray(sensorstring_array, 30);   //convert the string to a char array 
	EC = strtok(sensorstring_array, ",");               //let's pars the array at each comma
	//TDS = strtok(NULL, ",");                            //let's pars the array at each comma
	SAL = strtok(NULL, ",");                            //let's pars the array at each comma
 // GRAV = strtok(NULL, ",");                           //let's pars the array at each comma

	Serial.print("EC:");                                //we now print each value we parsed separately
	Serial.println(EC);                                 //this is the EC value

 // Serial.print("TDS:");                               //we now print each value we parsed separately
	//Serial.println(TDS);                                //this is the TDS value

	Serial.print("SALINITY:");                               //we now print each value we parsed separately
	Serial.println(SAL);                                //this is the salinity value

 // Serial.print("GRAV:");                              //we now print each value we parsed separately
	//Serial.println(GRAV);                               //this is the specific gravity
	//Serial.println();                                   //this just makes the output easier to read
	ecValue = atof(EC);
	psu = atof(SAL);
 // Serial.print("Salinity:");                               //we now print each value we parsed separately
 // Serial.println(psu);  
	
//f_ec= atof(EC);                                     //uncomment this line to convert the char to a float
}

void initTemperatureSensors()
{
	digitalWrite(TemperatureEnablePin, LOW); // make sure all temperature are OFF
	delay(TEMPSENSOR_SETTLEDELAY);
}


//void printAddress(DeviceAddress deviceAddress)
//{
//	for (uint8_t i = 0; i < 8; i++)
//	{
//		// zero pad the address if necessary
//		if (deviceAddress[i] < 16) Serial.print("0");
//		Serial.print(deviceAddress[i], HEX);
//	}
//	Serial.println("");
//}


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
		// Serial.println(i);
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
	 // Serial.print(c);
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
//--------------------LTE----------------------------
//
//void sendDataLTE(){
//Serial.println("start LTE.");  
//    delay(2000);
//    
//    bool connection = startLTE();
//    if (connection){
//          delay(5000);
//          enqueueGPS();   
//          enqueueTemperature();
//          enqueueEC();
//    
//            // Connection Status
//          uint8_t connectionStatus = sakuraio.getConnectionStatus();
//          Serial.print("LTE Status ");
//          Serial.println(connectionStatus);
//      
//          //getSignalQuality
//          int32_t signalQuality = sakuraio.getSignalQuality();
//          Serial.print("LTE signal Quality ");
//          Serial.println(signalQuality);
//          enqueueLTEStrength(signalQuality);
//          
//          //send data
//          uint8_t st = sakuraio.send();
//          Serial.println(st);
//          Serial.println("sent data -------");
//          delay(3000);
//         //get data
//         
//      // Rx Queue
//        Serial.println("access server to get status -------");
//        uint8_t avail;
//        uint8_t queued;
//        uint8_t ret;
//        sakuraio.getRxQueueLength(&avail, &queued);
//        Serial.print("Rx Available=");
//        Serial.print(avail);
//        Serial.print(" Queued=");
//        Serial.println(queued);
//        bool restart = false;
//        for(uint8_t i=0; i<queued; i++){
//          uint8_t channel;
//          uint8_t type;
//          uint8_t values[8];
//          int64_t offset;
//          ret = sakuraio.dequeueRx(&channel, &type, values, &offset);
//          Serial.print("Dequeue ");
//          Serial.print(ret);
//          if(ret == 0x01){
//            Serial.print(" ch="); Serial.print(channel);
//            Serial.print(" type="); Serial.print((char)type);
//            Serial.print(" values=[");
//            for(uint8_t b=0; b<7; b++){
//              Serial.print(values[b]);
//              Serial.print(" ");
//            }
//            Serial.print(values[7]);
//            Serial.print("] offset="); Serial.println((int32_t)offset);
//            if(values[0]==1){
//              restart = true;
//              Serial.println("------------------Restart-------------------------");
//              software_reset();
//            }
//          }else{
//            Serial.println(" ERROR");
//          }
//        }
//          
//        delay(10000);
//    }
//}
//
//bool startLTE(){
//    Serial.println("Trying to establish connection of sakura module.");
//
//  for (int i =0; i < LTEtry ; i++)
//  {
//    if ((sakuraio.getConnectionStatus() & 0x80) == 0x80)
//    {
//      Serial.println("");
//      Serial.println("Connected!");
//      return true;
//    }
//   Serial.print(i);
//   Serial.print(" ");
//   delay(1000);
//  }
//  return false; 
//}
//
//void enqueueTemperature()
//{
//  sakuraio.enqueueTx((uint8_t)4, temperatureValueOnboard);
//  sakuraio.enqueueTx((uint8_t)5, temperatureValueOutside);
//  sakuraio.enqueueTx((uint8_t)6, temperatureValueOnboard);
//  sakuraio.enqueueTx((uint8_t)7, meters);//preparation
//}
//
//void enqueueEC()
//{
//  sakuraio.enqueueTx((uint8_t)3, psu);
//}
//
//void enqueueGPS()
//{
//  sakuraio.enqueueTx((uint8_t)0, numSatellites);
//  sakuraio.enqueueTx((uint8_t)1, lat);
//  sakuraio.enqueueTx((uint8_t)2, lng);
//}
//
//void enqueueLTEStrength(int32_t strength)
//{
//  sakuraio.enqueueTx((uint8_t)8, strength);
//}

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
//		float temp1 = temperatureValueOnboard;
//		float temp2 = temperatureValueOutside;
//		float temp3 = ecValue;
		float salinity = psu;

		 Serial.println("Temperatures -----------");
//		 Serial.println(temperatureValueOnboard, 6);
//		 Serial.println(temperatureValueOutside, 6);
		 Serial.println(temp1, 2);
		 Serial.println(temp2, 2);
		 Serial.println(temp3, 2);
		 Serial.println("Salinity -----------");
		 Serial.println(psu, 6);
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

		float2ByteString(salinity, str);
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



			 // デバイスの準備ができるのを待つ
		while (!LoRaWAN.getReady()) {
				printResult();
				Serial.println(F("=getReady:0"));
				delay(1000);
		}
		printResult();

		// 工場出荷時リセットを試す
		// 成功すれば真
		f = LoRaWAN.factoryReset();
		Serial.print(F("=factoryReset:")); Serial.println(f);
		printResult();

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
	// Overwrite bytes of union with float variable
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
