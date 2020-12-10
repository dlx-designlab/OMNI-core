//This code was written to be easy to understand.
//Modify this code as you see fit.
//This code will output data to the Arduino serial monitor.
//Type commands into the Arduino serial monitor to control the EC circuit.
//This code was written in the Arduino 1.8.5 IDE
//An Arduino MEGA was used to test this code.
//This code was last tested 4/2018

//setting
//get temperature of water


////command
//// get salinity data
// R

////set temperature
//T,23.65

////get salinity data with temperature compensation
// RT,25

////continuous mode (every 1s)
//C,1
////stop
//C,0

////calibration
////dry calibration
//Cal,dry
////single point calibration
////Normal Standard Seawater (S = 35)
////53000µS
//Cal,53000

#include <OneWire.h>
#include <DallasTemperature.h>
#include <SoftwareSerial.h>                           //we have to include the SoftwareSerial library, or else we can't use it
#define ECrx 4                                         //define what pin rx is going to be
#define ECtx 5                                          //define what pin tx is going to be

SoftwareSerial ECserial(ECrx, ECtx);                      //define how the soft serial port is going to work

//temperature
const uint8_t  outsideTemperature = 10;
OneWire oneWire2(outsideTemperature);
DallasTemperature temperatureSensorsOut(&oneWire2);

String inputstring = "";                              //a string to hold incoming data from the PC
String sensorstring = "";                             //a string to hold the data from the Atlas Scientific product
boolean input_string_complete = false;                //have we received all the data from the PC
boolean sensor_string_complete = false;               //have we received all the data from the Atlas Scientific product


void setup() {                                        //set up the hardware
  Serial.begin(9600);                                 //set baud rate for the hardware serial port_0 to 9600
  ECserial.begin(9600);                                //set baud rate for software serial port_3 to 9600
  inputstring.reserve(10);                            //set aside some bytes for receiving data from the PC
  sensorstring.reserve(30);                           //set aside some bytes for receiving data from Atlas Scientific product

  temperatureSensorsOut.begin();
  Serial.println("please press [a] for getting temperature.");
}


void serialEvent() {                                  //if the hardware serial port_0 receives a char
  inputstring = Serial.readStringUntil(13);           //read the string until we see a <CR>
  input_string_complete = true;                       //set the flag used to tell if we have received a completed string from the 
   Serial.print(inputstring);
     if(inputstring == "a"){
      get_temperature();
    }
}


void serialEvent3() {                                 //if the hardware serial port_3 receives a char
  sensorstring = ECserial.readStringUntil(13);         //read the string until we see a <CR>
  sensor_string_complete = true;                      //set the flag used to tell if we have received a completed string from the PC
}


void loop() {                                         //here we go...

  if (input_string_complete == true) {                //if a string from the PC has been received in its entirety
    ECserial.print(inputstring);                       //send that string to the Atlas Scientific product
    ECserial.print('\r');                              //add a <CR> to the end of the string
    inputstring = "";                                 //clear the string
    input_string_complete = false;                    //reset the flag used to tell if we have received a completed string from the PC
  }

  if (ECserial.available() > 0) {                     //if we see that the Atlas Scientific product has sent a character
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

void get_temperature(){
  for(int i = 0; i < 5 ; i++){
  temperatureSensorsOut.requestTemperatures();
  float temperatureValueOutside = temperatureSensorsOut.getTempCByIndex(0);
   Serial.print("Temeperature");
   Serial.println(temperatureValueOutside);
   ECserial.print("T,");
   ECserial.print(temperatureValueOutside);
   ECserial.print("\r");
   delay(1000);
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
  TDS = strtok(NULL, ",");                            //let's pars the array at each comma
  SAL = strtok(NULL, ",");                            //let's pars the array at each comma
  GRAV = strtok(NULL, ",");                           //let's pars the array at each comma

  Serial.print("EC:");                                //we now print each value we parsed separately
  Serial.println(EC);                                 //this is the EC value

  Serial.print("TDS:");                               //we now print each value we parsed separately
  Serial.println(TDS);                                //this is the TDS value

  Serial.print("SAL:");                               //we now print each value we parsed separately
  Serial.println(SAL);                                //this is the salinity value

  Serial.print("GRAV:");                              //we now print each value we parsed separately
  Serial.println(GRAV);                               //this is the specific gravity
  Serial.println();                                   //this just makes the output easer to read

//f_ec= atof(EC);                                     //uncomment this line to convert the char to a float
}
