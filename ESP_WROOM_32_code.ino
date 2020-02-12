/*

	Electrical Stimulation Circuit 
				+
	ESP_WROOM_32 microcontroller
	
	Controlled by bluetooth through Android APP 

*/

// Load libraries
#include "Arduino.h"
#include "analogWrite.h"
#include "BluetoothSerial.h"


 // microcontroller pins 
#define measure_pin 4        
#define boost_pin 5          
#define stimulation_pin  17  
// ADC read  PIN A6          

// ASCII characters
#define voltage_inc  38			//V
#define voltage_dec  70			//v
#define frequency_inc  22		//F	
#define frequency_dec  54		//f
#define Mode           29

// Check if Bluetooth configs are enabled
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// Bluetooth Serial object
BluetoothSerial SerialBT;

int command_number;
int Duty_cycle = 55;
int Voltage_read = 0;
int set_volt = 20;
int frequency = 5;
int Pulse_duration = 100;

char incomingChar;


bool Device_On = false;

float voltage = 0;

const int analogInPin = A6;


void setup()
{

  Serial.begin(115200);
  // Bluetooth device name
  SerialBT.begin("ESP32_eStim");
  Serial.println("The device started, now you can pair it with bluetooth!");

  pinMode(measure_pin, OUTPUT);
  pinMode(boost_pin, OUTPUT);
  pinMode(stimulation_pin, OUTPUT);
}

void loop()
{
  bluetooth();
  command(command_number);
  
  if (Device_On) device_on();
  
}

void device_on(){
	
	    feedback();
    if (incomingChar != '\n') {
      command(command_number);
    }
    stimulation();
    delay(200 / frequency);
    feedback();
    delay(200 / frequency);
    feedback();
    delay(200 / frequency);
    feedback();
    delay(200 / frequency);
    feedback();
    delay(200 / frequency);
    feedback();
	return;
}

void feedback()
{
  digitalWrite(measure_pin, HIGH);
  Voltage_read = analogRead(analogInPin);       // Read voltage from ADC pin 
  digitalWrite(measure_pin, LOW);

  voltage = ((float)Voltage_read * 3.3) / 4096.0;
  //Serial.print(voltage * 62);
  //Serial.println(" V");

  if ((voltage * 62) < set_volt)
  {
    analogWrite(boost_pin, Duty_cycle);        // driving mosfet-switch with pwm pulse with V_high = Duty_cycle
  }
  else
  {
    analogWrite(boost_pin, 0);					// When selected voltage reached, pwm stop
  }
  Voltage_read = 0;
}

void bluetooth()
{
  if (SerialBT.available())
  {
    incomingChar = SerialBT.read();
    if (incomingChar != '\n')command_number = int(incomingChar) - 48;
  }
}

void command(int c)
{

  switch (c)
  {

    case 1:
      Device_On = true;
      Serial.println("Device on");
      break;

    case 2:
      Device_On = false;
      analogWrite(boost_pin, 0);
      Serial.println("Device Off");
      digitalWrite(measure_pin, HIGH);
      delay(500);
      digitalWrite(measure_pin, LOW);
      break;

    case voltage_inc:
      if (set_volt < 100)set_volt += 2;
      Serial.print("output voltage = " );
      Serial.println(set_volt);
      if (set_volt > 50) Duty_cycle = 70;
      break;

    case voltage_dec:
      if (set_volt > 4)set_volt -= 2;
      Serial.print("output voltage = " );
      Serial.println(set_volt);
      break;

    case frequency_inc:
      if (frequency < 200) frequency++;
      Serial.print("frequency = " );
      Serial.println(frequency);
      break;

    case frequency_dec:
      if (frequency > 1) frequency--;
      Serial.print("frequency = " );
      Serial.println(frequency);
      break;
  }
  command_number = 0;
}

void stimulation()
{
  digitalWrite(stimulation_pin, HIGH);	// controlling the mosfet for the stimulation 
  delayMicroseconds(Pulse_duration);	// stimulation pulse duration in mseconds
  digitalWrite(stimulation_pin, LOW);
}
