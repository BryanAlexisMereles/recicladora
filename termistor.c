
#include <PID_v1.h>
#include <thermistor.h>                     //https://github.com/miguel5612/Arduino-ThermistorLibrary
#include <Wire.h>

#include <Arduino.h>



#define initialTemp 100                      //Define parameters for each edjustable setting
#define minTemp 20                      
#define maxTemp 300

const int temperaturePin = A2;              //Define the remaining IO pins for motor, pushbutton & thermistor
const int pwmPin = 13;

int loopTime = 50;                          //Define time for each loop cycle
unsigned long currentTime = 0;

double Kp = 80.0;                            //Define PID constants
double Ki = 35.0;
double Kd = 80.0;

thermistor therm1(temperaturePin,0);         //Connect thermistor on A2

double setpoint = initialTemp;               //Define PID variables & establish PID loop
double input, output;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);



void setup() 
{
  //Initialize serial communication
  Serial.begin(9600);


  
  //Configure PWM pin
  pinMode(pwmPin, OUTPUT);
  
  //Set the PID parameters
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(0, 255);
  
  //Read and set the initial input value
  input = therm1.analog2temp();
  
                  //Set the initial direction of motion for motor

  Serial.println("Setup complete");                       //Write to serial monitor to indicate the setup function is complete
}

void loop() 
{
  //Record the start time for the loop
  currentTime = millis();

  //Read the temperature
  input = therm1.analog2temp(); // read temperature
  
  //Compute the PID output
  pid.Compute();
  
  //Update the PWM output
  analogWrite(pwmPin, output);

  int temp = input;
  
  //Print the temperature and PWM output

  Serial.print(initialTemp);
  Serial.print(" ºC / ");
  Serial.print(temp);
  Serial.println(" ºC");

}
