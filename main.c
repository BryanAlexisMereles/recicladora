/* Control velocidad y temperatura para una recicladora de PETG.
 * Una Arielada de Franco Soluciones & Bryan Mereles.
 */
////////////////////////////////////Librerias
#include <PID_v1.h>
#include <thermistor.h>                     //https://github.com/miguel5612/Arduino-ThermistorLibrary

///////////////////////////////////Control de temperatura y lectura del termistor

#define initialTemp 200                      //Define parameters for each edjustable setting
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


/////////////////////////////////////////////////////Control del Motor

const int analogInPin = A1;  // Analog input pin that the potentiometer is attached to
const int motor = 5; // Analog output pin that the LED is attached to
int MOUT = 5;
int Vel = 300;        // value read from the pot
int outputValue = 0; 


/////////////////////////////////////////////////////Botones
int V;
int v;
int T;
int t;

/////////////////////////////////////////////////////


void setup() 
{
  //Initialize serial communication
Serial.begin(9600);

///////////////////////////////////////////////////Control de Temperatura  
  //Configure PWM pin
  pinMode(pwmPin, OUTPUT);
  
  //Set the PID parameters
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(0, 255);
  
  //Read and set the initial input value
  input = therm1.analog2temp();


///////////////////////////////////////////////////Motor               
pinMode(MOUT, OUTPUT);
     
}

void loop() 
{

///////////////////////////////////////////////////Botones
 char C = Serial.read();  //lee comandos de letras desde el celular por app inventor

if(C == 'T') //si oprimo mas temperatura el objetivo de temperatura a alcanzar sube en +1
    {
      setpoint++;
      }
     else if(C == 't')
     {
      setpoint--;
      }

 if(C == 'V') //si oprimo mas temperatura el objetivo de temperatura a alcanzar sube en +1
    {
      Vel= Vel+10;
      }
     else if(C == 'v')
     {
      Vel= Vel-10;
      }
///////////////////////////////////////////////////Control de Temperatura  
  //Record the start time for the loop
  currentTime = millis();

  //Read the temperature
  input = therm1.analog2temp(); // read temperature
  
  //Compute the PID output
  pid.Compute();
  
  //Update the PWM output
  analogWrite(pwmPin, output);


///////////////////////////////////////////////////Control de Motor
//  Vel = analogRead(analogInPin);
  // map it to the range of the analog out:
  outputValue = map(Vel, 0, 1023, 0, 255);
  // change the analog out value:
  analogWrite(MOUT, outputValue);   


///////////////////////////////////////////////////Envio de datos por KornFlakers 
//escalados y estetica de los datos
int TFij=setpoint;
int TReal=input;
int Velocidad= Vel/10;

     Serial.print(Velocidad);
    Serial.println ("; ");
 Serial.print(TFij);
     Serial.println ("; ");
  Serial.print(TReal);
  Serial.println("; ");
delay (40);
}
