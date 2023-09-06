/* Control velocidad y temperatura para una recicladora de PETG.
 * Una Arielada de Franco Soluciones & Bryan Mereles.
 */
#include <thermistor.h> 
#define B 3950 // B factor
#define RESISTOR 100000 // resistencia del resistor, 200 kOhm
#define THERMISTOR 100000 // resistencia nominal del termistor, 100 kOhm
#define NOMINAL 25 // temperatura nominal
 
#define sensor A0
#define    POT 10             // lectura del potenciometro o comando
#define Motor  8            //Encendido del motor
#define BZ     9     // Boozer
#define PWM_pin   13      // pin Bloque calefactor y ventilador

// Variables de temperatura
int TempF = 200;
float set_temperature = 200;            //Default temperature setpoint. Leave it 0 and control it with rotary encoder
float temperature_read = 0.0;
float PID_error = 0;
float previous_error = 0;
float elapsedTime, Time, timePrev;
float PID_value = 0;
thermistor therm1(A0,0);
int max_PWM = 255;

//PID constants
//////////////////////////////////////////////////////////
int kp = 90;   int ki = 30;   int kd = 80;
//////////////////////////////////////////////////////////

int PID_p = 0;    int PID_i = 0;    int PID_d = 0;
float last_kp = 0;
float last_ki = 0;
float last_kd = 0;

int PID_values_fixed =0;
/////////////////////////////////////////////////////////

void setup() {                
  
  // inicializamos pin como salidas.
  Serial.begin(9600);
  pinMode(5, OUTPUT); 
   pinMode(13, OUTPUT);
  
}

void loop() {
  
 //Lecturas
  char C = Serial.read();  //lee comandos de letras desde el celular por app inventor
  int  Temp = analogRead(A0);     // leemos el sensor de temperatura
int t = analogRead(sensor);
    float tr = 1023.0 / t - 1;
    tr = RESISTOR / tr;

      float steinhart;
    steinhart = tr / THERMISTOR;
    steinhart = log(steinhart);
    steinhart /= B;
    steinhart += 1.0 / (NOMINAL + 273.15);
    steinhart = 1.0 / steinhart;
    steinhart -= 273.15; 
///////////////////////////////////////////////////////////////////////////////////////////////////

// Control de Temperatura
  if(C == 'T') //si oprimo mas temperatura el objetivo de temperatura a alcanzar sube en +1
    {
      TempF= TempF + 1;
      }
     else if(C == 't')
{
      TempF= TempF - 1;
      }
set_temperature = TempF;
  // First we read the real value of temperature
  temperature_read = therm1.analog2temp(); // read temperature
  
  //Next we calculate the error between the setpoint and the real value
  PID_error = set_temperature - temperature_read + 6;
  //Calculate the P value
  PID_p = 0.01*kp * PID_error;
  //Calculate the I value in a range on +-6
  PID_i = 0.01*PID_i + (ki * PID_error);
  
  
  //For derivative we need real time to calculate speed change rate
  timePrev = Time;                            // the previous time is stored before the actual time read
  Time = millis();                            // actual time read
  elapsedTime = (Time - timePrev) / 1000; 
  //Now we can calculate the D calue
  PID_d = 0.01*kd*((PID_error - previous_error)/elapsedTime);
  //Final total PID value is the sum of P + I + D
  PID_value = PID_p + PID_i + PID_d;

  
  //We define PWM range between 0 and 255
  if(PID_value < 0){
    PID_value = 0;
  }
  if(PID_value > max_PWM){
    PID_value = max_PWM;
  }
  
  //Now we can write the PWM signal to the mosfet on digital pin D5
  analogWrite(PWM_pin,PID_value);
  previous_error = PID_error;     //Remember to store the previous error for next loop.
///////////////////////////////////////////////////////////////////////////////////////////////////

// Velocidad del bobinador
    if(C == 'V') //si oprimo aumentar velocidad el tiempo entre pasos debe ser mas pequeño
    {
      POT= POT - 5;
      }
     else if(C == 'v')
{
      POT= POT + 5;
      }
      int Vel= POT*-1; //obtengo el opuesto del valor POT para luego graficar un incremento de la velocidad como valor positivo y biceversa
//////////////////////////////////////////////////////////////////////////////////////////////////

// Encendido del motor de la bobinadora y parada    

    if(C =='A') {
    digitalWrite(5, HIGH);         // Aqui generamos un flanco de bajada HIGH - LOW
    //delayMicroseconds(5);              // Pequeño retardo para formar el pulso en STEP
   // digitalWrite(5, LOW);         // y el A4988 de avanzara un paso el motor
   // delayMicroseconds(POT); // generamos un retardo con el valor leido del potenciometro
   }
    
 else if (C =='a') digitalWrite (5, LOW);
  
///////////////////////////////////////////////////////////////////////////////////////////////////

//Imprimimos los datos en el monitor serial para control
 /* Serial.println (C);
  Serial.print ("Velocidad " );
  Serial.print (Vel );
  Serial.println ("%");
  Serial.print ("Temperatura Fijada " );
   Serial.print (TempF );
  Serial.println ("ºC");
*/   Serial.print ("Temperatura Real " );
  Serial.print (steinhart);
  Serial.println ("ºC");
 
 
 
 
 ///////////////////////////////////////////////////////////////////////////////////////////////////
  }
