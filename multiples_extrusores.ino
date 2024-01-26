const int termistorPin1 = A0;
const int termistorPin2 = A1;
const int termistorPin3 = A2;   
const int salidaPin1 = 11;     // Pin digital para la salida
const int salidaPin2 = 12;
const int salidaPin3 = 13;

// Cargar la temperatura por defecto a la que quieres que se mantengan los extrusores
int temperaturaRequerida = 207;
  

// Valores de resistencia conocidos del termistor y la resistencia de referencia
const float resistenciaConocida = 100000.0;  // Resistencia del termistor en ohmios (100k)
const float resistenciaReferencia = 4700;  // Resistencia de referencia en ohmios (4.7k)

void setup() {
  Serial.begin(9600);
  pinMode(salidaPin1, OUTPUT);
  pinMode(salidaPin2, OUTPUT);
  pinMode(salidaPin3, OUTPUT);
}

void loop() {
  // Bluetooth

   char C = Serial.read();  //lee comandos de letras desde el celular por app inventor

  if(C == 'T') //si oprimo mas temperatura el objetivo de temperatura a alcanzar sube en +1
    {
      temperaturaRequerida++;
      }
     else if(C == 't')
     {
      temperaturaRequerida--;
      }

  // Leer los valores de los termistores
  int valorTermistor1 = analogRead(termistorPin1);
  int valorTermistor2 = analogRead(termistorPin2);
  int valorTermistor3 = analogRead(termistorPin3);

  // Convertir el valor a resistencia usando la ley de Ohm
  float resistenciaTermistor1 = resistenciaReferencia / (1023.0 / valorTermistor1 - 1.0);
  float resistenciaTermistor2 = resistenciaReferencia / (1023.0 / valorTermistor2 - 1.0);
  float resistenciaTermistor3 = resistenciaReferencia / (1023.0 / valorTermistor3 - 1.0);

  // Calcular la temperatura en grados Celsius utilizando la ecuación de Steinhart-Hart
  float temperaturaCelsius1 = 1.0 / ((log(resistenciaTermistor1 / resistenciaConocida) / 3950.0) + (1.0 / 298.15)) - 273.15;
  float temperaturaCelsius2 = 1.0 / ((log(resistenciaTermistor2 / resistenciaConocida) / 3950.0) + (1.0 / 298.15)) - 273.15;
  float temperaturaCelsius3 = 1.0 / ((log(resistenciaTermistor3 / resistenciaConocida) / 3950.0) + (1.0 / 298.15)) - 273.15;

  // Mostrar la temperatura en el puerto serie (puedes omitir esto si no es necesario)
  Serial.print("Temperatura 1: ");
  Serial.print(temperaturaCelsius1);
  Serial.println("C");
  Serial.print("Temperatura 2: ");
  Serial.print(temperaturaCelsius2);
  Serial.println("C");
  Serial.print("Temperatura 3: ");
  Serial.print(temperaturaCelsius3);
  Serial.println("C");
  Serial.print("Temperatura objetivo:");
  Serial.println(temperaturaRequerida);


  // Lógica para activar o desactivar la salida en función de la temperatura

  // Extrusor 1
  if (temperaturaCelsius1 < temperaturaRequerida) {
    digitalWrite(salidaPin1, HIGH);  // Activar la salida
  } else{
    digitalWrite(salidaPin1, LOW);   // Desactivar la salida
  }
  // Extrusor 2
  if (temperaturaCelsius2 < temperaturaRequerida) {
    digitalWrite(salidaPin2, HIGH);  // Activar la salida
  } else{
    digitalWrite(salidaPin2, LOW);   // Desactivar la salida
  }
  // Extrusor 3
  if (temperaturaCelsius3 < temperaturaRequerida) {
    digitalWrite(salidaPin3, HIGH);  // Activar la salida
  } else{
    digitalWrite(salidaPin3, LOW);   // Desactivar la salida
  }


    delay(100);
}
