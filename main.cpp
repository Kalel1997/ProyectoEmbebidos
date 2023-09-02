#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define ADELANTE 1
#define ATRAS 2
#define IZQUIERDA 3
#define DERECHA 4
#define STOP 0

#define MOTOR_TRASERO_DERECHO 0
#define MOTOR_TRASERO_IZQUIERDO 1
#define MOTOR_DELANTERO_DERECHO 2
#define MOTOR_DELANTERO_IZQUIERDO 3


LiquidCrystal_I2C lcd(PCF8574_ADDR_A21_A11_A01, 4, 5, 6, 16, 11, 12, 13, 14, POSITIVE); 

// Variables para el cálculo del tiempo y la distancia
bool carIsMoving = false;         // Indicador de si el carro está en movimiento
unsigned long startTime = 0.0;      // Tiempo en milisegundos cuando el carro comienza a moverse
unsigned long stopTime = 0.0;       // Tiempo en milisegundos cuando el carro se detiene
double totalDistance = 0.0;       // Distancia total recorrida en cm
double currentDistance = 0.0;     // Distancia recorrida durante el movimiento actual
double averageSpeed = 41.18;       // Velocidad promedio en cm/s

// float wheelDiameter = 6.604;  // Diámetro de la llanta en centímetros



struct MOTOR_PINS
{
  int pinIN1;
  int pinIN2;
  int pinEn; 
  int pwmSpeedChannel;
};

std::vector<MOTOR_PINS> motorPins = 
{
  {16, 17, 2, 4},  // MOTOR_TRASERO_DERECHO
  {18, 19, 23, 5},  // MOTOR_TRASERO_IZQUIERDO
  {26, 27, 14, 6},  // MOTOR_DELANTERO_DERECHO
  {33, 25, 32, 7},  // MOTOR_DELANTERO_IZQUIERDO   
};

#define MAX_MOTOR_SPEED 200

const int PWMFreq = 1000; /* 1 KHz */
const int PWMResolution = 8;

#define SIGNAL_TIMEOUT 1000  // This is signal timeout in milliseconds. We will reset the data if no signal
unsigned long lastRecvTime = 0;

struct PacketData
{
  byte xAxisValue;
  byte yAxisValue;
  byte zAxisValue;
};
PacketData receiverData;

const int trigPin = 4; // HC-SR04 Trigger pin connected to D4 (GPIO 4)
const int echoPin = 5; // HC-SR04 Echo pin connected to D5 (GPIO 5)
const int safeDistance = 20; // Safe distance in centimeters

// Function to measure the distance from the HC-SR04 sensor
int getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  int distance = duration * 0.034 / 2; // Calculate the distance in centimeters
  return distance;
}

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) 
{
  if (len == 0)
  {
    return;
  }
  memcpy(&receiverData, incomingData, sizeof(receiverData));
  String inputData ;
  inputData = inputData + "values " + receiverData.xAxisValue + "  " + receiverData.yAxisValue + "  " + receiverData.zAxisValue;
  Serial.println(inputData);

  int obstacleDistance = getDistance();
  Serial.print("Obstacle Distance: ");
  Serial.println(obstacleDistance);

  if (obstacleDistance <= safeDistance) {
    processCarMovement(STOP);
  }
  else if ( receiverData.xAxisValue < 75 && receiverData.yAxisValue < 75)
  {
    processCarMovement(ADELANTE);    
  }
  else if ( receiverData.xAxisValue > 175 && receiverData.yAxisValue < 75)
  {
    processCarMovement(DERECHA);    
  } 
  else if ( receiverData.xAxisValue < 75 && receiverData.yAxisValue > 175)
  {
    processCarMovement(ATRAS);    
  }
  else if ( receiverData.xAxisValue > 175 && receiverData.yAxisValue > 175)
  {
    processCarMovement(IZQUIERDA);    
  }  
  else if (receiverData.zAxisValue > 175)
  {
    processCarMovement(DERECHA);
  }
  else if (receiverData.zAxisValue < 75)
  {
    processCarMovement(IZQUIERDA);
  }
  else if (receiverData.yAxisValue < 75)
  {
    processCarMovement(ADELANTE);  
  }
  else if (receiverData.yAxisValue > 175)
  {
    processCarMovement(ATRAS);     
  }
  else if (receiverData.xAxisValue > 175)
  {
    processCarMovement(DERECHA);   
  }
  else if (receiverData.xAxisValue < 75)
  {
    processCarMovement(IZQUIERDA);    
  } 
  else
  {
    processCarMovement(STOP);     
  }

  lastRecvTime = millis();   
}

void processCarMovement(int inputValue)
{
  unsigned long currentTime = millis();

  if (inputValue == STOP) {
    if (carIsMoving) {
      unsigned long elapsedTime = currentTime - startTime;
      currentDistance += (elapsedTime / 1000.0) * averageSpeed; // Calcular la distancia en segundos
      carIsMoving = false;
      stopTime = currentTime; // Guardar el tiempo de detención
    }
  } else {
    if (!carIsMoving) {
      startTime = currentTime;
      carIsMoving = true;
      currentDistance = 0.0; // Restablecer la distancia actual al comenzar el movimiento
    }
  }

  switch (inputValue)
  {
    case ADELANTE:
      rotateMotor(MOTOR_DELANTERO_DERECHO, MAX_MOTOR_SPEED);
      rotateMotor(MOTOR_TRASERO_DERECHO, MAX_MOTOR_SPEED);
      rotateMotor(MOTOR_DELANTERO_IZQUIERDO, MAX_MOTOR_SPEED);
      rotateMotor(MOTOR_TRASERO_IZQUIERDO, MAX_MOTOR_SPEED);                  
      break;
  
    case ATRAS:
      rotateMotor(MOTOR_DELANTERO_DERECHO, -MAX_MOTOR_SPEED);
      rotateMotor(MOTOR_TRASERO_DERECHO, -MAX_MOTOR_SPEED);
      rotateMotor(MOTOR_DELANTERO_IZQUIERDO, -MAX_MOTOR_SPEED);
      rotateMotor(MOTOR_TRASERO_IZQUIERDO, -MAX_MOTOR_SPEED);   
      break;
  
    case IZQUIERDA:
      rotateMotor(MOTOR_DELANTERO_DERECHO, MAX_MOTOR_SPEED);
      rotateMotor(MOTOR_TRASERO_DERECHO, -MAX_MOTOR_SPEED);
      rotateMotor(MOTOR_DELANTERO_IZQUIERDO, -MAX_MOTOR_SPEED);
      rotateMotor(MOTOR_TRASERO_IZQUIERDO, MAX_MOTOR_SPEED);   
      break;
  
    case DERECHA:
      rotateMotor(MOTOR_DELANTERO_DERECHO, -MAX_MOTOR_SPEED);
      rotateMotor(MOTOR_TRASERO_DERECHO, MAX_MOTOR_SPEED);
      rotateMotor(MOTOR_DELANTERO_IZQUIERDO, MAX_MOTOR_SPEED);
      rotateMotor(MOTOR_TRASERO_IZQUIERDO, -MAX_MOTOR_SPEED);  
      break;
  
    case STOP:
      rotateMotor(MOTOR_DELANTERO_DERECHO, STOP);
      rotateMotor(MOTOR_TRASERO_DERECHO, STOP);
      rotateMotor(MOTOR_DELANTERO_IZQUIERDO, STOP);
      rotateMotor(MOTOR_TRASERO_IZQUIERDO, STOP);    
      break;
  
    default:
      rotateMotor(MOTOR_DELANTERO_DERECHO, STOP);
      rotateMotor(MOTOR_TRASERO_DERECHO, STOP);
      rotateMotor(MOTOR_DELANTERO_IZQUIERDO, STOP);
      rotateMotor(MOTOR_TRASERO_IZQUIERDO, STOP);    
      break;
  }
}



void rotateMotor(int motorNumber, int motorSpeed)
{
  if (motorSpeed < 0)
  {
    digitalWrite(motorPins[motorNumber].pinIN1, LOW);
    digitalWrite(motorPins[motorNumber].pinIN2, HIGH);    
  }
  else if (motorSpeed > 0)
  {
    digitalWrite(motorPins[motorNumber].pinIN1, HIGH);
    digitalWrite(motorPins[motorNumber].pinIN2, LOW);       
  }
  else
  {
    digitalWrite(motorPins[motorNumber].pinIN1, LOW);
    digitalWrite(motorPins[motorNumber].pinIN2, LOW);      
  }
  
  ledcWrite(motorPins[motorNumber].pwmSpeedChannel, abs(motorSpeed));
}

void setUpPinModes()
{
  for (int i = 0; i < motorPins.size(); i++)
  {
    pinMode(motorPins[i].pinIN1, OUTPUT);
    pinMode(motorPins[i].pinIN2, OUTPUT);  
    //Set up PWM for motor speed
    ledcSetup(motorPins[i].pwmSpeedChannel, PWMFreq, PWMResolution);  
    ledcAttachPin(motorPins[i].pinEn, motorPins[i].pwmSpeedChannel);     
    rotateMotor(i, STOP);  
  }
}

void setup() 
{
  setUpPinModes();
  
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) 
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
  
  lcd.begin();                      // Inicializar la pantalla LCD
  lcd.backlight();                 // Encender la retroiluminación
  lcd.setCursor(0, 0);
  lcd.print("Tiempo: ");
  lcd.setCursor(0, 1);
  lcd.print("Distancia: ");

}
 
void loop() {
  // Check Signal lost.
  unsigned long now = millis();
  if (now - lastRecvTime > SIGNAL_TIMEOUT) {
    processCarMovement(STOP);
  }

  // Verificar si el carro está en movimiento
  if (carIsMoving) {
    // Calcular el tiempo transcurrido desde que comenzó el movimiento
    unsigned long elapsedTime = now - startTime;

    // Calcular la distancia recorrida en cm usando la velocidad promedio
    double currentDistance = averageSpeed * (elapsedTime / 1000.0);  // Dividido por 1000 para convertir de ms a s

    // Actualizar la distancia total recorrida
    totalDistance = currentDistance;

    // Mostrar el tiempo y la distancia en la pantalla LCD
    lcd.setCursor(8, 0);
    lcd.print(elapsedTime / 1000.0);  // Mostrar el tiempo en segundos
    lcd.setCursor(10, 1);
    lcd.print(totalDistance, 2);    // Mostrar la distancia con 2 decimales
    lcd.print(" cm");
  } else {
    // Mostrar el tiempo y la distancia acumulada en la pantalla LCD
    lcd.setCursor(8, 0);
    lcd.print((stopTime - startTime) / 1000.0); // Mostrar el tiempo en segundos
    lcd.setCursor(10, 1);
    lcd.print(totalDistance, 2); // Mostrar la distancia acumulada con 2 decimales
    lcd.print(" cm");
  }
  
 
}