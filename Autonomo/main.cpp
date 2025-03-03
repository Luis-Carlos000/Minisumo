#include <Arduino.h>
#include <SPI.h>
#include <InterCom.h>
#include "BluetoothSerial.h"

BluetoothSerial BT_CL;
SimpleCommand CMD_CL;

float ST = 0;
// Velocidades predeterminadas para búsqueda y ataque
#define V_SEARCH 50
#define V_ATACK 255

int VL = 0; // Velocidad motor izquierdo
int VR = 0; // Velocidad motor derecho

// Definiciones de pines para LEDs y botones
#define LED_ON 18
#define LED_OFF 17
#define LED_ST1 21
#define LED_ST2 19
#define LED_ST3 22
#define PBL 4
#define PBR 2
// #define GO 39

// Definiciones de pines para control de motores
#define FML 14
#define BML 27
#define FMR 13
#define BMR 12

// Definiciones de pines de sensores de proximidad

#define LLS 32 // Sensor Izquierda
#define FS 33  // Sensor Centro
#define RLS 26 // Sensor Derecha

// Variables para el control PID
int16_t previousError = 0;
int32_t integral = 0;
uint32_t lastPIDTime = 0;
bool sensorValues[3];

// Constantes de PID
#define KP 100
#define KI 0
#define KD 100
#define SCALE_FACTOR 100
#define MULTIPLICADOR_LADOS 1
#define BASE_SPEED 70
#define BASE_SPEED_FRONT V_ATACK

// Definiciones para la máquina de estados
#define TIEMPO_DESORIENTADO_MAX 2000 // Tiempo maximo en milisegundos en que puede permanecer en modo PID sin detectar nada

// Estado actual de la máquina de estados
uint8_t
	Estado = 0;
bool GoVal = 1;

// Variables para medir el tiempo sin detecciones en la funcion de calculo de error
uint32_t lastSenseTime = 0;
uint32_t currentTimeSense = 0;
uint32_t deltaSenseTime = 0;

// Definiciones para la funcion de busqueda
#define duracionGiro 500 // milisegundos para duración de cada giro en la funcion de busqueda

// Variables para la funcion de busqueda
uint32_t tiempoAnterior = 0;
uint32_t tiempoActual = 0;
bool girandoDerecha = true;

void COMMAND_CONFIGURATION()
{
	BT_CL.begin(115200);
	CMD_CL.begin(&BT_CL);
	CMD_CL.enable_echo(true);
	CMD_CL.addCommand("S", &ST);
}

// Función para detener todos los motores
void STOP()
{
	analogWrite(FML, 0);
	analogWrite(BML, 0);
	analogWrite(FMR, 0);
	analogWrite(BMR, 0);
}

// Función para girar a la izquierda con una velocidad variable
void GIRAR_IZQUIERDA(int pwm)
{
	analogWrite(FML, pwm);
	analogWrite(BML, 0);
	analogWrite(FMR, 0);
	analogWrite(BMR, pwm);
}

// Función para girar a la derecha con una velocidad variable
void GIRAR_DERECHA(int pwm)
{
	analogWrite(FML, 0);
	analogWrite(BML, pwm);
	analogWrite(FMR, pwm);
	analogWrite(BMR, 0);
}

// Función para avanzar con una velocidad variable
void AVANZAR(int pwm)
{
	analogWrite(FML, 0);
	analogWrite(BML, pwm);
	analogWrite(FMR, pwm);
	analogWrite(BMR, 0);
}

// Función de búsqueda en abanico
void SEARCH(int velocidad)
{
	tiempoActual = millis();
	// Cambiar de dirección después de girar un lado durante el tiempo especificado
	if (tiempoActual - tiempoAnterior >= duracionGiro)
	{
		tiempoAnterior = tiempoActual;	  // Actualizar el tiempo de referencia
		girandoDerecha = !girandoDerecha; // Cambiar el sentido del giro
	}

	// Realizar el giro según el estado de `girandoDerecha`
	if (girandoDerecha)
	{
		GIRAR_DERECHA(velocidad);
	}
	else
	{
		GIRAR_IZQUIERDA(velocidad);
	}
}

// Función para leer sensores y calcular el error para el control PID
int16_t calculateError()
{
	// Pesos ajustados para cada sensor
	const int16_t weights[3] = {-250, 0, 250};
	int16_t weightedSum = 0;
	uint8_t activeSensors = 0;

	// Calcular la suma ponderada basada en los sensores activos
	for (uint8_t i = 0; i < 3; i++)
	{
		if (sensorValues[i])
		{
			weightedSum += weights[i];
			activeSensors++;
		}
	}

	if (activeSensors > 0)
	{
		deltaSenseTime = 0;
		currentTimeSense = millis();
		lastSenseTime = currentTimeSense;
	}
	else
	{
		currentTimeSense = millis();
		deltaSenseTime = currentTimeSense - lastSenseTime;
	}

	// Si no hay sensores activos, reutilizar el error anterior
	return (activeSensors > 0) ? weightedSum / activeSensors : previousError;
}

// Función de control PID para el seguimiento de línea
void pidControl()
{
	uint32_t currentTime = millis();
	uint32_t deltaTime = currentTime - lastPIDTime;
	lastPIDTime = currentTime;

	int16_t error = calculateError();
	integral += error * deltaTime;

	// Calcular la corrección PID
	int32_t proportional = KP * error;
	int32_t derivative = (error - previousError) * KD / deltaTime;
	int32_t correction = (proportional + (KI * integral) + derivative) / SCALE_FACTOR;

	// Calcular velocidades para los motores
	int16_t speedLeft = 0;
	int16_t speedRight = 0;
	if (sensorValues[1])
	{
		speedLeft = BASE_SPEED_FRONT;
		speedRight = BASE_SPEED_FRONT;
	}
	else
	{
		if ((sensorValues[0] || sensorValues[2]) && (!(sensorValues[1])))
		{
			speedLeft = BASE_SPEED + (correction * MULTIPLICADOR_LADOS);
			speedRight = BASE_SPEED - (correction * MULTIPLICADOR_LADOS);
		}
		else
		{
			speedLeft = BASE_SPEED + correction;
			speedRight = BASE_SPEED - correction;
		}
	}
	/* speedLeft = BASE_SPEED + correction;
	speedRight = BASE_SPEED - correction; */
	bool directionLeft = speedLeft >= 0;
	bool directionRight = speedRight >= 0;

	// Aplicar las velocidades a los motores
	if (directionLeft)
	{
		analogWrite(FML, constrain(abs(speedLeft), 0, 255));
		analogWrite(BML, 0);
	}
	else
	{
		analogWrite(FML, 0);
		analogWrite(BML, constrain(abs(speedLeft), 0, 255));
	}
	if (directionRight)
	{
		analogWrite(FMR, constrain(abs(speedRight), 0, 255));
		analogWrite(BMR, 0);
	}
	else
	{
		analogWrite(FMR, 0);
		analogWrite(BMR, constrain(abs(speedRight), 0, 255));
	}

	previousError = error;
}

// Configuración inicial de pines y comunicación serial
void setup()
{
	pinMode(LED_OFF, OUTPUT);
	pinMode(LED_ON, OUTPUT);
	pinMode(LED_ST1, OUTPUT);
	pinMode(LED_ST2, OUTPUT);
	pinMode(LED_ST3, OUTPUT);
	pinMode(FMR, OUTPUT);
	pinMode(BMR, OUTPUT);
	pinMode(FML, OUTPUT);
	pinMode(BML, OUTPUT);

	pinMode(LLS, INPUT);
	pinMode(FS, INPUT);
	pinMode(RLS, INPUT);
	// pinMode(GO, INPUT_PULLUP);

	pinMode(PBL, INPUT_PULLUP);
	pinMode(PBR, INPUT_PULLUP);

	// Serial.begin(9600);
	tiempoActual = millis();
	tiempoAnterior = tiempoActual;
	COMMAND_CONFIGURATION();
}

// Bucle principal de la máquina de estados
void loop()
{
	CMD_CL.listen();
	sensorValues[0] = digitalRead(LLS);
	sensorValues[1] = digitalRead(FS);
	sensorValues[2] = digitalRead(RLS);
	// sensorValues[3] = digitalRead(RLS);
	delay(10);
	/* GoVal = digitalRead(GO);
	delay(10);
	if (GoVal == 1)
	{
		Estado = 0;
		tiempoActual = millis();
		tiempoAnterior = tiempoActual;
	}
	else
	{
		Estado = 3;
	} */
	if (ST < 1)
	{
		STOP();
	}
	else
	{
		switch (Estado)
		{
		case 0:
			// Estado de búsqueda, girar hasta detectar algo en sensores
			if (sensorValues[0] || sensorValues[1] || sensorValues[2])
			{
				Estado = 1;
				lastPIDTime = millis();
				currentTimeSense = millis();
				lastSenseTime = currentTimeSense;
			}
			else
			{
				SEARCH(V_SEARCH);
			}
			break;

		case 1:
			// Estado de ataque con control PID
			if (deltaSenseTime > TIEMPO_DESORIENTADO_MAX)
			{ // Si pasa más de X milisegundos sin detección, regresa a búsqueda
				Estado = 0;
				tiempoActual = millis();
				tiempoAnterior = tiempoActual;
			}
			else
			{
				pidControl(); // Ejecutar control PID basado en sensores
			}
			break;

		case 3:
			STOP();
			break;

		default:
			STOP();
			break;
		}
	}
}
