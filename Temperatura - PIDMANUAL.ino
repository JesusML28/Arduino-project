#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);  //Especificacion de la pantalla LCD
//Definicion de PID
double Setpoint = 35;             // Valor deseado del proceso
double Input, Output;             // Entrada y Salida del Sistema
double Kp = 2, Ki = 5, Kd = 400;  // Constantes del controlador PID
double ITerm, lastInput, maxLimit = 1000, minLimit = 0;

unsigned long currentTime, prevTime;  // Tiempo actual y Tiempo anterior
unsigned long timeInterval = 1000;    // Intervalo de tiempo en milisegundos

//Definicion de pines
#define sensor_temp 2  // Pin del sensor DS18B20
#define led_green 3    // Pin del Led Verde (Indicador de que el sistema esta funcionando)
#define switch_mode 4  // Interruptor para Iniciar o Detener el sistema de control.
#define led_red 5      // Pin del Led Rojo (Indicador de que el sistema esta Desactivado)
#define SSR 6          // relé de estado solido ON/OFF conectado a la Resistencia.
#define sensor_agua 7  // Boya encargada de detectar si hay la suficiente agua en el sistema para funcionar.
#define pot A0         // Ajustar manualmente el Setpoint(Valor deseado)

// Instancia a las clases OneWire y DallasTemperature ¡Necesarias para el sensor de temperatura!
OneWire oneWireObjeto(sensor_temp);
DallasTemperature sensorDS18B20(&oneWireObjeto);

void setup() {
  sensorDS18B20.begin();  // Inicializamos el sensor
  lcd.init();             // Inicializamos la LCD
  lcd.backlight();        //Inicializamos la retroiluminación de la LCD
  Serial.begin(115200);   //Establecemos la comunicación Serial a 115200 baudios
  //Configuracion de Entradas y Salidas (I/O)
  pinMode(led_green, OUTPUT);
  pinMode(led_red, OUTPUT);
  pinMode(SSR, OUTPUT);
  digitalWrite(SSR, LOW);
  pinMode(switch_mode, INPUT);
  pinMode(sensor_agua, INPUT);
  pinMode(sensor_temp, INPUT);

  // Una pequeña animación de Inicialización del sistema.
  lcd.setCursor(3, 0);
  lcd.print("CARGANDO");
  lcd.setCursor(3, 1);
  for (int i = 0; i < 8; i++) {
    lcd.print("*");
    delay(1000);
  }
  lcd.clear();
  // Antes de entrar al programa de ejecución se revisa por primera vez el estado de la estación, verificando sí tiene agua.
  while (agua() == 0) { delay(800); }  // Si no tiene agua se queda en un bucle hasta que llenen el tanque principal.
}

void loop() {
  lcd.clear();
  while (agua() == 1) {                      // Se revisa entre cada ejecución del programa Sí el sistema cuenta con agua, mientras exista la suficiente agua el sistema continua su ejecución.
    if (digitalRead(switch_mode) == HIGH) {  // Entramos al modo de AUTOMATICO donde se ejecuta toda la operacion del PID.
      lcd_display("AUTO", "RUNTIME   ");     // función encargada de mostrar en la LCD la mayor parte de la informacion.
      digitalWrite(led_green, HIGH);
      digitalWrite(led_red, LOW);
      pid();                    //función donde se encuentra toda la operación del PID
      control();                // función encargada de Encender o apagar la resistencia en función de la salida del PID
    } else {                    //MANUAL
      new_setpoint(&Setpoint);  // función que establece un nuevo setpoint en el mismo espacio de memoria de la variable a través de un puntero.
      lcd_display("MAN ", "STANDBY   ");
      digitalWrite(led_green, LOW);
      digitalWrite(led_red, HIGH);
      digitalWrite(SSR, LOW);
    }
  }

  delay(800);
}
void new_setpoint(double* sp) {
  *sp = map(analogRead(pot), 0, 1023, 30, 40);  // Rango de Setpoint 30 y 40 °C totalmente ajustable donde 30°C es la temperatura ambiente del agua en Barranquilla
}

void lcd_display(String md, String st) {
  lcd.setCursor(0, 0);
  lcd.print("Sp:");
  lcd.setCursor(4, 0);
  lcd.print((int)Setpoint);
  lcd.setCursor(7, 0);
  lcd.print("MD:");
  lcd.setCursor(11, 0);
  lcd.print(md);
  lcd.setCursor(0, 1);
  lcd.print("SV:");
  sensorDS18B20.requestTemperatures();
  int t = sensorDS18B20.getTempCByIndex(0);
  lcd.print(t);
  lcd.print(" ");
  lcd.setCursor(6, 1);
  lcd.print(st);
}
// función para verificar el agua en el sistema.
bool agua() {
  if (digitalRead(sensor_agua) == HIGH) {
    return 1;
  } else {
    lcd.setCursor(3, 0);
    lcd.print("NECESITA");
    lcd.setCursor(5, 1);
    lcd.print("AGUA");
    digitalWrite(SSR, LOW);
    return 0;
  }
}
//función de PID
void pid() {
  currentTime = millis();
  double timeChange = (double)(currentTime - prevTime);
  if (timeChange >= timeInterval) {
    sensorDS18B20.requestTemperatures();
    Input = sensorDS18B20.getTempCByIndex(0);  // Leer el valor del sensor
    double error = Setpoint - Input;
    /*
    La PID Manual es una trampa que probablemente  más  para principiantes que cualquier otra. Ocurre cuando el PID cree que puede hacer algo que no puede. 
    Por ejemplo, la salida PWM de un Arduino acepta valores de 0 a 255. Por defecto, el PID no sabe esto. Si cree que 300-400-500 funcionará
    probará esos valores esperando obtener lo que necesita. Dado que en realidad el valor está fijado en 255, seguirá intentando números 
    cada vez más altos sin llegar a ninguna parte. Una vez que se alcanza cualquiera de los límites, el pid deja de sumar (integrar) y se lleva a 0.
    */
    ITerm += (Ki * error) * 1;

    double dInput = (Input - lastInput) / 1;

    /*Compute PID Output*/
    Output = Kp * error + Ki * errSum + Kd * dInput;
    if (Output > maxLimit)
      ITerm -= Output - maxLimit;
    Output = maxLimit;
    else if (Output < minLimit)
      ITerm += minLimit - Output;
    Output = minLimit;
    if (ITerm > maxLimit) ITerm = maxLimit;
    else if (ITerm < minLimit) ITerm = minLimit;
    // la entrada es menor al setpoint entonces el motor funcionara por pwm si es superior se apaga, debido a que la señal entre 0 y 60 no tiene la suficiente corriente para que el motor gire.
    if (Input < Setpoint) {
      analogWrite(motor, Output);
    } else {
      digitalWrite(motor, LOW);
      Output = 0;
    }

    Serial.print(Input);
    Serial.print(",");
    Serial.println(Output);
    lastInput = Input;
    prevTime = currentTime;  // el tiempo actual almacenado en la variable pasa a ser un tiempo pasado o previo o anterior.
  }
}
void control() {
  /* Para controlar un sistema ON/OFF con PID el creador de la librería recomienda implementar una lógica diferente donde la salida del PID no es
       más que el tiempo en milisegundos en el sistema que va a estar encendido o apagado.
       Sí el tiempo actual es menor a la suma del tiempo anterior más la salida del PID o la salida es igual al tiempo de Intervalo y la entrada es menor al setpoint entonces la resistencia estará encendida 
       */

  if (((millis() <= (prevTime + Output)) || (Output == timeInterval)) && Input < Setpoint) {
    // Power on:
    digitalWrite(SSR, HIGH);

  } else {
    // Power off:
    digitalWrite(SSR, LOW);
  }
}
