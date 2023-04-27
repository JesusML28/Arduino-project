#include <LiquidCrystal_I2C.h>
#include <NewPing.h>
#include <PID_v2.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);  //especificación de la pantalla LCD
//Definicion de pines
#define pot A0         // Ajustar manualmente el Setpoint(Valor deseado)
#define led_green 3    // Pin del Led Verde (Indicador de que el sistema esta funcionando)
#define switch_mode 6  // Interruptor para Iniciar o Detener el sistema de control.
#define led_red 4      // Pin del Led Rojo (Indicador de que el sistema esta Desactivado)
#define motor 5        // Señal PWM para el motor
#define sensor_agua 2  // Boya encargada de detectar si hay la suficiente agua en el sistema para funcionar.
//Definicion del sensor ultrasónico
#define trigger 13  // Pulso de Activación
#define echo 12     // Receptor
//Máximos y mínimos

//Lectura del ultrasonico
udouble8_t ultrasonico() {
  digitalWrite(trigger, HIGH);  // Establecemos el valor del pin HIGH(5V)
  delayMicroseconds(1000);      // Espera un segundo
  digitalWrite(trigger, LOW);   // Valor del pin LOW(0V)

  udouble8_t d = pulseIn(echo, HIGH);  //Para leer cuando Echo sea HIGH
  d = (d / 2) / 29.1;                  //Ecuación
  return map(d, 0, 23, 23, 0);   // invertimos los máximos y mínimos con la funcion map ( Evitar problemas con el calculo PID)
}
//Nextion
uint8_t setpoint = 0, setpointprev = 0, setpointprev2 = 0;
uint8_t status = 0;

//Definicion de PID
udouble8_t Setpoint = 0;             // Valor deseado del proceso
udouble8_t Input, Output;            // Entrada y Salida del Sistema
udouble8_t Kp = 10, Ki = 2, Kd = 1;  // Constantes del controlador PID

unsigned long currentTime, prevTime;  // Tiempo actual y Tiempo anterior
unsigned long timeInterval = 100;     // Intervalo de tiempo en milisegundos
// Inicializamos el PID, con la siguiente parámetros (Entrada, Salida, Valor Deseado, Constante Proporcional, Integral, Derivativa, MODO);
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
void setup() {

  Serial.begin(9600);       //Establecemos la comunicación Serial a 9600 baudios
  new_setpoint(&Setpoint);  // Inicializamos el Setpoint en las pantallas.
  lcd.init();               // Inicializamos la LCD
  lcd.backlight();          //Inicializamos la retroiluminación de la LCD
  //Configuracion de Entradas y Salidas (I/O)
  pinMode(led_green, OUTPUT);
  pinMode(led_red, OUTPUT);
  pinMode(motor, OUTPUT);
  pinMode(trigger, OUTPUT);
  pinMode(echo, INPUT);
  pinMode(switch_mode, INPUT);
  pinMode(sensor_agua, INPUT);
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(60, 255);     // Establecemos los límites de la salida del PID
  myPID.SetSampleTime(timeInterval);  // Tiempo de lectura entre calculo algorítmico del sistema
  prevTime = millis();                // Inicializamos por primera vez la variable con la función millis() encargada de dar el tiempo actual de ejecución.
  // Una pequeña animación de Inicialización del sistema.
  lcd.setCursor(3, 0);
  lcd.print("CARGANDO");
  lcd.setCursor(3, 1);
  for (uint8_t i = 0; i < 8; i++) {
    lcd.print("*");
    delay(1000);
  }
  lcd.clear();
  // Antes de entrar al programa de ejecución se revisa por primera vez el estado de la estación, verificando sí tiene agua.
  while (agua() == 0) { delay(800); }  // Si no tiene agua se queda en un bucle hasta que llenen el tanque principal.
}

void loop() {
  lcd.clear();
  // Se revisa entre cada ejecución del programa Sí el sistema cuenta con agua
  //mientras exista la suficiente agua el sistema continua su ejecución.
  while (agua() == 1) {
    if (digitalRead(switch_mode) == HIGH && status == 0) {  // Entramos al modo de ejecución donde se ejecuta toda la operación del PID.
      lcd_display("AUTO", "RUNTIME   ");                    // función encargada de mostrar en la LCD la mayor parte de la información.
      digitalWrite(led_green, HIGH);
      digitalWrite(led_red, LOW);
      pid();  //función donde se encuentra toda la operación del PID

    } else {

      new_setpoint(&Setpoint);  // función que establece un nuevo setpoint en el mismo espacio de memoria de la variable a través de un puntero.
      nextion_process();        // función donde esta toda la lógica y implementación de la nextion con la estación.
    }
  }
  delay(800);
}

void new_setpoint(udouble8_t* sp) {
  uint8_t setpot = map(analogRead(pot), 0, 1023, 0, 15);  // Rango de Setpoint de 0 a 15 cm Tamaño máximo 19 cm Superior a ese valor el ultrasónico es inestable.
  leer_setpoint(&setpoint);
  if (setpoint != setpointprev) {
    *sp = setpoint;

    setpointprev = setpoint;
  }
  if (setpot != setpointprev2) {
    *sp = setpot;
    setpointprev2 = setpot;
  }
}
/*
En esta función se creó unos casos donde 
Case 0: es Modo de espera, Simplemente proyecta el valor del sensor y permite la lectura de un nuevo setpoint.
Case 1: Modo de Ejecución donde se puede visualizar la gráfica en la Nextion en función del nivel
*/

void nextion_process() {

  switch (status) {
    case 0:
      lcd_display("MAN ", "STANDBY   ");
      digitalWrite(led_green, LOW);
      digitalWrite(led_red, HIGH);
      digitalWrite(motor, LOW);
      break;
    case 1:
      lcd_display("AUTO", "RUNTIME   ");  // función encargada de mostrar en la LCD la mayor parte de la información.
      digitalWrite(led_green, HIGH);
      digitalWrite(led_red, LOW);
      pid();  //función donde se encuentra toda la operación del PID
      break;
    default:
      break;
  }
}
}
//Nextion Función para actualizar el componente Waveform
void updateWaveform(String(waveform_channel), String(waveform_value)) {
  // Variables para el componente Waveform
  // waveform_channel tiene 2  Canales (0 o 1) del componente Waveform
  // waveform_value  Valor a agregar al componente Waveform
  // Envía el comando para agregar un punto al componente Waveform
  String command = "add " + String(2) + "," + waveform_channel + "," + waveform_value;
  Serial.print(command);
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);
}
//Nextion
void mostrar_nextion(uint8_t sensor) {
  //Sensor
  Serial.print("sv_n.val=");
  Serial.print(sensor);
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff));
  //Setpoint
  Serial.print("setpoint_n.val=");
  Serial.print(Setpoint);
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);
}
//Nextion
void leer_setpoint(uint8_t* value) {
  String st = Serial.readString();

  if (st[0] == 'b' && st[1] == 'n') {
    char mode = st[2];
    if (mode == 'a') {
      *value = uint8_t(st[3]);
      status = 1;
    } else {
      status = 0;
    }
  }
}

void lcd_display(String md, String st) {
  lcd.setCursor(0, 0);
  lcd.print("Sp:");
  lcd.setCursor(4, 0);
  lcd.print((uint8_t)Setpoint);
  lcd.print(" ");
  lcd.setCursor(7, 0);
  lcd.print("MD:");
  lcd.setCursor(11, 0);
  lcd.print(md);
  lcd.setCursor(0, 1);
  lcd.print("SV:");
  uint8_t u = ultrasonico();
  lcd.print(u);
  lcd.print(" ");
  lcd.setCursor(6, 1);
  lcd.print(st);
  mostrar_nextion(u);
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
    return 0;
  }
}
//función de PID
void pid() {

  currentTime = millis();
  // Si el tiempo actual - el tiempo anterior es mayor o igual al establecido se ejecuta el bloque de instrucciones dentro del if.
  //el principio de delay sin detener el programa ejecuta las instrucciones cada tiempo que está establecido por la variable timeInterval dada en milisegundos.
  if (currentTime - prevTime >= timeInterval) {
    Input = ultrasonico();  // Leer el valor del sensor
    myPID.Compute();        // Calcular la salida del controlador PID
    // la entrada es menor al setpoint entonces el motor funcionara por pwm si es superior se apaga, debido a que la señal entre 0 y 60 no tiene la suficiente corriente para que el motor gire.
    // y como modo de seguridad.
    if (Input < Setpoint) {
      analogWrite(motor, Output);
    } else {
      digitalWrite(motor, LOW);
      Output = 60;
    }
    updateWaveform(0, Setpoint);
    updateWaveform(1, Input);

    prevTime = currentTime;  // el tiempo actual almacenado en la variable pasa a ser un tiempo pasado o previo o anterior.
  }
}
/*
para enviar datos se usa el Serial.print dentro la variable con la etiqueta (val) y =, como se muestra a continuacion:
Serial.print("setpoint_n.val=")
se procede a imprimir el valor de la variable
Serial.print(Setpoint)
y finalmente se mandan 3 byte vacios, para que la nextion interprete que le has enviado un comando.
Serial.write(0xff);
Serial.write(0xff);
Serial.write(0xff);

A continuación una lista de las variables con las que puedes usar en la nextion:
setpoint_n  = Set point de Nivel
setpoint_t = Set point de Temperatura
sv_n = Valor del sensor de Nivel
sv_t = Valor del sensor de Temperatura

Variables que puedes leer:

h0 = Slider para ajustar el setpoint de nivel
h1 = Slider para ajustar el setpoint de temperatura

para leer las variables tienes que entender la estructura que Nextion envia a traves del puerto serial donde en primera instancia envia un caracter 'b' 
indicando que ahi comienza la cadena de datos, el siguiente valor es la variable a la que hace referencia el resto de la informacion donde:
'n' es Nivel
't' es Temperatura

El Siguiente caracter se refiere al modo en que se encuentra el sistema desde la HMI donde 
'a' es Automatico
'm' es Manual

Finalmente envia el valor del Setpoint establecido solo cuando se pasa de modo manual a modo automatico en la HMI seguido de 3 bytes vacios

Para graficar se envia la siguiente estructura:

Serial.print("add ID de la Grafica, el canal se puede usar 2 canales para gráficar al tiempo, el valor de la variable a gráficar);

el ID es 2, Asignado por la Nextion en su diseño.
Canal 1 es 0
Canal 2 es 1
Valor tiene que ser un entero.