#include <LiquidCrystal_I2C.h>
#include <NewPing.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);  //Especificacion de la pantalla LCD
//Definicion de pines
#define pot A0         // Ajustar manualmente el Setpoint(Valor deseado)
#define led_green 3    // Pin del Led Verde (Indicador de que el sistema esta funcionando)
#define switch_mode 6  // Interruptor para Iniciar o Detener el sistema de control.
#define led_red 4      // Pin del Led Rojo (Indicador de que el sistema esta Desactivado)
#define motor 5        // Señal PWM para el motor
#define sensor_agua 2  // Boya encargada de detectar si hay la suficiente agua en el sistema para funcionar.
//Definicion del sensor ultrasonico
#define trigger 13 // Envia el pulso Ultrasonico
#define echo 12 // Receptor
//Lectura del ultrasonico
double ultrasonico() {
  digitalWrite(trigger, HIGH);  // Establecemos el valor del pin HIGH(5V)
  delayMicroseconds(1000);      // Espera un segundo
  digitalWrite(trigger, LOW);   // Valor del pin LOW(0V)

  double d = pulseIn(echo, HIGH);  //Para leer cuando Echo sea HIGH
  d = (d / 2) / 29.1;              //Ecuacion
  return map(d, 0, 23, 23, 0);     // invertimos los maximos y minimos con la funcion map ( Evitar problemas con el calculo PID)
}
//Definicion de PID
double Setpoint = 0;             // Valor deseado del proceso
double Input, Output;            // Entrada y Salida del Sistema
double Kp = 10, Ki = 2, Kd = 1;  // Constantes del controlador PID
double ITerm, lastInput, maxLimit = 255, minLimit = 0;

unsigned long currentTime, prevTime;  // Tiempo actual y Tiempo anterior
unsigned long timeInterval = 100;     // Intervalo de tiempo en milisegundos

void setup() {
  new_setpoint(&Setpoint);  // Inicializamos el Setpoint en el valor del potenciometro.
  lcd.init();               // Inicializamos la LCD
  lcd.backlight();          //Inicializamos la retroiluminación de la LCD
  Serial.begin(115200);     //Establecemos la comunicación Serial a 115200 baudios
  //Configuracion de Entradas y Salidas (I/O)
  pinMode(led_green, OUTPUT);
  pinMode(led_red, OUTPUT);
  pinMode(motor, OUTPUT);
  pinMode(trigger, OUTPUT);
  pinMode(echo, INPUT);
  pinMode(switch_mode, INPUT);
  pinMode(sensor_agua, INPUT);

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
  // Se revisa entre cada ejecución del programa Sí el sistema cuenta con agua
  //mientras exista la suficiente agua el sistema continua su ejecución.
  while (agua() == 1) {
    if (digitalRead(switch_mode) == HIGH) {  // Entramos al modo de ejecución donde se ejecuta toda la operacion del PID.
      lcd_display("AUTO", "RUNTIME   ");     // función encargada de mostrar en la LCD la mayor parte de la informacion.
      digitalWrite(led_green, HIGH);
      digitalWrite(led_red, LOW);
      pid();  //función donde se encuentra toda la operación del PID

    } else {
      new_setpoint(&Setpoint);  // función que establece un nuevo setpoint en el mismo espacio de memoria de la variable a través de un puntero.
      lcd_display("MAN ", "STANDBY   ");
      digitalWrite(led_green, LOW);
      digitalWrite(led_red, HIGH);
      digitalWrite(motor, LOW);
    }
  }
  delay(800);
}

void new_setpoint(double* sp) {
  *sp = map(analogRead(pot), 0, 1023, 0, 15);  // Rango de Setpoint de 0 a 15 cm Tamaño maximo 19 cm Superior a ese valor el ultrasonico es inestable.
}
void lcd_display(String md, String st) {
  lcd.setCursor(0, 0);
  lcd.print("Sp:");
  lcd.setCursor(4, 0);
  lcd.print((int)Setpoint);
  lcd.print(" ");
  lcd.setCursor(7, 0);
  lcd.print("MD:");
  lcd.setCursor(11, 0);
  lcd.print(md);
  lcd.setCursor(0, 1);
  lcd.print("SV:");
  int u = ultrasonico();
  lcd.print(u);
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
    return 0;
  }
}
//función de PID
void pid() {


  // Si el tiempo actual - el tiempo anterior es mayor o igual al establecido se ejecuta el bloque de instrucciones dentro del if.
  //el principio de delay sin detener el programa ejecuta las instrucciones cada tiempo que está establecido por la variable timeInterval dada en milisegundos.
  /*El PID basico es el siguiente:

   unsigned long now = millis();
   double timeChange = (double)(now - lastTime);

   double error = Setpoint - Input;
   errSum += (error * timeChange);
   double dErr = (error - lastInput) / timeChange;

   Output = kp * error + ki * errSum + kd * dErr;
 
   lastInput = error;
   lastTime = now;
   
   el PID anterior tiene muchos problemas como un mal manejo de la Integral y Derivada, donde en el caso de la Integral no hay una opcion que determine que cuando
   
   la salida es igual o mayor al setpoint la parte Integral deberia limitarse a 0 o en su defecto empezar a restar si deseas un control mas limitado, pero para 
   efectos practicos y rapido es mejor llevarlo a 0 para evitar que el sistema se desfase con respecto al setpoint estando con un error minimo o nulo
   
   La derivada  Con error=Setpoint-Input, cualquier cambio en Setpoint resultará en un transitorio de error. La derivada de este cambio es infinita 
   (en realidad, como dt no es 0, termina siendo un número muy grande).Este número se introduce en la ecuación pid, lo que provoca picos no deseados
    en la salida. para corregir Resulta que la derivada del error es igual a la derivada negativa de la entrada, EXCEPTO cuando el punto de referencia está cambiando.
    En lugar de sumar (Kd * derivada de Error), restamos (Kd * derivada de Entrada). Esto se conoce como el uso de "Derivada en la Medición"
    Las modificaciones aquí son bastante fáciles. Estamos reemplazando +dError con -dInput. En lugar de recordar el último error, ahora recordamos la última entrada*/
  currentTime = millis();
  double timeChange = (double)(currentTime - prevTime);
  if (timeChange >= timeInterval) {
    Input = ultrasonico();  // Leer el valor del sensor


    double error = Setpoint - Input;
    /*
    La PID Manual es una trampa que probablemente  más  para principiantes que cualquier otra. Ocurre cuando el PID cree que puede hacer algo que no puede. 
    Por ejemplo, la salida PWM de un Arduino acepta valores de 0 a 255. Por defecto, el PID no sabe esto. Si cree que 300-400-500 funcionará
    probará esos valores esperando obtener lo que necesita. Dado que en realidad el valor está fijado en 255, seguirá intentando números 
    cada vez más altos sin llegar a ninguna parte. Una vez que se alcanza cualquiera de los límites, el pid deja de sumar (integrar) y se lleva a 0.
    */
    ITerm += (Ki * error) * 0.1;

    double dInput = (Input - lastInput) / 0.1;

    /*Compute PID Output*/
    Output = Kp * error + Ki * errSum + Kd * dInput;
    if (Output > maxLimit)
      ITerm -= Output – maxLimit;
      Output = maxLimit;
    else if (Output < minLimit)
      ITerm += minLimit – Output;
      Output = minLimit;
    if (ITerm > maxLimit) ITerm = outMax;
    else if (ITerm < minLimit) ITerm = outMin;
    // la entrada es menor al setpoint entonces el motor funcionara por pwm si es superior se apaga.
    //debido a que la señal entre 0 y 60 no tiene la suficiente corriente para que el motor gire.
    if (Input < Setpoint) {
      analogWrite(motor, Output);
    } else {
      digitalWrite(motor, LOW);
      Output = 60;
    }


    Serial.print(Input);
    Serial.print(",");
    Serial.println(Output);
    lastInput = Input;
    prevTime = currentTime;  // el tiempo actual almacenado en la variable pasa a ser un tiempo pasado o previo o anterior.
  }
}
