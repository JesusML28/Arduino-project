#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v2.h>
LiquidCrystal_I2C lcd(0x27,16,2);//Especificacion de la pantalla LCD
//Definicion de PID
double Setpoint = 35;  // Valor deseado del proceso
double Input, Output;// Entrada y Salida del Sistema
double Kp = 2, Ki = 5, Kd = 1;  // Constantes del controlador PID

unsigned long currentTime, prevTime;// Tiempo actual y Tiempo anterior
unsigned long timeInterval = 1000;  // Intervalo de tiempo en milisegundos
// Inicializamos el PID, con la siguiente parametros (Entrada, Salida, Valor Deseado, Constante Proporcional, Integral, Derivativa, MODO);
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
//Definicion de pines
#define sensor_temp 2 // Pin del sensor DS18B20
#define led_green 3 // Pin del Led Verde (Indicador de que el sistema esta funcionando)
#define switch_mode 4 // Interruptor para Iniciar o Detener el sistema de control.
#define led_red 5 // Pin del Led Rojo (Indicador de que el sistema esta Desactivado)
#define SSR 6 // relé de estado solido ON/OFF conectado a la Resistencia.
#define sensor_agua 7 // Boya encargada de detectar si hay la suficiente agua en el sistema para funcionar.
#define pot A0 // Ajustar manualmente el Setpoint(Valor deseado)

// Instancia a las clases OneWire y DallasTemperature ¡Necesarias para el sensor de temperatura!
OneWire oneWireObjeto(sensor_temp);
DallasTemperature sensorDS18B20(&oneWireObjeto);
void setup()
{  
    sensorDS18B20.begin(); // Inicializamos el sensor
    lcd.init(); // Inicializamos la LCD
    lcd.backlight(); //Inicializamos la retroiluminación de la LCD
    Serial.begin(115200); //Establecemos la comunicación Serial a 115200 baudios
    //Configuracion de Entradas y Salidas (I/O)
    pinMode(led_green,OUTPUT);
    pinMode(led_red,OUTPUT);
    pinMode(SSR,OUTPUT);
    digitalWrite(SSR,LOW);
    pinMode(switch_mode,INPUT);
    pinMode(sensor_agua,INPUT);
    pinMode(sensor_temp,INPUT);
    //Parametrizamos el PID
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(0, timeInterval); // Establecemos los límites de la salida del PID                      
    myPID.SetSampleTime(timeInterval);   // Tiempo de lectura entre calculo algorítmico del sistema
    prevTime = millis(); // Inicializamos por primera vez la variable con la función millis() encargada de dar el tiempo actual de ejecución.
    // Una pequeña animación de Inicialización del sistema.
    lcd.setCursor(3,0);
    lcd.print("CARGANDO");
    lcd.setCursor(3,1);
    for(int i = 0;i<8;i++){
      lcd.print("*");
      delay(1000);  
 }
    lcd.clear();
    // Antes de entrar al programa de ejecución se revisa por primera vez el estado de la estación, verificando sí tiene agua.
    while(agua()==0){delay(800);}// Si no tiene agua se queda en un bucle hasta que llenen el tanque principal.
    
}

void loop()
{
  lcd.clear();
  while(agua()==1){// Se revisa entre cada ejecución del programa Sí el sistema cuenta con agua, mientras exista la suficiente agua el sistema continua su ejecución.
   if(digitalRead(switch_mode) == HIGH){// Entramos al modo de ejecución donde se ejecuta toda la operacion del PID.
    lcd_display("AUTO", "RUNTIME   "); // función encargada de mostrar en la LCD la mayor parte de la informacion.
    digitalWrite(led_green, HIGH);
    digitalWrite(led_red, LOW);
    pid(); //función donde se encuentra toda la operación del PID
    control();// función encargada de Encender o apagar la resistencia en función de la salida del PID
  }else{
    new_setpoint(&Setpoint);// función que establece un nuevo setpoint en el mismo espacio de memoria de la variable a través de un puntero.
    lcd_display("MAN ","STANDBY   ");
    digitalWrite(led_green, LOW);
    digitalWrite(led_red,HIGH);
    digitalWrite(SSR, LOW);
  } 
  }
  
  delay(800);
}
void new_setpoint(double* sp){
  *sp = map(analogRead(pot),0,1023,30,40);// Rango de Setpoint 30 y 40 °C totalmente ajustable donde 30°C es la temperatura ambiente del agua en Barranquilla
}

void lcd_display(String md, String st){
  lcd.setCursor(0,0);
  lcd.print("Sp:");
  lcd.setCursor(4,0);
  lcd.print((int)Setpoint);
  lcd.setCursor(7,0);
  lcd.print("MD:");
  lcd.setCursor(11,0);
  lcd.print(md);
  lcd.setCursor(0,1);
  lcd.print("SV:");
  sensorDS18B20.requestTemperatures();
  int t = sensorDS18B20.getTempCByIndex(0);
  lcd.print(t);
  lcd.print(" ");
  lcd.setCursor(6,1);
  lcd.print(st);

}
// función para verificar el agua en el sistema.
bool agua(){
  if (digitalRead(sensor_agua) == HIGH){
    return 1;
  }else{
    lcd.setCursor(3,0);
    lcd.print("NECESITA");
    lcd.setCursor(5,1);
    lcd.print("AGUA");
    digitalWrite(SSR, LOW);
    return 0;
  }
    }
//función de PID
void pid(){
  currentTime = millis();
  // Si el tiempo actual - el tiempo anterior es mayor o igual al establecido se ejecuta el bloque de instrucciones dentro del if.
  //el principio de delay sin detener el programa ejecuta las instrucciones cada tiempo que está establecido por la variable timeInterval dada en milisegundos.
  if (currentTime - prevTime >= timeInterval) {
    //Se envía una petición al sensor para que calcule la temperatura que está midiendo.
    sensorDS18B20.requestTemperatures();
    Input = sensorDS18B20.getTempCByIndex(0); // Obtenemos el valor del sensor
    myPID.Compute();  // La función compute de la librería PID lee la variable Input realiza el cálculo y como resultado modifica el valor de la variable de salida
   //Output usando Punteros por lo que no requiere parámetros en la función, debido a que modifica el valor de la variable en el espacio de memoria asignado.
    Serial.print(Input);
    Serial.print(",");
    Serial.println(Output);
    //Se envía una petición para la siguiente medición.
    sensorDS18B20.requestTemperatures();
    prevTime = currentTime;// el tiempo actual almacenado en la variable pasa a ser un tiempo pasado o previo o anterior.
  }
}
void control() {
    /* Para controlar un sistema ON/OFF con PID el creador de la librería recomienda implementar una lógica diferente donde la salida del PID no es
       más que el tiempo en milisegundos en el sistema que va a estar encendido o apagado.
       Sí el tiempo actual es menor a la suma del tiempo anterior más la salida del PID o la salida es igual al tiempo de Intervalo y la entrada es menor al setpoint entonces la resistencia estará encendida 
       */

    if (((millis() <= (prevTime + Output)) || (Output == timeInterval)) && Input<Setpoint) {
    // Power on:
    digitalWrite(SSR, HIGH);

  } else {
    // Power off:
    digitalWrite(SSR, LOW);
  }
}

