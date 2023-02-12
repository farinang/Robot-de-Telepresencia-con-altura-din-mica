#include <AccelStepper.h>
#include <Encoder.h>
#include <ezButton.h>

#define pul_pin 7 //Pin PUL- del Driver
#define dir_pin 6 //Pin DIR- del Driver

#define microstep 2 //Factor de pasos del motor en el Driver DM542 (400)
#define pasos_motor 200 //Pasos del motor 360°/1.8°
#define avance_tornillo 2 //[mm] L=n*p; n=1 (Hilos) y p=2mm (pasos)

#define pulsador1_pin A1 //Pin del final de carrera superior
#define pulsador2_pin A0 //Pin final de carrera inferior

#define pin_A_encoder 4
#define pin_B_encoder 3

const uint8_t cantidad_ejes = 3; //Cantidad de ejes x, y, z
char nombre_ejes[cantidad_ejes]={'x','y','z'};
char delimitador1 = ',';
char delimitador2 = ':';
String string_eje[cantidad_ejes];
int posicion_delimitadores[cantidad_ejes-1];
float coordenadas_xyz[cantidad_ejes]; //[mm] distancia del usuario en x, y, z.
int aceleracion = 12000; //[Pasos/segundos^2] Aceleración del motor
int velocidad_maxima =2500;//[Pasos/segundos] Velocidad del motor
//int pasos_mm; //[pasos/mm] Factor de conversión de mm a pasos para saber cuánto se debe mover el motor 
long pasos_mm; //[pasos/mm] Factor de conversión de mm a pasos para saber cuánto se debe mover el motor
String serial; //Comando recibido por el serial
//int recorrido_maximo= 370; //mm
long recorrido_maximo= 370; //mm
long posicion_home = 200;
long posicion_objetivo;
long posicion_encoder;
int sentido_giro;
long distancia_segura_inferior = 10; //[mm]
long distancia_segura_superior = 360; //[mm]
bool estado_pulsador_inferior;
bool estado_pulsador_superior;

Encoder encoder(pin_B_encoder,pin_A_encoder);
AccelStepper motor(AccelStepper::DRIVER, pul_pin, dir_pin); //Objeto de la librería AccelStepper llamado "motor"
ezButton pulsador_inferior(pulsador2_pin);
ezButton pulsador_superior(pulsador1_pin);

void establecer_sentido_giro(int distancia_mm){
  posicion_objetivo = abs(distancia_mm*pasos_mm);
  posicion_encoder = (abs(encoder.read()*400))/1600;
  if((posicion_objetivo-posicion_encoder)<0){
    sentido_giro = 200;
  }else{
    sentido_giro = -200;
  }
}

void mover_home(int distancia_mm){
  establecer_sentido_giro(distancia_mm);
  while((posicion_objetivo-posicion_encoder)>0){
    motor.move(sentido_giro);
    motor.run();
    posicion_encoder = (abs(encoder.read()*400))/1600;
  }
}

void limite_superior_inferior(){
  if(estado_pulsador_inferior || estado_pulsador_superior){
    int distancia_segura = 0;
    if(estado_pulsador_inferior){
      distancia_segura = distancia_segura_inferior;
    }else if(estado_pulsador_superior){
      distancia_segura = distancia_segura_superior;
    }     
    establecer_sentido_giro(distancia_segura);
    Serial.println(posicion_objetivo-posicion_encoder);
    while((posicion_objetivo-posicion_encoder)!=0){
      motor.move(sentido_giro);
      motor.run();
      posicion_encoder = (abs(encoder.read()*400))/1600;
    }
  }
}

void mover_motor(int distancia_mm){
  establecer_sentido_giro(distancia_mm);
  estado_pulsador_inferior = pulsador_inferior.isPressed();
  estado_pulsador_superior = pulsador_superior.isPressed(); 
  while((posicion_objetivo-posicion_encoder)!=0 && !estado_pulsador_inferior && !estado_pulsador_superior){
    /*Serial.print("Posicion objetivo");
    Serial.println(posicion_objetivo);
    Serial.print("Posicion encoder");
    Serial.println(posicion_encoder);
    Serial.print("resta");
    Serial.println(posicion_objetivo-posicion_encoder);*/
    pulsador_inferior.loop();
    pulsador_superior.loop();
    motor.move(sentido_giro);
    motor.run();
    posicion_encoder = (abs(encoder.read()*400))/1600;
    estado_pulsador_inferior = pulsador_inferior.isPressed();
    estado_pulsador_superior = pulsador_superior.isPressed(); 
  }
  delay(500);
}

void desplazar_home(){
  while(!pulsador_inferior.isPressed()){
    pulsador_inferior.loop();
    motor.move(posicion_home);
    motor.run();
  }
  delay(500);
  motor.setCurrentPosition(0);
  encoder.write(0);
  motor.setMaxSpeed(velocidad_maxima); //Se establece la velocidad máxima del motor
  motor.setAcceleration(aceleracion); //Se establece la aceleración del motor
  mover_home(recorrido_maximo/2);
  motor.disableOutputs();
  delay(50);
}

void desplazar_objetivo_z(){
  for (uint8_t i=0; i<cantidad_ejes; i++){
    if(nombre_ejes[i]=='z'){
      mover_motor(coordenadas_xyz[i]);
      limite_superior_inferior();
      motor.disableOutputs();
    }
  }
}

void actualizar_coordenadas_xyz(){
  if(Serial.available() > 0){
    serial = Serial.readString();
    for (uint8_t i=0; i<cantidad_ejes; i++){
      if(i==0){
        posicion_delimitadores[i] = serial.indexOf(delimitador1);
        string_eje[i] = serial.substring(0,posicion_delimitadores[i]);
      }else if(i==cantidad_ejes){
        string_eje[i] = serial.substring(posicion_delimitadores[i-1]+1);
      }else{
        posicion_delimitadores[i] = serial.indexOf(delimitador1, posicion_delimitadores[i-1]+1);
        string_eje[i] = serial.substring(posicion_delimitadores[i-1]+1, posicion_delimitadores[i]);
      }
      coordenadas_xyz[i] = string_eje[i].substring(string_eje[i].indexOf(delimitador2)+1).toFloat();
    }
    delay(50);
    motor.enableOutputs();
    desplazar_objetivo_z();
  }
}

void setup() {
  Serial.begin(115200);
  pulsador_inferior.setDebounceTime(50);
  pulsador_superior.setDebounceTime(50);
  pasos_mm = ((pasos_motor * microstep)/avance_tornillo);
  motor.setMaxSpeed(velocidad_maxima); //Se establece la velocidad máxima del motor
  motor.setAcceleration(aceleracion); //Se establece la aceleración del motor
  desplazar_home();
}

void loop() {
  pulsador_inferior.loop();
  pulsador_superior.loop();
  actualizar_coordenadas_xyz();  
}
