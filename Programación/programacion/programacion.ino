
/*==================== Librerias ====================*/
#include <AccelStepper.h> //Librería para el motor PaP
#include <Encoder.h> //Librería para el encoder
#include <ezButton.h> //Librería para los finales de carrera

/*==================== Pines y variables globales ====================*/
#define pul_pin 7 //Pin PUL- del Driver
#define dir_pin 6 //Pin DIR- del Driver

#define microstep 2 //Factor de pasos del motor en el Driver DM542 (400 pasos)
#define pasos_motor 200 //Pasos del motor 360°/1.8°
#define avance_tornillo 2 //[mm] L=n*p; n=1 (Hilos) y p=2mm (pasos)

#define pulsador2_pin A0 //Pin del final de carrera inferior en el eje z
#define pulsador1_pin A1 //Pin del final de carrera superior en el eje z

#define pin_A_encoder 2 //Fase A (cable blanco del encoder)
#define pin_B_encoder 3 //Fase B (cable verde del encoder)

//Formato de la trama -> x:0,y:0,z:1640,R:0,P:0,Y:0
const uint8_t cantidad_grados_libertad = 6; //Cantidad de ejes x, y, z; cantidad de angulos Roll, Pitch, Yaw
char nombre_grados_libertad[cantidad_grados_libertad]={'x','y','z','R','P','Y'}; //Arreglo de los nombres de los grados de libertad
char delimitador1 = ','; //Delimitador de los grados de libertad
char delimitador2 = ':'; //Delimitador de los valores de los grados de libertad
String string_eje[cantidad_grados_libertad]; //Arreglo para almacenar los nombres y valores de los grados de libertad (Ejemplo: [x:0,y:0,z:1640....])
int posicion_delimitadores[cantidad_grados_libertad-1]; //Arreglo de los índices del delimitador 1 (,)
float coordenadas_xyz_RPY[cantidad_grados_libertad]; //[mm] Arreglo de valores de los grados de libertad (Ejemplo: [0,0,1640....])

String serial; //Comando recibido por el serial

int aceleracion = 12000; //[Pasos/segundos^2] Aceleración del motor
int velocidad_maxima = 2500;//[Pasos/segundos] Velocidad del motor
long pasos_mm; //[pasos/mm] Factor de conversión de mm a pasos para saber cuánto se debe mover el motor

const int altura_pantalla_referencia = 1475; //[mm] Referencia de altura mínima con respecto al suelo hasta el 25% de la distancia entre la mitad de la pantalla y la cámara
int altura_pantalla_actual = 0; //[mm] Posición donde se encuentra la pantalla actualmente
const int limite_inferior_z = 0; //[mm] Posición mínima de desplazamiento en el eje z
const int limite_superior_z= 290; //[mm] Posición máxima de desplazamiento en el eje z
const int distancia_segura_inferior_z = 9; //[mm] 
const int distancia_segura_superior_z = 286; //[mm]

long posicion_objetivo; //[pasos]
long posicion_encoder;  //[pasos]

//En caso de perdida de pasos del encoder bajar a 100
const int pasos = 200; //Pasos para el movimiento del motor PaP durante su funcionamiento (Esta variable tiene como Valormax recomendado = 200)

int sentido_giro; //Valores: pasos, -pasos

bool estado_pulsador_inferior; 
bool estado_pulsador_superior;

/*==================== Objetos ====================*/
Encoder encoder(pin_B_encoder,pin_A_encoder); 
AccelStepper motor(AccelStepper::DRIVER, pul_pin, dir_pin); 
ezButton pulsador_inferior(pulsador2_pin);
ezButton pulsador_superior(pulsador1_pin);

//Esta funcion baja en el z al monitor hasta encontrar el final de carrera, luego la desplaza a la mitad de su recorrido total
void desplazar_home(int valor_maximo){
  pulsador_inferior.loop();
  estado_pulsador_inferior = pulsador_inferior.getState(); 
  while(estado_pulsador_inferior){
    motor.move(pasos);
    motor.run();
    pulsador_inferior.loop();
    estado_pulsador_inferior = pulsador_inferior.getState(); 
  }
  delay(500);
  motor.setCurrentPosition(0);
  encoder.write(0);
  motor.setMaxSpeed(velocidad_maxima); 
  motor.setAcceleration(aceleracion); 
  desplazar_mitad_recorrido(valor_maximo/2);
  altura_pantalla_actual = altura_pantalla_referencia + posicion_encoder/pasos_mm;
  enviar_altura();
  motor.disableOutputs();
  delay(50);
}

//Esta funcion desplaza al monitor hasta la mitad de su recorrido total y se tiene precaucion con el final de carera ubicado en la parte superior
void desplazar_mitad_recorrido(int distancia_mm){
  establecer_sentido_giro(distancia_mm);
  pulsador_superior.loop();
  estado_pulsador_superior = pulsador_superior.getState(); 
  while((posicion_objetivo-posicion_encoder)>0 && estado_pulsador_superior){
    motor.move(sentido_giro);
    motor.run();
    posicion_encoder = (abs(encoder.read()*400))/1600;
    pulsador_superior.loop();
    estado_pulsador_superior = pulsador_superior.getState(); 
  }
}

//Esta funcion permite saber si se debe hacer un giro horario o antihorario
//distancia_mm:distancia relativa a la que se debe dirigir el monitor
void establecer_sentido_giro(int distancia_mm){
  posicion_objetivo = abs(distancia_mm*pasos_mm);
  posicion_encoder = (abs(encoder.read()*400))/1600;
  if((posicion_objetivo-posicion_encoder)<0){
    sentido_giro = pasos;
  }else{
    sentido_giro = (-1*pasos);
  }
}

//Esta funcion lee la trama enviada por el algoritmo de vision por computadora y actualiza todas las coordenadas
void actualizar_coordenadas_xyz_RPY(){
  if(Serial.available() > 0){
    serial = Serial.readString();
    for (uint8_t i=0; i<cantidad_grados_libertad; i++){
      if(i==0){
        posicion_delimitadores[i] = serial.indexOf(delimitador1);
        string_eje[i] = serial.substring(0,posicion_delimitadores[i]);
      }else if(i==cantidad_grados_libertad){
        string_eje[i] = serial.substring(posicion_delimitadores[i-1]+1);
      }else{
        posicion_delimitadores[i] = serial.indexOf(delimitador1, posicion_delimitadores[i-1]+1);
        string_eje[i] = serial.substring(posicion_delimitadores[i-1]+1, posicion_delimitadores[i]);
      }
      coordenadas_xyz_RPY[i] = string_eje[i].substring(string_eje[i].indexOf(delimitador2)+1).toFloat();
    }
    delay(50);
    motor.enableOutputs();
    mover_objetivo('z', altura_pantalla_referencia, limite_inferior_z, limite_superior_z, distancia_segura_inferior_z, distancia_segura_superior_z);
    enviar_altura();
    motor.disableOutputs();
    
  }
}

//Esta funcion permite que los valores ingresados de movimiento esten dentro del rango permitido
//valor_coordenada:Valor escogido del algoritmo de vision por computadora que correspone a una distancia absoluta
//referencia:medida de longitud desde el piso hasta la posicion de interes
//valor_minimo:posicion relativa minima
//valor_maxima:posicion relativa maxima
//valor_minimo_seguro:posicion relativa minima segura
//valor_maximo_seguro:posicion relativa maxima segura 
int validar_rango(float valor_coordenada, int referencia, int valor_minimo, int valor_maximo, int valor_minimo_seguro, int valor_maximo_seguro){
  valor_coordenada = valor_coordenada - referencia;
  float valor_coordenada_final= 0;
  if (valor_coordenada<=valor_minimo){
    valor_coordenada_final = valor_minimo_seguro;
  }else if(valor_coordenada >= valor_maximo){
    valor_coordenada_final = valor_maximo_seguro;
  }else{
    valor_coordenada_final = valor_coordenada;
  }
  return valor_coordenada_final;
}

//Funcion que mueve el motor hasta la distancia requerida, donde tiene en cuenta los finales de carrera en caso de que el encoder pierda sus pasos
//distancia_mm:distancia relativa a la que se debe dirigir el monitor
//referencia:medida de longitud desde el piso hasta la posicion de interes
//valor_minimo_seguro:posicion relativa minima segura
//valor_maximo_seguro:posicion relativa maxima segura 
void mover_motor(int distancia_mm, int referencia, int valor_minimo_seguro, int valor_maximo_seguro){
  if(distancia_mm != altura_pantalla_actual- referencia){
    establecer_sentido_giro(distancia_mm);
    pulsador_inferior.loop();
    pulsador_superior.loop();
    estado_pulsador_inferior = pulsador_inferior.getState(); 
    estado_pulsador_superior = pulsador_superior.getState(); 
    bool condicion = true;
    if(sentido_giro < 0){
       condicion = (posicion_objetivo-posicion_encoder)>0;
    }else{
      condicion = (posicion_objetivo-posicion_encoder)<0;
    }
    while(condicion && estado_pulsador_inferior && estado_pulsador_superior && valor_minimo_seguro <= distancia_mm && valor_maximo_seguro >= distancia_mm ){
      motor.move(sentido_giro);
      motor.run();
      posicion_encoder = (abs(encoder.read()*400))/1600;
      pulsador_inferior.loop();
      pulsador_superior.loop();
      if(sentido_giro < 0){
        condicion = (posicion_objetivo-posicion_encoder)>0;
      }else{
        condicion = (posicion_objetivo-posicion_encoder)<0;
      }
      estado_pulsador_inferior = pulsador_inferior.getState(); 
      estado_pulsador_superior = pulsador_superior.getState();  
    }
  }
}

//Esta funcion mueve el sistema en la coordenada que se pase como parametro (mejorar)
//gdl:grado de libertad a mover
//referencia:medida de longitud desde el piso hasta la posicion de interes
//valor_minimo:posicion relativa minima
//valor_maxima:posicion relativa maxima
//valor_minimo_seguro:posicion relativa minima segura
//valor_maximo_seguro:posicion relativa maxima segura 
void mover_objetivo(char gdl, int referencia, int valor_minimo, int valor_maximo, int valor_minimo_seguro, int valor_maximo_seguro){
  for (uint8_t i=0; i<cantidad_grados_libertad; i++){
    if(nombre_grados_libertad[i] == gdl){
      int posicion_relativa_final = validar_rango(coordenadas_xyz_RPY[i], referencia, valor_minimo, valor_maximo, valor_minimo_seguro, valor_maximo_seguro);
      mover_motor(posicion_relativa_final, referencia, valor_minimo_seguro, valor_maximo_seguro);
      if (gdl == 'z'){
        altura_pantalla_actual = posicion_encoder/pasos_mm + referencia;
      }
    }
  }
}

void enviar_altura(){
  Serial.println(altura_pantalla_actual);
}

void setup(){
  Serial.begin(115200);
  pulsador_inferior.setDebounceTime(50);
  pulsador_superior.setDebounceTime(50);
  pasos_mm = ((pasos_motor * microstep)/avance_tornillo);
  motor.setMaxSpeed(velocidad_maxima);
  motor.setAcceleration(aceleracion);
  desplazar_home(limite_superior_z);
}

void loop(){
  pulsador_inferior.loop();
  pulsador_superior.loop();
  actualizar_coordenadas_xyz_RPY();  
}
