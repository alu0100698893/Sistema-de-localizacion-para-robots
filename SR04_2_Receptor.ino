/*RECEPTOR
 *  Codigo que se encargara de  recibir el mensaje del emisor, identificarlo y realizar
 *  los calculos del tiempo (Con el timer) y la distancia (nueva formula distinta a la anterior)
 *  
 *  Ademas realizaremos mediante el sistema de trilateración y sus fórmulas una localizacion para
 *  nuestro robot
 *  
 *  Vamos a desarrollar  un código que nos permita realizar una comunicación entre nuestro Arduino y ROS
 *  Además este tendrá que escenificar y obtener como mensaje la información de la posición que estamos obteniendo
 *  mediante el sistema de trilateración.
 *  
 *  Autor: Alberto Martínez Chincho
 *  Universidad de La Laguna
 */
#include <VirtualWire.h> //libreria para los modulos RF
#include <TimerThree.h> //libreria para el timer
/*#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
*/
const int pin_RF_Receptor = 9; //pin del receptor RF
const int pin_SR04_Echo_1 = 19; //pin del ultrasonido Echo
const int pin_SR04_Echo_2 = 21; //pin del ultrasonido Echo
const int pin_SR04_Echo_3 = 18; //pin del ultrasonido Echo
const int pin_SR04_Echo_4 = 2; //pin del ultrasonido Echo
const int pin_SR04_Echo_5 = 3; //pin del ultrasonido Echo
const int pin_SR04_Echo_6 = 20; //pin del ultrasonido Echo

/*ros::NodeHandle nh; //nodo objeto para ROS
geometry_msgs::TransformStamped t; //Instancia de un mensaje para la comunicacion
tf::TransformBroadcaster broadcaster; //radiodifusor para la comunicacion
*/
float distancia_A; //variable para almacenar la distancia al emisor A
float distancia_B; //variable para almacenar la distancia al emisor B
float distancia_C; //variable para almacenar la distancia al emisor C

//Marcos sobre los que se va a realizar la tranformacion
/*char base_link[] = "/base_link";
char odom[] = "/odom";
*/

//Variables para obtener los tiempos del timer
volatile uint32_t tiempo_1 = 0; 
volatile uint32_t tiempo_2 = 0;
volatile uint32_t tiempo_3 = 0;
volatile uint32_t tiempo_4 = 0;
volatile uint32_t tiempo_0 = 0;
volatile int num_int = 0; //contador para ver el numero de interrupciones

/*Variables que se usaran para la trilateración*/
int a;
int b;
int c;
int d;
int e;
int f;
int x1;
int y1;
int x2;
int y2;
int x3;
int y3;
int x_total;
int y_total;
int pos_1;
int pos_2;

//Creamos un mensaje
//La constante VW_MAX_MESSAGE_LEN viene definida en la libreria VirtualWire
uint8_t sms[VW_MAX_MESSAGE_LEN];
uint8_t sms_len = VW_MAX_MESSAGE_LEN;

void setup() {
  
  Serial.begin(9600);
  pinMode(pin_SR04_Echo_1, INPUT); //activamos el pin del ultrasonido como entrada de la señal
  pinMode(pin_SR04_Echo_2, INPUT); //activamos el pin del ultrasonido como entrada de la señal
  pinMode(pin_SR04_Echo_4, INPUT); //activamos el pin del ultrasonido como entrada de la señal
 /* pinMode(pin_SR04_Echo_4, INPUT); //activamos el pin del ultrasonido como entrada de la señal
  pinMode(pin_SR04_Echo_5, INPUT); //activamos el pin del ultrasonido como entrada de la señal
  pinMode(pin_SR04_Echo_6, INPUT); //activamos el pin del ultrasonido como entrada de la señal*/
  
  vw_setup(2000); //inicializamos la libreria
  vw_set_rx_pin(pin_RF_Receptor); //Configuramos el pin de recepcion
  vw_rx_start(); //Activamos el proceso de escucha(recepcion)

 /* nh.initNode();
  broadcaster.init(nh);
 */ 
  Timer3.initialize(50000); //Preescalado para el timer
  attachInterrupt(digitalPinToInterrupt(pin_SR04_Echo_1), calc_time_distancia_1, RISING); //interrupción asignada a nuestro pin arduino
  attachInterrupt(digitalPinToInterrupt(pin_SR04_Echo_2), calc_time_distancia_2, RISING); //interrupción asignada a nuestro pin arduino
  attachInterrupt(digitalPinToInterrupt(pin_SR04_Echo_4), calc_time_distancia_3, RISING); //interrupción asignada a nuestro pin arduino
  /*attachInterrupt(digitalPinToInterrupt(pin_SR04_Echo_4), calc_time_distancia_4, RISING); //interrupción asignada a nuestro pin arduino
  attachInterrupt(digitalPinToInterrupt(pin_SR04_Echo_5), calc_time_distancia_5, RISING); //interrupción asignada a nuestro pin arduino
  attachInterrupt(digitalPinToInterrupt(pin_SR04_Echo_6), calc_time_distancia_6, RISING); //interrupción asignada a nuestro pin arduino*/
}

//Inicio de las ISR para coger el tiempo de nuestro Timer
void calc_time_distancia_1(void){
  if(num_int == 0){
    tiempo_1 = TCNT3;
    num_int++;
  }
  else if(num_int == 1){
    tiempo_2 = TCNT3;
    num_int++;
  }
  else if(num_int == 2){
    tiempo_3 = TCNT3;
    num_int++;
  }
  else if(num_int == 3){
    tiempo_4 = TCNT3;
    num_int++;
  }
}

void calc_time_distancia_2(void){
  if(num_int == 0){
    tiempo_1 = TCNT3;
    num_int++;
  }
  else if(num_int == 1){
    tiempo_2 = TCNT3;
    num_int++;
  }
  else if(num_int == 2){
    tiempo_3 = TCNT3;
    num_int++;
  }
  else if(num_int == 3){
    tiempo_4 = TCNT3;
    num_int++;
  }
}

void calc_time_distancia_3(void){
  if(num_int == 0){
    tiempo_1 = TCNT3;
    num_int++;
  }
  else if(num_int == 1){
    tiempo_2 = TCNT3;
    num_int++;
  }
  else if(num_int == 2){
    tiempo_3 = TCNT3;
    num_int++;
  }
  else if(num_int == 3){
    tiempo_4 = TCNT3;
    num_int++;
  }
}

/*void calc_time_distancia_4(void){
  if(num_int == 0){
    tiempo_1 = TCNT3;
    num_int++;
  }
  else if(num_int == 1){
    tiempo_2 = TCNT3;
    num_int++;
  }
  else if(num_int == 2){
    tiempo_3 = TCNT3;
    num_int++;
  }
  else if(num_int == 3){
    tiempo_4 = TCNT3;
    num_int++;
  }
}

void calc_time_distancia_5(void){
  if(num_int == 0){
    tiempo_1 = TCNT3;
    num_int++;
  }
  else if(num_int == 1){
    tiempo_2 = TCNT3;
    num_int++;
  }
  else if(num_int == 2){
    tiempo_3 = TCNT3;
    num_int++;
  }
  else if(num_int == 3){
    tiempo_4 = TCNT3;
    num_int++;
  }
}

void calc_time_distancia_6(void){
  if(num_int == 0){
    tiempo_1 = TCNT3;
    num_int++;
  }
  else if(num_int == 1){
    tiempo_2 = TCNT3;
    num_int++;
  }
  else if(num_int == 2){
    tiempo_3 = TCNT3;
    num_int++;
  }
  else if(num_int == 3){
    tiempo_4 = TCNT3;
    num_int++;
  }
}*/
//Fin de las ISR para coger el tiempo de nuestro Timer

void loop() {
/* Con vw_get_message leemos el ultimo mensaje recibido 
 *  devolviendo true si existe un mensaje o false en caso
 *  de no haber recibido ninguno o de producirse algun fallo
 */
    if(vw_get_message(sms, &sms_len)){
      if(sms[0] == 'A'){ //Comprobamos que la señal provenga del primer sensor
        //Serial.println("Conectado a A");
        if(tiempo_1 > 0 || tiempo_2 > 0 || tiempo_3 > 0 || tiempo_4 > 0 || num_int == 4 ){
          tiempo_0=(tiempo_1 + tiempo_2 + tiempo_3 + tiempo_4)/4;
          num_int = 0; 
          Timer3.restart();
          //Serial.print("Tiempo A ");
          //Serial.println(tiempo_0);  
        } 
        distancia_A = ((0.0167 * tiempo_0) - 2.9587 );
        /*Serial.print("Distancia al emisor A: ");
        Serial.print(distancia_A);
        Serial.println(" cm");*/
      }
      if(sms[0] == 'B'){ //Comprobamos que la señal provenga del segundo sensor
        //Serial.println("Conectado a B");
        if(tiempo_1 > 0 || tiempo_2 > 0 || tiempo_3 > 0 || tiempo_4 > 0 || num_int == 4 ){
          tiempo_0=(tiempo_1 + tiempo_2 + tiempo_3 + tiempo_4)/4;
          num_int = 0; 
          Timer3.restart();
          //Serial.print("Tiempo B ");
         // Serial.println(tiempo_0);  
        } 
        distancia_B = ((0.0167 * tiempo_0) - 2.9587 );
        /*Serial.print("Distancia al emisor B: ");
        Serial.print(distancia_B);
        Serial.println(" cm");*/
      }
      if(sms[0] == 'C'){ //Comprobamos que la señal provenga del tercer sensor
        //Serial.println("Conectado a C");
        if(tiempo_1 > 0 || tiempo_2 > 0 || tiempo_3 > 0 || tiempo_4 > 0 || num_int == 4 ){
          tiempo_0=(tiempo_1 + tiempo_2 + tiempo_3 + tiempo_4)/4;
          num_int = 0; 
          Timer3.restart();
          //Serial.print("Tiempo C ");
          //Serial.println(tiempo_0);  
        } 
        distancia_C = ((0.0167 * tiempo_0) - 2.9587 );
       /* Serial.print("Distancia al emisor C: ");
        Serial.print(distancia_C);
        Serial.println(" cm");*/
      } 
  }
  delay(6000);
  
  /*Apartir de aqui calculamos la trilateración*/
  /*Damos valores fijos a las posiciones de los sensores que actuaran de emisor*/
  x1 = 37;
  y1 = 0;
  x2 = 5;
  y2 = 25;
  x3 = -34; 
  y3 = -2;

  /*Calculos de los sistemas de ecuaciones*/
  a = (-2*x1)+(2*x2);
  b = (-2*y1)+(2*y2);
  c = (diatancia_A*distancia_A)-(distancia_B*distancia_B)-(x1*x1)+(x2*x2)-(y1*y1)+(y2*y2);
  d = (-2*x2)+(2*x3);
  e = (-2*y2)+(2*y3);
  f = (distancia_B*distancia_B)-(distancia_C*distancia_C)-(x2*x2)+(x3*x3)-(y2*y2)+(y3*y3);
  
  x_total = ((c*e)-(f*b))/((e*a)-(b*d)); 
  y_total = ((c*d)-(a*f))/((b*d)-(a*e));

  /*Mostramos la posición actual del robot*/
  pos_1 = x_total;
  pos_2 = y_total;

  Serial.println("Posición aproximada");
  Serial.println(pos_1);
  Serial.println(pos_2);

  /*Apartir de aqui utilizaremos las funciones de ROS para poder establecer una comunicacion*/
  //Rellenamos los campos de nuestra transformación (tf odom->base_link)
 /*t.header.frame_id = odom;
  t.child_frame_id = base_link;
  t.transform.translation.x = x;
  t.transform.translation.y = y;
  t.header.stamp = nh.now();

  //Finalmente publicamos la transformacion y esperamos un poco antes de volver hacerlo
  broadcaster.sendTransform(t);
  nh.spinOnce();

 // delay(10);
 */
}
