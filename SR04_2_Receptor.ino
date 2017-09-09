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
#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

const int pin_RF_Receptor = 9; //pin del receptor RF
const int pin_SR04_Echo_1 = 19; //pin del ultrasonido Echo
const int pin_SR04_Echo_2 = 21; //pin del ultrasonido Echo
const int pin_SR04_Echo_3 = 18; //pin del ultrasonido Echo
const int pin_SR04_Echo_4 = 2; //pin del ultrasonido Echo
const int pin_SR04_Echo_5 = 3; //pin del ultrasonido Echo
const int pin_SR04_Echo_6 = 20; //pin del ultrasonido Echo

float distancia_A; //variable para almacenar la distancia al emisor A
float distancia_B; //variable para almacenar la distancia al emisor B
float distancia_C; //variable para almacenar la distancia al emisor C

//Variables para obtener los tiempos del timer
volatile uint32_t tiempo_1 = 0; 
uint8_t ultimo_mensaje_recibido; //variable que almacena el ultimo mensaje que recibimos a tra vés de RF
bool dist_calculada_A = 0;
bool dist_calculada_B = 0;
bool dist_calculada_C = 0;

//Creamos un mensaje radio-frecuencia
//La constante VW_MAX_MESSAGE_LEN viene definida en la libreria VirtualWire
uint8_t sms[VW_MAX_MESSAGE_LEN];
uint8_t sms_len = VW_MAX_MESSAGE_LEN;

/*Variables que se usaran para la trilateración*/
float a;
float b;
float c;
float d;
float e;
float f;
float x1;
float y1;
float x2;
float y2;
float x3;
float y3;
float x_total;
float y_total;
float pos_1;
float pos_2;

//ROS
ros::NodeHandle nh; //nodo objeto para ROS
geometry_msgs::TransformStamped t; //Instancia de un mensaje para la comunicacion
tf::TransformBroadcaster broadcaster; //radiodifusor para la comunicacion
nav_msgs::Odometry odom_msg;
ros::Publisher odomPub("/odom", &odom_msg);

//Marcos sobre los que se va a realizar la tranformacion
char base_link[] = "/base_link";
char odom[] = "/odom";

void setup() {
  
  Serial.begin(9600);
  pinMode(pin_SR04_Echo_1, INPUT); //activamos el pin del ultrasonido como entrada de la señal
  pinMode(pin_SR04_Echo_2, INPUT); //activamos el pin del ultrasonido como entrada de la señal
  pinMode(pin_SR04_Echo_3, INPUT); //activamos el pin del ultrasonido como entrada de la señal
  pinMode(pin_SR04_Echo_4, INPUT); //activamos el pin del ultrasonido como entrada de la señal
  pinMode(pin_SR04_Echo_5, INPUT); //activamos el pin del ultrasonido como entrada de la señal
  pinMode(pin_SR04_Echo_6, INPUT); //activamos el pin del ultrasonido como entrada de la señal
  
  vw_setup(2000); //inicializamos la libreria
  vw_set_rx_pin(pin_RF_Receptor); //Configuramos el pin de recepcion
  vw_rx_start(); //Activamos el proceso de escucha(recepcion)

  nh.initNode();
  //broadcaster.init(nh);
  
  Timer3.initialize(50000); //Preescalado para el timer
  attachInterrupt(digitalPinToInterrupt(pin_SR04_Echo_1), calc_time_distancia_1, RISING); //interrupción asignada a nuestro pin arduino
  attachInterrupt(digitalPinToInterrupt(pin_SR04_Echo_2), calc_time_distancia_2, RISING); //interrupción asignada a nuestro pin arduino
  attachInterrupt(digitalPinToInterrupt(pin_SR04_Echo_3), calc_time_distancia_3, RISING); //interrupción asignada a nuestro pin arduino
  attachInterrupt(digitalPinToInterrupt(pin_SR04_Echo_4), calc_time_distancia_4, RISING); //interrupción asignada a nuestro pin arduino
  attachInterrupt(digitalPinToInterrupt(pin_SR04_Echo_5), calc_time_distancia_5, RISING); //interrupción asignada a nuestro pin arduino
  attachInterrupt(digitalPinToInterrupt(pin_SR04_Echo_6), calc_time_distancia_6, RISING); //interrupción asignada a nuestro pin arduino
  Serial.println("INICIANDO... ");
}

//Inicio de las ISR para coger el tiempo de nuestro Timer
void calc_time_distancia_1(void){
  
    tiempo_1 = TCNT3;
}

void calc_time_distancia_2(void){
  
    tiempo_1 = TCNT3;
}

void calc_time_distancia_3(void){
  
    tiempo_1 = TCNT3;
}

void calc_time_distancia_4(void){

    tiempo_1 = TCNT3;
}

void calc_time_distancia_5(void){
  
    tiempo_1 = TCNT3;
}

void calc_time_distancia_6(void){
 
    tiempo_1 = TCNT3;
}
//Fin de las ISR para coger el tiempo de nuestro Timer

void loop() {
/* Con vw_get_message leemos el ultimo mensaje recibido 
 *  devolviendo true si existe un mensaje o false en caso
 *  de no haber recibido ninguno o de producirse algun fallo
 */
   if(vw_get_message(sms, &sms_len)){
      //Serial.print("sms: ");
      //Serial.println(sms[0]);
      if(sms[0] == 'A'){ //Comprobamos que la señal provenga del primer sensor
        Timer3.restart();
        tiempo_1 = 0;
        dist_calculada_A = 0;
        if(sms[0] != ultimo_mensaje_recibido){
          Serial.println(" recibido A");
        }
        ultimo_mensaje_recibido = sms[0];
      }
      if(sms[0] == 'B'){ //Comprobamos que la señal provenga del segundo sensor
        Timer3.restart();
        tiempo_1 = 0;
        dist_calculada_B = 0;
        if(sms[0] != ultimo_mensaje_recibido){
          Serial.println(" recibido B");
        }
        ultimo_mensaje_recibido = sms[0];
      }
      if(sms[0] == 'C'){ //Comprobamos que la señal provenga del tercer sensor
        Timer3.restart(); 
        tiempo_1 = 0;
        dist_calculada_C = 0;
        if(sms[0] != ultimo_mensaje_recibido){
          Serial.println(" recibido C");
        }
        ultimo_mensaje_recibido = sms[0];
      } 
  }
  //Serial.print("ultimo mensaje: ");
  //Serial.println(ultimo_mensaje_recibido);
  if(ultimo_mensaje_recibido == 'A'){
    if(tiempo_1 > 0 && dist_calculada_A == 0){
       dist_calculada_A = 1;
       distancia_A = ((0.0175 * tiempo_1) - 8.9222 );
       Serial.print("Distancia al emisor A: ");
       //Serial.println(tiempo_1);
       Serial.print(distancia_A);
       Serial.println(" cm");
    }
  }
  if(ultimo_mensaje_recibido == 'B'){
    if(tiempo_1 > 0 && dist_calculada_B == 0){
      dist_calculada_B = 1;   
      distancia_B = ((0.0175 * tiempo_1) - 8.9222 );
      Serial.print("Distancia al emisor B: ");
      //Serial.println(tiempo_1);
      Serial.print(distancia_B);
      Serial.println(" cm");
    }
  }
  if(ultimo_mensaje_recibido == 'C'){
    if(tiempo_1 > 0 && dist_calculada_C == 0){
      dist_calculada_C = 1;
      distancia_C = ((0.0175 * tiempo_1) - 8.9222 );
      Serial.print("Distancia al emisor C: ");
      //Serial.println(tiempo_1);
      Serial.print(distancia_C);
      Serial.println(" cm");  
    }
  }
  /*Apartir de aqui calculamos la trilateración*/
  /*Damos valores fijos a las posiciones de los sensores que actuaran de emisor*/
  if(dist_calculada_A && dist_calculada_B && dist_calculada_C){ //solo entra cuando todas sean true
    noInterrupts();//deshabilitamos las interrupciones para que en este proceso no salte ninguna 
    x1 = 59;
    y1 = 31;
    x2 = -67;
    y2 = 39;
    x3 = 0; 
    y3 = 0;
    dist_calculada_A = 0;
    dist_calculada_B = 0;
    dist_calculada_C = 0;

    /*Calculos de los sistemas de ecuaciones*/
    a = (-2*x1)+(2*x2);
    b = (-2*y1)+(2*y2);
    c = (distancia_A*distancia_A)-(distancia_B*distancia_B)-(x1*x1)+(x2*x2)-(y1*y1)+(y2*y2);
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
    
    interrupts();//volvemos a habilitar las interrupciones para que se puede realizar de nuevo el calculo de la distancia
  }

  /*Apartir de aqui utilizaremos las funciones de ROS para poder establecer una comunicacion*/
  //Rellenamos los campos de nuestra transformación (tf odom->base_link)
  odom_msg.header.frame_id = odom;
  odom_msg.child_frame_id = base_link;
  odom_msg.pose.pose.position.x = x_total;
  odom_msg.pose.pose.position.y = y_total;
  odom_msg.pose.pose.position.z = 0.0;
  odomPub.publish(&odom_msg);
  
  t.header = odom_msg.header;
  t.child_frame_id = odom_msg.child_frame_id;
  t.transform.translation.x = odom_msg.pose.pose.position.x;
  t.transform.translation.y = odom_msg.pose.pose.position.y;
  t.transform.translation.z = odom_msg.pose.pose.position.z;
  t.header.stamp = nh.now();

  //Finalmente publicamos la transformacion y esperamos un poco antes de volver hacerlo
  broadcaster.sendTransform(t);
  nh.spinOnce();
}
