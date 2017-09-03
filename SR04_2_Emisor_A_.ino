/* EMISOR (A)
 *  Codigo que envia un caracter para poder identificar el emisor.
 *  Esto se enviara al mismo tiempo que se envia el pulso del ultrasonido emisor
 * 
 * Autor: Alberto Martínez Chincho
 * Universidad de La Laguna
 */
//libreria necesaria para los modulos RF
#include <VirtualWire.h>
#include <TimerOne.h>

const int pin_RF_Emisor = 12; //pin del emisor
const int pin_RF_Receptor = 11; //pin del receptor
const int pin_SR04_Trigger = 13; //pin del ultrasonido (solo trigger)

volatile int contador = 0; //contador para la interrupcion que inicia la emision

//Creamos un mensaje
uint8_t sms[VW_MAX_MESSAGE_LEN];
uint8_t sms_len = VW_MAX_MESSAGE_LEN;

void setup() {
  Serial.begin(9600);
  pinMode(pin_SR04_Trigger, OUTPUT); //activamos el pin del ultrasonido como salida
  
  vw_setup(2000); //inicializamos la libreria
  vw_set_tx_pin(pin_RF_Emisor); //Configuramos el pin de emision
  vw_set_rx_pin(pin_RF_Receptor); //Configuramos el pin de recepcion
  vw_rx_start(); //Activamos el proceso de escucha
  Serial.println("Enviando...");

  Timer1.initialize(65000);
  Timer1.attachInterrupt(inicio_sensores);
}

void inicio_sensores(void){
  contador++;
 // Serial.println(contador);
  if(contador >= 10 ){
   // Serial.println("IF de la interrupcion");
    //Comunicacion RF
    const char *mensaje = "A"; //mensaje identificativo que enviaremos al receptor
    vw_send((uint8_t *)mensaje, strlen(mensaje)); //transmite el mensaje con la long dada
    vw_wait_tx(); //esperamos que el mensaje sea transmitido en su totalidad      
    //SR04(Ultrasonido)
    digitalWrite(pin_SR04_Trigger, LOW); //Para estabilizar el sensor
    delayMicroseconds(5);
    digitalWrite(pin_SR04_Trigger, HIGH); //Activamos el envio del pulso
    contador = 0;
  }
}

void loop() {
  if(vw_get_message(sms, &sms_len)){
    if(sms[0] == 'C'){ //Emitimos cuando recibamos el mensaje del emisor C
      //delay(2000);
      //Comunicacion RF
      const char *mensaje = "A"; //mensaje identificativo que enviaremos al receptor
      vw_send((uint8_t *)mensaje, strlen(mensaje)); //transmite el mensaje con la long dada
      vw_wait_tx(); //esperamos que el mensaje sea transmitido en su totalidad
  
      //SR04(Ultrasonido)
      digitalWrite(pin_SR04_Trigger, LOW); //Para estabilizar el sensor
      delayMicroseconds(5);
      digitalWrite(pin_SR04_Trigger, HIGH); //Activamos el envio del pulso

      //Ponemos el contador a cero para que en la interrupcion no se envie señales si todo va bien
      contador = 0;
    } 
  }
}
