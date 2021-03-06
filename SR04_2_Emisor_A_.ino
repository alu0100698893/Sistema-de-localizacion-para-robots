/* EMISOR (A)
 *  Codigo que envia un caracter para poder identificar el emisor.
 *  Esto se enviara al mismo tiempo que se envia el pulso del ultrasonido emisor
 * 
 * Autor: Alberto Martínez Chincho
 * Universidad de La Laguna
 */
//libreria necesaria para los modulos RF
#include <VirtualWire.h>

const int pin_RF_Emisor = 11; //pin del emisor
const int pin_RF_Receptor = 9; //pin del receptor
const int pin_SR04_Trigger = 13; //pin del ultrasonido (solo trigger)

double Ultimo_envio = 0; //para saber el ultimo envio que realiza 
int i; //variable para el for que envia X veces seguidas el mensaje de RF
int j; //variable para el for que envia X veces seguidas el mensaje de RF

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
}

void loop() {
  if(vw_get_message(sms, &sms_len)){
    if(sms[0] == 'C'){ //Emitimos cuando recibamos el mensaje del emisor C
      Serial.println("Mensaje C Recibido");
      delay(1000);
      for( j = 0; j < 10; j++ ){
        //Comunicacion RF
        const char *mensaje = "A"; //mensaje identificativo que enviaremos al receptor
        vw_send((uint8_t *)mensaje, strlen(mensaje)); //transmite el mensaje con la long dada
        vw_wait_tx(); //esperamos que el mensaje sea transmitido en su totalidad  
      }
      if(j == 10){
        //SR04(Ultrasonido)
        digitalWrite(pin_SR04_Trigger, LOW); //Para estabilizar el sensor
        delayMicroseconds(5);
        digitalWrite(pin_SR04_Trigger, HIGH); //Activamos el envio del pulso
      }
      j = 0;
      //Ponemos el contador a cero para que en la interrupcion no envie señales si todo va bien
      Ultimo_envio = millis();
    } 
  }
  if (millis() - Ultimo_envio > 20000) {
    Serial.println("Nuevo Mensaje A");
    delay(1000); 
    for( i = 0; i < 10; i++ ){
      Ultimo_envio = millis();
      const char *mensaje = "A"; //mensaje identificativo que enviaremos al receptor
      vw_send((uint8_t *)mensaje, strlen(mensaje)); //transmite el mensaje con la long dada
      vw_wait_tx(); //esperamos que el mensaje sea transmitido en su totalidad
    }
    if(i == 10){      
      //SR04(Ultrasonido)
      digitalWrite(pin_SR04_Trigger, LOW); //Para estabilizar el sensor
      delayMicroseconds(5);
      digitalWrite(pin_SR04_Trigger, HIGH); //Activamos el envio del pulso
    }
    i = 0;
  }
}
