#include <Servo.h> 
#include <Bounce.h>
 
Servo myservo;  
     
int gradosRaspberry = 0;
int target = 90;
int pos = 60;    
Bounce boton = Bounce(2, 10);

int calKp(){

  int error = gradosRaspberry * 0.5;
  return error;
}

void updateServo(int k){
  myservo.write(constrain(target+k,40,135));
  Serial.println(target+k);
  }
}

int serial1Update() {
  // Máquina de estados y buffers persistentes entre llamadas
  enum { WAIT_HDR, GET_HI, GET_LO, GET_CK };
  static uint8_t st = WAIT_HDR;
  static uint8_t hi = 0, lo = 0;

  while (Serial1.available()) {
    uint8_t b = (uint8_t)Serial1.read();

    switch (st) {
      case WAIT_HDR:
        if (b == 0xAA) st = GET_HI;
        break;

      case GET_HI:
        hi = b;
        st = GET_LO;
        break;

      case GET_LO:
        lo = b;
        st = GET_CK;
        break;

      case GET_CK: {
        uint8_t chk = (uint8_t)(0xAA ^ hi ^ lo);
        st = WAIT_HDR;  // reset para buscar siguiente frame

        if (b == chk) {
          // Valor en décimas de grado (0..3600 esperado)
          uint16_t raw = (uint16_t(hi) << 8) | lo;

          // Asegurar límites: si llega algo fuera, se satura
          if (raw > 3600) raw = 3600;

          // Redondeo a grado entero y clamp final 0..360
          int deg = (int)((raw + 5) / 10); // 0..360
          if (deg < 0) deg = 0;
          if (deg > 360) deg = 360;

          return deg;  // ÉXITO: devolvemos el ángulo
        }
        // Si el checksum no coincide, se descarta y seguimos leyendo
      } break;
    }
  }

  // No se completó ningún frame válido en esta llamada
  return -1;
}


void setup() 
{ 
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
  Serial.begin(115200);
  Serial1.begin(115200);  // UART1 a la Raspberry Pi
  myservo.attach(8);  // attaches the servo on pin 20 
  while(!boton.update()){
  delay(100);
  Serial.println("esperando");  
  }
  digitalWrite(5,HIGH);
  digitalWrite(7,LOW);
  analogWriteFrequency(6,20000);
  //myservo.write(40);
  //limite superior 135 y limite inferior 40 

} 
 
 
void loop() 
{ 
  /*analogWrite(6,100);
  if(Serial.available()){
    pos = Serial.read();
    Serial.println(pos);
    Serial.clear();
  }
  myservo.write(pos);*/
} 