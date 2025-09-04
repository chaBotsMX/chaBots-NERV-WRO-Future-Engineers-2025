#include <Servo.h> 
#include <Bounce.h>
 
Servo myservo;  
     
int gradosRaspberry = 0;
int target = 90;
int pos = 60;    
Bounce boton = Bounce(2, 10);
int pwm = 0;
int dir = 1;
int lastError = 0;
int calKp(float newD,float newK, int input){
  
  float error = ((input - 90) * newK) + ((input - 90) - lastError * newD);
  lastError = input - 90;
  //Serial.println(error);           
  return int(error);
}

void updateServo(int k){
  myservo.write(constrain(target+k,60,120));
  Serial.println(target+k);
  
}
int serial1Update() {
  enum State : uint8_t { WAIT_HDR, ANG_H, ANG_L, PWM_B, KP_B, GET_CK };
  static State   st = WAIT_HDR;
  static uint8_t ang_hi = 0, ang_lo = 0;
  static uint8_t kp = 0;
  static uint8_t xor_acc = 0;

  while (Serial1.available()) {
    const uint8_t b = (uint8_t)Serial1.read();

    switch (st) {
      case WAIT_HDR:
        if (b == 0xAB) { xor_acc = b; st = ANG_H; }
        break;

      case ANG_H:
        ang_hi = b; xor_acc ^= b; st = ANG_L; 
        break;

      case ANG_L:
        ang_lo = b; xor_acc ^= b; st = PWM_B; 
        break;

      case PWM_B:
        pwm = b;    xor_acc ^= b; st = KP_B;  
        break;

      case KP_B:
        kp  = b;    xor_acc ^= b; st = GET_CK; 
        break;

      case GET_CK: {
        const bool ok = (b == xor_acc);
        // siempre resetea para el siguiente frame
        st = WAIT_HDR;

        if (ok) {
          uint16_t ang_tenths = (uint16_t(ang_hi) << 8) | ang_lo;
          if (ang_tenths > 3600) ang_tenths = 3600;

          // DEBUG opcional: imprime el frame decodificado
          // Serial.print("ANG_0.1deg="); Serial.print(ang_tenths);
          // Serial.print("  PWM="); Serial.print(pwm);
          // Serial.print("  KP=");  Serial.println(kp);
          dir = kp;
          int deg = ang_tenths;  // redondeo a grados
      
          // aquí puedes guardar pwm/kp globalmente si los necesitas
          // g_pwm_byte = pwm; g_kp_byte = kp;
          return deg;
        } else {
          // checksum inválido: simplemente espera al próximo header (WAIT_HDR)
          // No autosync inmediato en medio del frame.
        }
      } break;
    }
  }

  return -1; // no completó un frame válido
}


void setup() 
{ 
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
  Serial.begin(115200);
  Serial1.begin(2000000);  // UART1 a la Raspberry Pi
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

  gradosRaspberry = serial1Update();
  if(pwm == 0){
    digitalWrite(5,LOW);
    digitalWrite(7,LOW);
  }
  else if(dir == 1){
    digitalWrite(5,LOW);
    digitalWrite(7,HIGH);
  }
  else{
    digitalWrite(5,HIGH);
    digitalWrite(7,LOW);
  }
  analogWrite(6,pwm);
  //updateServo(calKp(gradosRaspberry));
 if(gradosRaspberry != -1){
  Serial.println(dir);
  //Serial.println(gradosRaspberry);
  //serial1.println(calKp(0.5,gradosRaspberry));
  updateServo(calKp(0.01,0.5 ,gradosRaspberry));
  }
} 