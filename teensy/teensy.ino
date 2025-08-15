#include <Servo.h> 
#include <Bounce.h>
 
Servo myservo;  
     
int gradosRaspberry = 0;
int target = 90;
int pos = 60;    
Bounce boton = Bounce(2, 10);
int pwm = 50;

int calKp(float newK, int input){
  
  int error = (input - 90) * newK;
  Serial.println(error);
  return error;
}

void updateServo(int k){
  myservo.write(constrain(target+k,40,140));
  Serial.println(target+k);
  
}

int serial1Update() {
  // Formato NUEVO:
  // [0xAB][ANG_H][ANG_L][VEL_H][VEL_L][KP_H][KP_L][CHK]
  // CHK = XOR de bytes 0..6 (incluye 0xAB)
  enum State : uint8_t { WAIT_HDR, ANG_H, ANG_L, VEL_H, VEL_L, KP_H, KP_L, GET_CK };
  static State st = WAIT_HDR;

  static uint8_t ang_hi = 0, ang_lo = 0;
  static uint8_t vel_hi = 0, vel_lo = 0;
  static uint8_t kp_hi  = 0, kp_lo  = 0;
  static uint8_t xor_acc = 0;

  while (Serial1.available()) {
    const uint8_t b = (uint8_t)Serial1.read();

    switch (st) {
      case WAIT_HDR:
        if (b == 0xAB) { xor_acc = b; st = ANG_H; }
        break;

      case ANG_H:
        ang_hi = b; xor_acc ^= b; st = ANG_L; break;

      case ANG_L:
        ang_lo = b; xor_acc ^= b; st = VEL_H; break;

      case VEL_H:
        vel_hi = b; xor_acc ^= b; st = VEL_L; break;

      case VEL_L:
        vel_lo = b; xor_acc ^= b; st = KP_H;  break;

      case KP_H:
        kp_hi = b;  xor_acc ^= b; st = KP_L;  break;

      case KP_L:
        kp_lo = b;  xor_acc ^= b; st = GET_CK; break;

      case GET_CK: {
        const bool ok = (b == xor_acc);
        st = WAIT_HDR;  // listo para el siguiente frame

        if (ok) {
          // Ángulo en décimas (0..3600 esperado)
          uint16_t ang_tenths = (uint16_t(ang_hi) << 8) | ang_lo;
          if (ang_tenths > 3600) ang_tenths = 3600;

          // (Opcional) si quieres guardar vel/kp, declara globales y descomenta:
          // extern volatile uint16_t g_vel_mmps, g_kp_milli;
           // pwm = (uint16_t(vel_hi) << 8) | vel_lo;
          // g_kp_milli = (uint16_t(kp_hi)  << 8) | kp_lo;

          // Devuelve grados enteros 0..360
          int deg = (int)((ang_tenths + 5) / 10);
          if (deg < 0)   deg = 0;
          if (deg > 360) deg = 360;
          return deg;
        } else {
          // Re-sincroniza rápido si el byte de checksum parece un nuevo header
          if (b == 0xAB) { xor_acc = b; st = ANG_H; }
        }
      } break;
    }
  }

  // No se completó un frame válido en esta llamada
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
  gradosRaspberry = serial1Update();
  analogWrite(6,pwm);
 // updateServo(calKp(gradosRaspberry));
 if(gradosRaspberry != -1){
 // Serial.println(gradosRaspberry);
 // Serial1.println(calKp(0.5,gradosRaspberry));
  updateServo(calKp(3,gradosRaspberry));}
} 