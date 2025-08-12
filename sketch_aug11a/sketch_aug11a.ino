#include <Servo.h> 
#include <Bounce.h>
 
Servo myservo;  
     
 
int pos = 60;    
Bounce boton = Bounce(2, 10);
void setup() 
{ 
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
  Serial.begin(115200);
  myservo.attach(8);  // attaches the servo on pin 20 
  while(!boton.update()){
  delay(100);
  Serial.println("esperando");  
  }
  digitalWrite(5,HIGH);
  digitalWrite(7,LOW);
  analogWriteFrequency(6,20000);
    myservo.write(pos);
} 
 
 
void loop() 
{ 
  analogWrite(6,100);
  if(Serial.available()){
    pos = Serial.read();
    Serial.println(pos);
    Serial.clear();
  }
  myservo.write(pos);
} 