const int IR_INPUT = A0;
bool victim_detect(){
  for (int i=0;i<10;i++){
    if (analogRead(IR_INPUT)<1000){
      return true;
    }
    delayMicroseconds(100);
  }
  return false;
}
void setup() {
  pinMode(IR_INPUT, INPUT);
  Serial.begin(9600);
}

void loop() {
  delay(1000);
  if (victim_detect()){
    Serial.println("VICTIM DETECTED :D");
  }else{
    Serial.println("NO VICTIM DETECTED :(");
  }

}
