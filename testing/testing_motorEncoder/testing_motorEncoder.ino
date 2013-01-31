#define ground 8
#define power 9
#define Index 10
#define CHA 11
#define CHB 12

void setup() {
   Serial.begin(38400);
   pinMode(Index, INPUT);
   pinMode(CHA, INPUT);
   pinMode(CHB, INPUT);
   pinMode(ground, OUTPUT);
   digitalWrite(ground, LOW);
   pinMode(power, OUTPUT);
   digitalWrite(power, HIGH);   
}

  int sumIndex = 0;
  int sumCHA = 0;
  int sumCHB = 0;
  
  int lastIndex = 0;
  int lastCHA = 0;
  int lastCHB = 0;
  
void loop(){

  int currentIndex = digitalRead(Index);
  int currentCHA = digitalRead(CHA);
  int currentCHB = digitalRead(CHB);
  
  if (currentIndex != lastIndex) {
     lastIndex = currentIndex;
     sumIndex += 1; 
  }
  
    if (currentCHA != lastCHA) {
     lastCHA = currentCHA;
     sumCHA += 1; 
  }
  
  if (currentCHB != lastCHB) {
     lastCHB = currentCHB;
     sumCHB += 1; 
  }
  
  Serial.print("Index: "); 
  Serial.print(currentIndex);
  Serial.print("  ");
  Serial.print(sumIndex);
  Serial.print("  CHA: ");
  Serial.print(currentCHA);
  Serial.print("  ");
  Serial.print(sumCHA);
  Serial.print("  CHB: ");
  Serial.print(sumCHB);
  Serial.print("  ");
  Serial.println(currentCHB);  
}
