#define power1 8
#define ground1 9
#define index1 4
#define power2 10
#define ground2 11
#define index2 5

void setup() {
   Serial.begin(38400);
   pinMode(index1, INPUT);
   pinMode(ground1, OUTPUT);
   digitalWrite(ground1, LOW);
   pinMode(power1, OUTPUT);
   digitalWrite(power1, HIGH);
   
   pinMode(index2, INPUT);
   pinMode(ground2, OUTPUT);
   digitalWrite(ground2, LOW);
   pinMode(power2, OUTPUT);
   digitalWrite(power2, HIGH);   
}

  int sumIndex1 = 0;
  int sumIndex2 = 0;
  
  int lastIndex1 = 0;
  int lastIndex2 = 0;
  
  int currentIndex1;
  int currentIndex2;
  
void loop(){

  currentIndex1 = digitalRead(index1);
  currentIndex2 = digitalRead(index2);
  
  if (currentIndex1 != lastIndex1) {
     lastIndex1 = currentIndex1;
     sumIndex1 += 1; 
  }
  
  if (currentIndex2 != lastIndex2) {
     lastIndex2 = currentIndex2;
     sumIndex2 += 1; 
  }
  
  Serial.print("Index1: "); 
  Serial.print(currentIndex1);
  Serial.print("  ");
  Serial.print(sumIndex1);
  Serial.print("    Index2: "); 
  Serial.print(currentIndex2);
  Serial.print("  ");
  Serial.println(sumIndex2); 
}
