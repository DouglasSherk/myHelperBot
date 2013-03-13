#define index1 9
#define chA 10
#define chB 11

void setup() {
   Serial.begin(38400);
   pinMode(index1, INPUT);
   pinMode(chA, INPUT);
   pinMode(chB, INPUT);
      
}

  int sumIndex = 0;
  int sumChA = 0;
  int sumChB = 0;
  
  int lastIndex = 0;
  int lastchA = 0;
  int lastchB = 0; 
  
  int currentIndex;
  int currentchA;
  int currentchB;
  
  int numMissedX = 0;
  int numMissedY = 0;
  
  int minMissed = 10000;
  
void loop(){
  
  currentIndex = digitalRead(index1);
  currentchA = digitalRead(chA);
  currentchB = digitalRead(chB);
  
  
  
  Serial.print("index1: ");
  Serial.print(currentIndex);
  Serial.print("\tchA: ");
  Serial.print(currentchA);
  Serial.print("\tchB: ");
  Serial.print(currentchB);
  Serial.println();
  
/*  currentIndex1 = digitalRead(index1);
  currentchA = digitalRead(chA);
  
  if (currentIndex1 != lastIndex1) {
     lastIndex1 = currentIndex1;
     sumIndex1 += 1; 
  }
  else {
    numMissedX += 1; 
  }
  
  if (currentIndex2 != lastIndex2) {
     lastIndex2 = currentIndex2;
     sumIndex2 += 1; 
  }
  else{
   numMissedY += 1; 
  }
  
  Serial.print("Index1: "); 
  Serial.print(currentIndex1);
  Serial.print("  ");
  Serial.print(sumIndex1);
  Serial.print("    Index2: "); 
  Serial.print(currentIndex2);
  Serial.print("  ");
  Serial.println(sumIndex2);
  
  if(numMissedX > minMissed || numMissedY > minMissed) {
     Serial.print("Index 1: ");
     Serial.print(sumIndex1);
     numMissedX = 0; 
     
     Serial.print("    Index 2: ");
     Serial.println(sumIndex2); 
     numMissedY = 0;
  }*/
  
}
