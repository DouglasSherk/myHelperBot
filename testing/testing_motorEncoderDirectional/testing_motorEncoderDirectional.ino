  void setup() {
  pinMode(6,INPUT);
  pinMode(7,INPUT);
  pinMode(8,INPUT); 
  Serial.begin(38400);
 }
 
 void loop() {
  Serial.print("ind: ");
  Serial.print(digitalRead(6));
  Serial.print("chA: ");
  Serial.print(digitalRead(7));
  Serial.print("chB: ");
  Serial.println(digitalRead(8)); 
 }
