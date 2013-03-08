#define power1 8
#define ground1 9
#define index1 4
#define power2 10
#define ground2 11
#define index2 5

// Just some math to turn wheel odometry into position updates
// Released into the public domain 3 June 2010

#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#define PI (3.14159)

#define WHEELBASE (340.0)

// left wheel
double Lx = -WHEELBASE/2.0;
double Ly = 0.0;

// right wheel
double Rx = WHEELBASE/2.0;
double Ry = 0.0;

void update_wheel_position(double l, double r) {
 // http://en.wikipedia.org/wiki/Dead_reckoning#Differential_steer_drive_dead_reckoning
}













void update_wheel_position_old2(double l, double r) {

  /*Serial.print("l: ");
  Serial.print(l);
  Serial.print("  r: ");
  Serial.println(r);*/
  
  if (abs(r - l) < 5) {
    // If both wheels moved about the same distance, then we get an infinite
    // radius of curvature.  This handles that case.

    // find forward by rotating the axle between the wheels 90 degrees
    double axlex = Rx - Lx;
    double axley = Ry - Ly;

    double forwardx, forwardy;
    forwardx = -axley;
    forwardy = axlex;

    // normalize
    double length = sqrt(forwardx*forwardx + forwardy*forwardy);
    forwardx = forwardx / length;
    forwardy = forwardy / length;

    // move each wheel forward by the amount it moved
    Lx = Lx + forwardx * l;
    Ly = Ly + forwardy * l;

    Rx = Rx + forwardx * r;
    Ry = Ry + forwardy * r;
    
    return;
  }

  if(r>l) {
      // radius of curvature for left wheel
      double rc = abs(WHEELBASE * l / (r - l));
      Serial.print("rc: ");
      Serial.print(rc);
      Serial.print("\t");
      
      // angle we moved around the circle, in radians
      double theta = r/(rc+WHEELBASE);
      Serial.print("theta: ");
      Serial.print(theta);
      Serial.print("\t");
      
      // Find the point P that we're circling
      double Px, Py;
      Px = Lx + rc*((Lx-Rx)/WHEELBASE);
      Py = Ly + rc*((Ly-Ry)/WHEELBASE);
      Serial.print("center point: ");
      Serial.print("(");
      Serial.print(Px);
      Serial.print(", ");
      Serial.print(Py);
      Serial.print(")");
      Serial.print("\t");
      
      // Translate everything to the origin
      double Lx_translated = Lx - Px;
      double Ly_translated = Ly - Py;
      double Rx_translated = Rx - Px;
      double Ry_translated = Ry - Py;
      
      // Rotate by theta
      double cos_theta = cos(theta);
      double sin_theta = sin(theta);
        
      double Lx_rotated = Lx_translated*cos_theta - Ly_translated*sin_theta;
      double Ly_rotated = Lx_translated*sin_theta + Ly_translated*sin_theta;
    
      double Rx_rotated = Rx_translated*cos_theta - Ry_translated*sin_theta;
      double Ry_rotated = Rx_translated*sin_theta + Ry_translated*sin_theta;
    
      // Translate back  
      Lx = Lx_rotated + Px;
      Ly = Ly_rotated + Py;
    
      Rx = Rx_rotated + Px;
      Ry = Ry_rotated + Py;
  }
  else {
      double rc = abs(WHEELBASE * r / (r - l));
      
      double theta = l/(rc+WHEELBASE);
      
      double Px, Py;
      Px = Rx + rc*((Rx-Lx)/WHEELBASE);
      Py = Ry + rc*((Ry-Ly)/WHEELBASE);
      
      double Lx_translated = Lx - Px;
      double Ly_translated = Ly - Py;
      double Rx_translated = Rx - Px;
      double Ry_translated = Ry - Py;
      
      double cos_theta = cos(theta);
      double sin_theta = sin(theta);
       
      double Lx_rotated = Lx_translated*cos_theta - Ly_translated*sin_theta;
      double Ly_rotated = Lx_translated*sin_theta + Ly_translated*sin_theta;
    
      double Rx_rotated = Rx_translated*cos_theta - Ry_translated*sin_theta;
      double Ry_rotated = Rx_translated*sin_theta + Ry_translated*sin_theta;
       
      Lx = Lx_rotated + Px;
      Ly = Ly_rotated + Py;
    
      Rx = Rx_rotated + Px;
      Ry = Ry_rotated + Py;
  }
}

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
   
   Serial.println("initial position: ");
     Serial.print("(");
     Serial.print(Lx);
     Serial.print(", ");
     Serial.print(Ly);
     Serial.print(") (");
     Serial.print(Rx);
     Serial.print(", ");
     Serial.print(Ry);
     Serial.println(")");
}
  double mmPerCount = 2.234301;
  int sumIndex1 = 0;
  int sumIndex2 = 0;
  int totalSumIndex1 = 0;
  int totalSumIndex2 = 0;
  
  int lastIndex1 = 0;
  int lastIndex2 = 0;
  
  int currentIndex1;
  int currentIndex2;
  
  int numMissedX = 0;
  int numMissedY = 0;
  
  int minMissed = 10000;
  
/*void loop(){

  currentIndex1 = digitalRead(index1);
  currentIndex2 = digitalRead(index2);
  
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
  
  if(numMissedX > minMissed || numMissedY > minMissed && (sumIndex1)) {     
     update_wheel_position(sumIndex1*mmPerCount, sumIndex2*mmPerCount);
     totalSumIndex1 += sumIndex1;
     totalSumIndex2 += sumIndex2;
     sumIndex1 = 0;
     sumIndex2 = 0;
     numMissedX = 0;
     numMissedY = 0;
     Serial.print("Lx: ");
     Serial.print(Lx);
     Serial.print("  Ly: ");
     Serial.print(Ly);
     Serial.print("  Rx: ");
     Serial.print(Rx);
     Serial.print("  Ry: ");
     Serial.print(Ry);
     Serial.print("  sum1: ");
     Serial.print(totalSumIndex1);
     Serial.print("  sum2: ");
     Serial.println(totalSumIndex2);
  }
}*/

void loop() {
     update_wheel_position(0,534.0);
     Serial.print("(");
     Serial.print(Lx);
     Serial.print(", ");
     Serial.print(Ly);
     Serial.print(") (");
     Serial.print(Rx);
     Serial.print(", ");
     Serial.print(Ry);
     Serial.println(")");
}
























































































































// given distances traveled by each wheel, updates the
// wheel position globals
void update_wheel_position_old(double l, double r) {

  /*Serial.print("l: ");
  Serial.print(l);
  Serial.print("  r: ");
  Serial.println(r);*/
  
  if (abs(r - l) < 5) {
    // If both wheels moved about the same distance, then we get an infinite
    // radius of curvature.  This handles that case.

    // find forward by rotating the axle between the wheels 90 degrees
    double axlex = Rx - Lx;
    double axley = Ry - Ly;

    double forwardx, forwardy;
    forwardx = -axley;
    forwardy = axlex;

    // normalize
    double length = sqrt(forwardx*forwardx + forwardy*forwardy);
    forwardx = forwardx / length;
    forwardy = forwardy / length;

    // move each wheel forward by the amount it moved
    Lx = Lx + forwardx * l;
    Ly = Ly + forwardy * l;

    Rx = Rx + forwardx * r;
    Ry = Ry + forwardy * r;
    
    return;
  }

  double inner, outer;
  if(r>l) {
      // radius of curvature for left wheel
      double rc = abs(WHEELBASE * l / (r - l));
      
      // angle we moved around the circle, in radians
      double theta = l/rc;
      
      // Find the point P that we're circling
      double Px, Py;
      Px = Lx + rc*((Lx-Rx)/WHEELBASE);
      Py = Ly + rc*((Ly-Ry)/WHEELBASE);
      
      // Translate everything to the origin
      double Lx_translated = Lx - Px;
      double Ly_translated = Ly - Py;
      double Rx_translated = Rx - Px;
      double Ry_translated = Ry - Py;
      
      // Rotate by theta
      double cos_theta = cos(theta);
      double sin_theta = sin(theta);
        
      double Lx_rotated = Lx_translated*cos_theta - Ly_translated*sin_theta;
      double Ly_rotated = Lx_translated*sin_theta + Ly_translated*sin_theta;
    
      double Rx_rotated = Rx_translated*cos_theta - Ry_translated*sin_theta;
      double Ry_rotated = Rx_translated*sin_theta + Ry_translated*sin_theta;
    
      // Translate back  
      Lx = Lx_rotated + Px;
      Ly = Ly_rotated + Py;
    
      Rx = Rx_rotated + Px;
      Ry = Ry_rotated + Py;
  }
  else {
      double rc = abs(WHEELBASE * r / (r - l));
      
      double theta = r/rc;
      
      double Px, Py;
      Px = Rx + rc*((Rx-Lx)/WHEELBASE);
      Py = Ry + rc*((Ry-Ly)/WHEELBASE);
      
      double Lx_translated = Lx - Px;
      double Ly_translated = Ly - Py;
      double Rx_translated = Rx - Px;
      double Ry_translated = Ry - Py;
      
      double cos_theta = cos(theta);
      double sin_theta = sin(theta);
       
      double Lx_rotated = Lx_translated*cos_theta - Ly_translated*sin_theta;
      double Ly_rotated = Lx_translated*sin_theta + Ly_translated*sin_theta;
    
      double Rx_rotated = Rx_translated*cos_theta - Ry_translated*sin_theta;
      double Ry_rotated = Rx_translated*sin_theta + Ry_translated*sin_theta;
       
      Lx = Lx_rotated + Px;
      Ly = Ly_rotated + Py;
    
      Rx = Rx_rotated + Px;
      Ry = Ry_rotated + Py;
  }


  double rl; // radius of curvature for left wheel
  rl = WHEELBASE * l / (r - l);

  //printf("Radius of curvature (left wheel): %.2lf\n", rl);
  Serial.print("radius of curvature:  ");
  Serial.println(rl);
  
  double theta; // angle we moved around the circle, in radians
  // theta = 2 * PI * (l / (2 * PI * rl)) simplifies to:
  theta = l / rl;

  //printf("Theta: %.2lf radians\n", theta);
  Serial.print("theta: ");
  Serial.println(theta);

  // Find the point P that we're circling
  double Px, Py;

  Px = Lx + rl*((Lx-Rx)/WHEELBASE);
  Py = Ly + rl*((Ly-Ry)/WHEELBASE);

  //printf("Center of rotation: (%.2lf, %.2lf)\n", Px, Py);
  Serial.print("center of rotation: ");
  Serial.print(Px);
  Serial.print(", ");
  Serial.println(Py);

  // Translate everything to the origin
  double Lx_translated = Lx - Px;
  double Ly_translated = Ly - Py;

  double Rx_translated = Rx - Px;
  double Ry_translated = Ry - Py;

  //printf("Translated: (%.2lf,%.2lf) (%.2lf,%.2lf)\n",
  //  Lx_translated, Ly_translated,
  //  Rx_translated, Ry_translated);
     /*Serial.print("translated ");
     Serial.print("(");
     Serial.print(Lx);
     Serial.print(", ");
     Serial.print(Ly);
     Serial.print(") (");
     Serial.print(Rx);
     Serial.print(", ");
     Serial.print(Ry);
     Serial.println(")");*/

  // Rotate by theta
  double cos_theta = cos(theta);
  double sin_theta = sin(theta);

  //printf("cos(theta)=%.2lf sin(theta)=%.2lf\n", cos_theta, sin_theta);

  double Lx_rotated = Lx_translated*cos_theta - Ly_translated*sin_theta;
  double Ly_rotated = Lx_translated*sin_theta + Ly_translated*sin_theta;

  double Rx_rotated = Rx_translated*cos_theta - Ry_translated*sin_theta;
  double Ry_rotated = Rx_translated*sin_theta + Ry_translated*sin_theta;

  //printf("Rotated: (%.2lf,%.2lf) (%.2lf,%.2lf)\n",
  //  Lx_rotated, Ly_rotated,
  //  Rx_rotated, Ry_rotated);

  // Translate back
  Lx = Lx_rotated + Px;
  Ly = Ly_rotated + Py;

  Rx = Rx_rotated + Px;
  Ry = Ry_rotated + Py;
}

/*main(int argc, char **argv) {
  if (argc != 3) {
    printf("Usage: %s left right\nwhere left and right are distances.\n",
      argv[0]);
    return 1;
  }

  double left = atof(argv[1]);
  double right = atof(argv[2]);

  printf("Old wheel positions: (%lf,%lf) (%lf,%lf)\n",
    Lx, Ly, Rx, Ry);
  update_wheel_position(left, right);
  printf("New wheel positions: (%lf,%lf) (%lf,%lf)\n",
    Lx, Ly, Rx, Ry);
}*/
