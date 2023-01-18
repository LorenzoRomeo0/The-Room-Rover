#include <Servo.h>

//#define ENABLE_RADAR
//#define ENABEL_MOTORS
#define SERVO_PIN 5
#define echoPin 2 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin 3 //attach pin D3 Arduino to pin Trig of HC-SR04

#define rf 12
#define rb 11
#define lf 9
#define lb 10

#define WIDTH 21
#define LENGTH 21

// defines variables
long duration; // variable for the duration of sound wave travel
int distance; // variable for the distance measurement
Servo myservo;  // create servo object to control a servo
int pos = 0; 

struct location {
  int x;
  int y;
  short **map;
};

struct location *loc;

void p_map(){
  char buf[5];
  for(short i=0; i<LENGTH; i++){
    for(short j=0; j<WIDTH; j++){
      if(loc -> x == i && loc -> y == j){
        Serial.print("  x ");
        continue;
      }
      sprintf(buf, "%3d ", loc -> map[i][j]);
      Serial.print(buf);
    }
    Serial.println(" ");
  }
}

void init_loc(){
  loc = malloc(sizeof(struct location));
  loc -> x = LENGTH/2;
  loc -> y = WIDTH/2;
  loc -> map = (short**) calloc(LENGTH, sizeof(short*));
  for(short i = 0; i<WIDTH; i++){
    loc -> map[i] = (short*) calloc(WIDTH, sizeof(short));
  }

  for(short i=0; i<LENGTH; i++){
    for(short j=0; j<WIDTH; j++){
      if(i == 0 || j == 0 || i==WIDTH-1 || j==LENGTH-1) loc->map[i][j] = -1;
    }
  }  
}

void setup() {
  #ifdef ENABLE_RADAR
    myservo.attach(SERVO_PIN);
    pinMode(trigPin, OUTPUT); 
    pinMode(echoPin, INPUT); 
  #endif
  #ifdef ENABLE_MOTORS
    pinMode(rf, OUTPUT); 
    pinMode(rb, OUTPUT);
    pinMode(lf, OUTPUT);
    pinMode(lb, OUTPUT);
  #endif

  Serial.begin(9600); // // Serial Communication is starting with 9600 of baudrate speed

  init_loc();
  p_map();
    
  Serial.println("---"); // print some text in Serial Monitor
  myservo.write(pos);
  delay(1000);
}


void printa(size_t size, int *arr){
  for(int i=0; i<size; i++){
    Serial.print(arr[i]);
    Serial.print(" ");
  }
  Serial.println("");
}

void readDistance(){
  // Clears the trigPin condition
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  //delay(100);
}

int readDistance_int(){

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);

  distance = duration * 0.034 / 2;
  return distance;
}

void scan(){
  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    readDistance();
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    readDistance();
    delay(15);                       // waits 15ms for the servo to reach the position
  } 
}

void scanSteps(int steps, int *arr){
  arr = calloc(steps, sizeof(int));
  for (pos = 0; pos <= 180-1; pos += (180-1)/steps) { // goes from 0 degrees to 180 degrees
    //myservo.attach(SERVO_PIN);
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    //myservo.detach();
    int distance = readDistance_int();
    arr[pos] = distance;
    Serial.println(distance);
    delay(400);                       
  }
}

void stop(){
  digitalWrite(rf, LOW);
  digitalWrite(rb, LOW);
  digitalWrite(lf, LOW);
  digitalWrite(lb, LOW);
}

void right(){
  digitalWrite(rf, HIGH);
}

void bright(){
  digitalWrite(rb, HIGH);
}

void left(){
  digitalWrite(lf, HIGH);
}

void bleft(){
  digitalWrite(lb, HIGH);
}

void forw(){
  right();
  left();
}

void backw(){
  bright();
  bleft();
}

void wheels_test(){
  right();
  delay(2000);
  stop();
  bright();
  delay(2000);
  stop();

  left();
  delay(2000);
  stop();
  bleft();
  delay(2000);
  stop();

  forw();
  delay(2000);
  stop();
  backw();
  delay(2000);
  stop();

  delay(1000);
}

void loop() {
  /*
  int *arr;
  scanSteps(10, arr);
  printa(10, arr);
  */
  /*
  myservo.write(0);
  delay(400); 
  myservo.write(90);
  delay(400); 
  myservo.write(170);

  */
  
}
