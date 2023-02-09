#include <Servo.h>

//#define ENABLE_RADAR
#define ENABLE_MOTORS

// servo
#define SERVO_PIN 5

// sensore ultrasonico
#define echoPin 2 
#define trigPin 3 

// sensori hall effect
#define hall_pin_r A0
#define hall_pin_l A1

// controlli per il ponte h
#define rf 10 // right forward
#define rb 9  // right backwards
#define lf 11 // left forwards
#define lb 12 // left backwards

#define WIDTH 21
#define LENGTH 21

// variabil per il sensore ultrasonico
long duration;  // variable for the duration of sound wave 
int distance;   // variable for the distance measurement

// servo
Servo myservo;  
int pos = 0;  //posizione corrente del servo

struct location {
  int x;
  int y;
  short **map;
};

struct location *loc;

// stampa la mappa. 
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

//inizializza la matrice della mappa. Imposta i limiti della matrice a -1 (ostacolo invalicabile)
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

// print array (al monitor seriale)
void printa(size_t size, int *arr){
  for(int i=0; i<size; i++){
    Serial.print(arr[i]);
    Serial.print(" ");
  }
  Serial.println("");
}

// mostra su console seriale la distanza misurata dal sensore ultrasonico in centimetri
void readDistance(){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2; 
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
}

// legge la distanza misurata dal sensore ultrasonico in centimetri
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

// scan deprecato
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

// scannerizza di 180 gradi l'ambiente verso il quale è rivolto il rover
// params:
//  steps: il numero di misurazioni
//  arr: l'array in cui verranno memorizzate le misurazioni (viene riallocato)
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

// ferma le ruote
void stop(){
  digitalWrite(rf, LOW);
  digitalWrite(rb, LOW);
  digitalWrite(lf, LOW);
  digitalWrite(lb, LOW);
}

// muovla ruota destra avanti
void right(){
  digitalWrite(rf, HIGH);
}

// muove la ruota destra indietro
void bright(){
  digitalWrite(rb, HIGH);
}

// muove la ruota sinistra avanti
void left(){
  digitalWrite(lf, HIGH);
}

// muove la ruota sinistra indietro
void bleft(){
  digitalWrite(lb, HIGH);
}

// muove avanti
void forw(){
  right();
  left();
}

// muove indietro
void backw(){
  bright();
  bleft();
}

// test per le ruote
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

//test per le ruote che stampa anche i valori dei sensori di hall
void wheels_test_hall(){
  right();
  read_hall(2000);
  

  stop();
  bright();
  read_hall(2000);
  stop();

  left();
  read_hall(2000);
  stop();
  bleft();
  read_hall(2000);
  stop();

  forw();
  read_hall(2000);
  stop();
  backw();
  read_hall(2000);
  stop();

  delay(1000);
}

void read_hall(int ms){
  for(int i=0; i<ms; i++){
    if(i%50==0){
      int hall_r = digitalRead(hall_pin_r);
      int hall_l = digitalRead(hall_pin_l);
      Serial.print(hall_l);
      Serial.print(", ");
      Serial.println(hall_r);
    }
    delay(1);
  }
}

// legge il valore del sensore di hall destro
// returns:
//   0 se non viene rilevato un campo magnetico
//   1 se viene rilevato
int read_hall_r(){
  return digitalRead(hall_pin_r);
}

// legge il valore del sensore di hall sinistro
// returns:
//   0 se non viene rilevato un campo magnetico
//   1 se viene rilevato
int read_hall_l(){
  return digitalRead(hall_pin_l);
}

// ruota la/e ruota/e clicks volte.
// params:
//   clicks: il numero di punti d'appoggio da contare
//   fun_rot: la funzione di rotazione
//   fun_hall: la funzione di lettura del valore del sensore di hall
void rotate_times(int clicks, void (*fun_rot)(), int (*fun_hall)()){
  int init_hall_state = fun_hall();
  int hall_state = 0;
  while(clicks--){
    if(init_hall_state){
      do{
        fun_rot();
        hall_state = fun_hall();
      }while(hall_state);
      do{
        fun_rot();
        hall_state = fun_hall();
      }while(!hall_state);
    }else{
      do{
        fun_rot();
        hall_state = fun_hall();
      }while(!hall_state);
    }
  }
  stop();
}


// Riposiziona le ruote sul primo punto di appoggio disponibile

void calibrate_hall(){
  rotate_times(1, right, read_hall_r);
  rotate_times(1, left, read_hall_l);
}

void light_sensor_test(){
  Serial.print("- ");
  right();
  int val = analogRead(A3);
  Serial.println(val);  
  
}

void setup() {
  pinMode(A3,INPUT);

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

  pinMode(hall_pin_r, INPUT);
  pinMode(hall_pin_l, INPUT);


  Serial.begin(9600);

  //inizializza e visualizza mappa 
  init_loc();
  p_map();
    
  Serial.println("---begin---");
  myservo.write(pos); //riposiziona servo

  //calibrate_hall(); //riposizioniamo le ruote
  delay(2000); //per riposizionare la macchinina
}

void loop() {
  
  // int *arr;
  // scanSteps(10, arr);
  // printa(10, arr);
  
  // wheels_test_hall();
  // delay(1000);

  // right();
  // read_hall(2000);
  // stop();
  // delay(1000);


  // rotate_times(10, right, read_hall_r);
  // delay(2000);

  // rotate_times(1, forw, read_hall_l); //21 cm
  // delay(1000);
  
  light_sensor_test();
  delay(10);
  
  // myservo.write(0);
  // delay(400); 
  // myservo.write(90);
  // delay(400); 
  // myservo.write(170);
  // delay(400); 
  
  //scan();
  //delay(100);
  //myservo.write(0);

  // int hall_r = digitalRead(hall_pin_r);
  // int hall_l = digitalRead(hall_pin_l);
  // Serial.print(hall_l);
  // Serial.print(", ");
  // Serial.println(hall_r);
  // delay(100);
}
