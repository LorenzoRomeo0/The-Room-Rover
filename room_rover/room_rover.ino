#include <Servo.h>

#define ENABLE_RADAR
#define ENABLE_MOTORS


// circonferenza ruote (cm)
#define WHELL_CIRC 21


// servo
#define SERVO_PIN 5

// sensore ultrasonico
#define echoPin 2
#define trigPin 3

// sensori hall effect
#define hall_pin_r A0
#define hall_pin_l A1

// controlli per il ponte h
#define rf 10  // right forward
#define rb 9   // right backwards
#define lf 11  // left forwards
#define lb 12  // left backwards


// dimensioni della mappa
#define SIZE 21

#define MEASURE_NR 21
#define UNIT 10
#define SCAN_STEP_TIME 400

// variabili per il sensore ultrasonico
long duration;  // variable for the duration of sound wave
int distance;   // variable for the distance measurement

// servo
Servo myservo;
int pos = 0;  //posizione corrente del servo

enum orientation { North,
                   South,
                   East,
                   West };

struct location {
  enum orientation orient;  //orientamento del rover
  int x;                    // x corrente
  int y;                    // y corrente
  short **map;              //matrice che rappresenta la mappa dell'ambiente circostante
};
struct location *loc;

// stampa la mappa (sul monitor seriale).
void p_map() {
  char buf[5];
  for (byte i = 0; i < SIZE; i++) {
    for (byte j = 0; j < SIZE; j++) {
      if (loc->x == i && loc->y == j) {
        Serial.print("  O ");
        continue;
      }
      sprintf(buf, "%s", (loc->map[i][j]==-1)?"  X ":"    ");
      //sprintf(buf, "%3d ", loc->map[i][j]);
      Serial.print(buf);
    }
    Serial.println(" ");
  }
}

//inizializza la matrice della mappa. Imposta i limiti della matrice a -1 (ostacolo invalicabile)
void init_loc() {
  loc = malloc(sizeof(struct location));
  loc->orient=North;
  loc->x = SIZE / 2;
  loc->y = SIZE / 2;
  loc->map = (short **)calloc(SIZE, sizeof(short *));
  for (short i = 0; i < SIZE; i++) {
    loc->map[i] = (short *)calloc(SIZE, sizeof(short));
  }

  for (short i = 0; i < SIZE; i++) {
    for (short j = 0; j < SIZE; j++) {
      if (i == 0 || j == 0 || i == SIZE - 1 || j == SIZE - 1) loc->map[i][j] = -1;
    }
  }
}

// print (int) array (sul monitor seriale)
void printa(size_t size, int arr[]) {
  for (int i = 0; i < size; i++) {
    Serial.print(arr[i]);
    Serial.print(" ");
  }
  Serial.println("");
}

// print double array (sul monitor seriale)
void printa_d(size_t size, double arr[]) {
  for (int i = 0; i < size; i++) {
    Serial.print(arr[i]);
    Serial.print(" ");
  }
  Serial.println("");
}

// mostra sul monitor seriale la distanza misurata dal sensore ultrasonico in centimetri
void readDistance() {
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

// restituisce la distanza misurata dal sensore ultrasonico in centimetri
int readDistance_int() {

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
void scan() {
  for (pos = 0; pos <= 180; pos += 1) {  // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);  // tell servo to go to position in variable 'pos'
    readDistance();
    delay(15);  // waits 15ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) {  // goes from 180 degrees to 0 degrees
    myservo.write(pos);                  // tell servo to go to position in variable 'pos'
    readDistance();
    delay(15);  // waits 15ms for the servo to reach the position
  }
}

// scannerizza i 180 gradi d'ambiente verso il quale è rivolto il rover.
// Le misurazioni vengono restituite in centimetri.
// params:
//  steps: il numero di misurazioni da prendere
//  arr: puntatore all'array in cui verranno memorizzate le misurazioni (viene riallocato)
void scanSteps(int steps, int **arr) {
  *arr = (int *)calloc(steps, sizeof(int));

  for (int i = 0; i < steps; i++) {
    pos = 180 / steps * i;
    myservo.write(pos);
    int distance = readDistance_int();
    (*arr)[i] = distance;

    //char buf[50];
    //sprintf(buf, "a[%d] = %d (%d) (pos=%d)\n", i, (*arr)[i], distance, pos);
    //Serial.print(buf);

    delay(SCAN_STEP_TIME);
  }
}

void calc_radians(int size, double **arr) {
  *arr = (double *)calloc(size, sizeof(double));
  for (int i = 0; i < size; i++)
    (*arr)[i] = (180 / (size)*i) * 2 * 3.14159265358979323846 / 360;
}

void calc_angles(int size, double **arr) {
  *arr = (double *)calloc(size, sizeof(double));
  for (int i = 0; i < size; i++)
    (*arr)[i] = 180 / (size)*i;
}

// void updateMap(int size, int *r){
//   double *theta = NULL;
//   calc_radians(size, &theta);
//   printa_d(size, theta);

//   for(int i=0; i<size; i++){
//     double x = -(r[i]/UNIT)*cos(theta[i]) + loc->x;
//     double y = -(r[i]/UNIT)*sin(theta[i]) + loc->y;



//     printf("[%f] (%d) -> x=%lf y=%lf\n", theta[i], r[i], x, y);
//     if(x < SIZE && x >= 0 && y< SIZE && y >= 0){
//       loc->map[(int)y][(int)x] = -1;
//     }
//   }
// }

void updateMap(int size, int *r) {
  double *theta = NULL;
  calc_angles(size, &theta);

  for (int i = 0; i < size; i++) {
    int x, y;    

    if(loc -> orient == North){ 
      x = -(r[i] / UNIT) * cos(theta[i] * M_PI / 180) + loc->x;
      y = -(r[i] / UNIT) * sin(theta[i] * M_PI / 180) + loc->y;
    }else if(loc -> orient == South){ 
      x = (r[i] / UNIT) * cos(theta[i] * M_PI / 180) + loc->x;
      y = (r[i] / UNIT) * sin(theta[i] * M_PI / 180) + loc->y;
    }
    // int x = (r[i] / UNIT) * cos(theta[i] * M_PI / 180) + loc->x;
    // int y = (r[i] / UNIT) * sin(theta[i] * M_PI / 180) + loc->y;
    // loc->map[x][y] = -1; // west

    //printf("[%f] (%d) -> x=%d y=%d\n", theta[i], r[i], x, y);
    if (x < SIZE && x >= 0 && y < SIZE && y >= 0) {
      loc->map[y][x] = -1;
    }
  }
  free(theta);
}

// ferma le ruote
void stop() {
  digitalWrite(rf, LOW);
  digitalWrite(rb, LOW);
  digitalWrite(lf, LOW);
  digitalWrite(lb, LOW);
}

// muove la ruota destra avanti
void right() {
  digitalWrite(rf, HIGH);
}

// muove la ruota destra indietro
void bright() {
  digitalWrite(rb, HIGH);
}

// muove la ruota sinistra avanti
void left() {
  digitalWrite(lf, HIGH);
}

// muove la ruota sinistra indietro
void bleft() {
  digitalWrite(lb, HIGH);
}

// muove avanti
void forw() {
  right();
  left();
}

// muove indietro
void backw() {
  bright();
  bleft();
}

void rotate_counterclockwise() {
  right();
  bleft();
}

void rotate_clockwise() {
  left();
  bright();
}

// test per le ruote
void wheels_test() {
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
void wheels_test_hall() {
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

//test sensori di hall deprecato
void read_hall(int ms) {
  for (int i = 0; i < ms; i++) {
    if (i % 50 == 0) {
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
int read_hall_r() {
  return digitalRead(hall_pin_r);
}

// legge il valore del sensore di hall sinistro
// returns:
//   0 se non viene rilevato un campo magnetico
//   1 se viene rilevato
int read_hall_l() {
  return digitalRead(hall_pin_l);
}

// ruota la/e ruota/e clicks punti di appoggio (magneti).
// params:
//   clicks: il numero di punti d'appoggio da contare
//   fun_rot: la funzione di rotazione
//   fun_hall: la funzione di lettura del valore del sensore di hall
void rotate_times(int clicks, void (*fun_rot)(), int (*fun_hall)()) {
  int init_hall_state = fun_hall();
  int hall_state = 0;
  while (clicks--) {
    if (init_hall_state) {
      do {
        fun_rot();
        hall_state = fun_hall();
      } while (hall_state);
      do {
        fun_rot();
        hall_state = fun_hall();
      } while (!hall_state);
    } else {
      do {
        fun_rot();
        hall_state = fun_hall();
      } while (!hall_state);
    }
  }
  stop();
}

// Riposiziona le ruote sul primo punto di appoggio disponibile
void calibrate_hall() {
  rotate_times(1, right, read_hall_r);
  rotate_times(1, left, read_hall_l);
}

void setup() {
  pinMode(A3, INPUT);

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


  //Serial.begin(9600);
  Serial.begin(19200);
  //Serial.begin(31250);

  //inizializza e visualizza mappa
  init_loc();
  p_map();

  Serial.println("---begin---");
  //myservo.write(pos);  //riposiziona servo

  calibrate_hall();  //riposizioniamo le ruote





  delay(2000);  //per riposizionare la macchinina
}

void loop() {

  // int *arr = NULL;
  // scanSteps(MEASURE_NR, &arr);

  // printa(MEASURE_NR, arr);
  // updateMap(MEASURE_NR, arr);
  // free(arr);

  // //p_map();
  // rotate_times(7, rotate_counterclockwise, read_hall_l);
  // loc -> orient = South;

  // scanSteps(MEASURE_NR, &arr);  
  // printa(MEASURE_NR, arr);
  // updateMap(MEASURE_NR, arr);
  // free(arr);
  // p_map();
  
  // rotate_times(7, rotate_clockwise, read_hall_l);
  // loc->orient=North;

  // delay(1000);


  // rotate_times(7, rotate_counterclockwise, read_hall_l);
  // delay(5000);
  // rotate_times(7, rotate_clockwise, read_hall_l);
  // delay(5000);
  
  // rotate_times(4, forw, read_hall_r);
  // delay(5000);


  // left();
  // read_hall(2000);
  // stop();
  // delay(1000);





  // rotate_times(1, forw, read_hall_r); //21 cm
  // delay(1000);


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
