#include <Servo.h>

#define ENABLE_RADAR
#define ENABLE_MOTORS

#define BROKEN_RIGHT_MOTOR

// servo
#define SERVO_PIN 5

// sensore ultrasonico
#define echo_pin 2
#define trig_pin 3

// sensori hall effect
#define hall_pin_r A0
#define hall_pin_l A1

// sensori encoder ottico
#define photo_pin_r A4
#define photo_pin_l A3

// variabili per l'encoder ottico
int max_photo_r = 0;  
int min_photo_r = 0;

int max_photo_l = 0;
int min_photo_l = 0;

// controlli per il ponte h
#define rf 10  // right forward
#define rb 9   // right backwards
#define lf 11  // left forwards
#define lb 12  // left backwards

// mappa
#define SIZE 21             // dimensione dei lati della mappa 
#define UNIT 10             //  la dimensione in centimetri di una cella della matrice della mappa. es: UNIT = 10 -> cella = 10 cm; SIZE = 21 -> dimensione della mappa = 21 * 10 cm

// radar
#define MEASURE_NR 20       // numero di misurazioni che deve prendere il radar  
#define SCAN_STEP_TIME 400  // tempo d'attesa minimo tra una misurazione e l'altra

int arr[MEASURE_NR];        // array d'appoggio su cui inserire le misurazioni prese con il radar

// variabili per il servo
Servo myservo;
int pos = 0;    //posizione corrente del servo


// orientamenti validi per il rover. "North" è da interpretare come l'orientamento d'avvio del rover, non come nord magnetico.
enum orientation { North,
                   South,
                   East,
                   West };
// struttura contenente le informazioni sulla posizione del rover e la mappa.
struct location {
  enum orientation orient;  //orientamento del rover
  int x;                    // x corrente
  int y;                    // y corrente
  short **map;              //matrice che rappresenta la mappa dell'ambiente circostante. Assume valore -1 dove ci sono ostacoli, 0 altrimenti.
};
struct location *loc;

// stampa la mappa (sul monitor seriale).
// stampa "x" in presenza di ostacoli e "O" nella posizione corrente del rover
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

//inizializza la matrice della mappa. Imposta i limiti della matrice a -1.
void init_loc() {
  loc = (location*) malloc(sizeof(struct location));
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
  long duration;  // variabile della durata di ritorno dell'impulso sonoro
  int distance;   // variable per contenere la misurazione della distanza calcolata 
  digitalWrite(trig_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);
  duration = pulseIn(echo_pin, HIGH);
  distance = duration * 0.034 / 2;
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
}

// restituisce la distanza misurata dal sensore ultrasonico in centimetri
int readDistance_int() {
  long duration;  // variabile della durata di ritorno dell'impulso sonoro
  int distance;   // variable per contenere la misurazione della distanza calcolata
  digitalWrite(trig_pin, LOW);
  delayMicroseconds(2);

  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);

  duration = pulseIn(echo_pin, HIGH);

  distance = duration * 0.034 / 2;
  return distance;
}

// scan deprecato
void scan() {
  for (pos = 0; pos <= 180; pos += 1) {  
    myservo.write(pos);  
    readDistance();
    delay(15);  // waits 15ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) {  
    myservo.write(pos);                  
    readDistance();
    delay(15);  // waits 15ms for the servo to reach the position
  }
}

// scannerizza i 180 gradi d'ambiente verso il quale è rivolto il rover.
// Le misurazioni vengono restituite in centimetri.
// params:
//  steps: il numero di misurazioni da prendere
//  arr: puntatore all'array in cui verranno memorizzate le misurazioni (viene riallocato)
void scanSteps(int steps, int arr[MEASURE_NR]) {
  for (int i = 0; i < steps; i++) {
    pos = 180 / steps * i;
    myservo.write(pos);
    int distance = readDistance_int();
    arr[i] = distance;

    // char buf[20];
    // sprintf(buf, "a[%d] = %d (%d) (pos=%d)\n", i, arr[i], distance, pos);
    // Serial.print(buf);

    delay(SCAN_STEP_TIME);
  }
}

// funzione che popola l'array theta con angoli divisi equamente su 180°.
// es. calc_angles(10, theta) -> theta = [0, 18, 36, 54, 72, 90, 108, 126, 144, 162]
void calc_angles(int size, double theta[]) {
  for (int i = 0; i < size; i++)
    theta[i] = 180 / (size)*i;
}

// aggiorna la mappa con le misurazioni contenute nell'array r.
// params:  size: la dimensione dell'array r
//          r: l'array contenente le nuove misurazioni
// modifies: loc->map
void updateMap(int size, int r[]) {
  double theta[size];
  calc_angles(size, theta);

  for (int i = 0; i < size; i++) {
    int x, y;    

    if(loc -> orient == North){ 
      x = -(r[i] / UNIT) * cos(theta[i] * M_PI / 180) + loc->x;
      y = -(r[i] / UNIT) * sin(theta[i] * M_PI / 180) + loc->y;
    }else if(loc -> orient == South){ 
      x = (r[i] / UNIT) * cos(theta[i] * M_PI / 180) + loc->x;
      y = (r[i] / UNIT) * sin(theta[i] * M_PI / 180) + loc->y;
    }

    //printf("[%f] (%d) -> x=%d y=%d\n", theta[i], r[i], x, y);
    if (x < SIZE && x >= 0 && y < SIZE && y >= 0) {
      loc->map[y][x] = -1;
    }
  }
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

// ruota in senso antiorario
void rotate_counterclockwise() {
  right();
  bleft();
}

// ruota in senso orario
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

// come rotate_times, però con doppia precisione. 
// prende in considerazione sia i punti in cui è presente il magnete che non.
// params:
//   clicks: il numero di punti d'appoggio da contare
//   fun_rot: la funzione di rotazione
//   fun_hall: la funzione di lettura del valore del sensore di hall
void rotate_times_2x(int clicks, void (*fun_rot)(), int (*fun_hall)()) {
  int hall_state = fun_hall();
  int hall_measurement = 0;
  while (clicks--) {
    do {
      fun_rot();
      hall_measurement = fun_hall();
    } while(hall_state == hall_measurement);
  }
  stop();
}

// gira il rover di 45° in senso orario
void clock45(){
  rotate_times(3, rotate_clockwise, read_hall_r);
  stop();
}

// gira il rover di 45° in senso antiorario
void counter45(){
  rotate_times(3, rotate_counterclockwise, read_hall_r);
  stop();
}

// riposiziona le ruote sul primo punto di appoggio (magnete) disponibile
void calibrate_hall() {

  #ifndef BROKEN_RIGHT_MOTOR
    rotate_times(1, right, read_hall_r);
  #endif
  
  rotate_times(1, left, read_hall_l);
}

// ottiene i valori correnti di luminosità massima/minima per entrambi gli encoder ottici. Attenzione, ruota le ruote in avanti per almeno val_num * 50 ms.
//  params: val_num: il numero di valori di campionamento della luce da prendere per la calibrazione
//  modifies: min_photo_l, max_photo_l, min_photo_r, max_photo_r
void calibrate_photo(int val_num){
  int maxl = 0;
  int minl = 2000;
  int maxr = 0;
  int minr = 2000;
  int current_val = 0;

  forw();

  for(int i=0; i<val_num; i++){
    current_val = analogRead(photo_pin_l);
    if(current_val > maxl) maxl=current_val;
    if(current_val < minl) minl=current_val;
    current_val = analogRead(photo_pin_r);
    if(current_val > maxr) maxr=current_val;
    if(current_val < minr) minr=current_val;
    delay(50);
  }
  stop();
  min_photo_l = minl;
  max_photo_l = maxl;
  min_photo_r = minr;
  max_photo_r = maxr;

  char buf[20];
  sprintf(buf, "r=[%d,%d,%d] l=[%d,%d,%d]", minr, maxr, (minr+maxr)/2, minl, maxl, (minl+maxl)/2);
  Serial.println(buf); 
}

// riposiziona gli encoder ottici delle ruote
void calibrate_optical(){
  rotate_times_photo_l(1, left);

  #ifndef BROKEN_RIGHT_MOTOR
  rotate_times_photo_r(1, right);
  #endif
}

// effettua clicks passi utilizzando la funzione di rotazione fun_rot, utilizzando l'encoder ottico sinistro
// utilizziamo come threshold del rilevamento di una luce la media tra max_photo_l e min_photo_l.
void rotate_times_photo_l(int clicks, void (*fun_rot)()) {
  int init_hall_state = (analogRead(photo_pin_l)>(max_photo_l+min_photo_l)/2)?1:0;
  int hall_state = init_hall_state;
  while (clicks--) {
    if (init_hall_state) {
      while(hall_state){
        fun_rot();
        hall_state = (analogRead(photo_pin_l)>(max_photo_l+min_photo_l)/2)?1:0; 
      }
      while(!hall_state){
        fun_rot();
        hall_state = (analogRead(photo_pin_l)>(max_photo_l+min_photo_l)/2)?1:0;
      }
    } else {
      while(!hall_state){
        fun_rot();
        hall_state = (analogRead(photo_pin_l)>(max_photo_l+min_photo_l)/2)?1:0;
      }
    }
  }
  stop();
}

// effettua clicks passi utilizzando la funzione di rotazione fun_rot, utilizzando l'encoder ottico destro
// utilizziamo come threshold del rilevamento di una luce la media tra max_photo_r e min_photo_r.
void rotate_times_photo_r(int clicks, void (*fun_rot)()) {
  int init_hall_state = (analogRead(photo_pin_r)>(max_photo_r+min_photo_r)/2)?1:0;
  int hall_state = 0;
  while (clicks--) {
    if (init_hall_state) {
      do {
        fun_rot();
        hall_state = (analogRead(photo_pin_r)>(max_photo_r+min_photo_r)/2)?1:0;
      } while (hall_state);
      do {
        fun_rot();
        hall_state = (analogRead(photo_pin_r)>(max_photo_r+min_photo_r)/2)?1:0;
      } while (!hall_state);
    } else {
      do {
        fun_rot();
        hall_state = (analogRead(photo_pin_r)>(max_photo_r+min_photo_r)/2)?1:0;
      } while (!hall_state);
    }
  }
  stop();
}

// effettua la scansione dell'area circostante, utilizzando gli encoder ottici per posizionare le ruote
//  requires: orientamento del rover a Nord
void read360_optical(){
  scanSteps(MEASURE_NR, arr);

  printa(MEASURE_NR, arr);
  updateMap(MEASURE_NR, arr);

  #ifndef BROKEN_RIGHT_MOTOR
  rotate_times_photo_l(17, rotate_counterclockwise); //+ 180°
  #else
  rotate_times_photo_l(43, rotate_counterclockwise); //+ 180°
  #endif
  loc -> orient = South;

  scanSteps(MEASURE_NR, arr);  
  printa(MEASURE_NR, arr);
  updateMap(MEASURE_NR, arr);
  p_map();


  #ifndef BROKEN_RIGHT_MOTOR
  rotate_times_photo_l(17, rotate_clockwise); //- 180°
  #else
  rotate_times_photo_l(43, rotate_clockwise); //- 180°
  #endif
  

  loc->orient=North;
}

// effettua la scansione dell'area circostante, utilizzando i sensori di hall per posizionare le ruote (il sensore sinistro)
//  requires: orientamento del rover a Nord
void read360_hall(){
  scanSteps(MEASURE_NR, arr);

  printa(MEASURE_NR, arr);
  updateMap(MEASURE_NR, arr);

  rotate_times(7, rotate_counterclockwise, read_hall_l); //+ 180°
  loc -> orient = South;

  scanSteps(MEASURE_NR, arr);  
  printa(MEASURE_NR, arr);
  updateMap(MEASURE_NR, arr);
  p_map();
  
  rotate_times(7, rotate_clockwise, read_hall_l); //- 180°
  loc->orient=North;
}

void setup() {

  #ifdef ENABLE_RADAR
  myservo.attach(SERVO_PIN);
  pinMode(trig_pin, OUTPUT);
  pinMode(echo_pin, INPUT);
  #endif
  #ifdef ENABLE_MOTORS
  pinMode(rf, OUTPUT);
  pinMode(rb, OUTPUT);
  pinMode(lf, OUTPUT);
  pinMode(lb, OUTPUT);
  #endif

  pinMode(hall_pin_r, INPUT);
  pinMode(hall_pin_l, INPUT);

  pinMode(photo_pin_l, INPUT);
  pinMode(photo_pin_r, INPUT);

  Serial.begin(19200);

  //inizializza e visualizza mappa
  init_loc();
  p_map();

  myservo.write(pos);     //riposiziona servo

  calibrate_photo(20);    //otteniamo valori di luce massimi e minimi dell'ambiente circostante
  calibrate_hall();       //riposizioniamo le ruote
  calibrate_optical();

  Serial.println("---begin---");
  delay(2000);            //per riposizionare il rover
}

int entrypoint = 1;    // scelta del nodo iniziale della macchina a stati finiti
int current_state = entrypoint;   // lo stato corrente della macchina a stati finiti

//definizione della macchina a stati finiti:
// stato 1 (entrypoint): scan dell'ambiente utilizzando gli encoder ottici
// stato 2 (entrypoint): scan dell'ambiente utilizzando i sensori ad effetto hall
// stato 3: esplorazione dell'ambiente

// 1 -> 3 -> 1 -> ...
// 2 -> 3 -> 2 -> ...

void loop() {
  Serial.println(current_state);
  // ------
  switch(current_state){
    case 1:                       //stato 1 (entrypoint): scan dell'ambiente utilizzando gli encoder ottici
      read360_optical();
      current_state = 3;          //  1 -> 3
      break;

    case 2:                       //stato 2 (entrypoint): scan dell'ambiente utilizzando i sensori ad effetto hall
      read360_hall();
      current_state = 3;          //  2 -> 3
      break;

    case 3:                       //stato 3: esplorazione ambientale
      //esplorazione ambientale
      current_state = entrypoint; //  ritorna allo stato iniziale
      break;

    default:
      Serial.println("errore macchina a stati finiti.");
  }
  
  delay(1000);

  // scanSteps(MEASURE_NR, arr);

  // printa(MEASURE_NR, arr);
  // updateMap(MEASURE_NR, arr);

  // //rotate_times(7, rotate_counterclockwise, read_hall_l);
  // rotate_times_photo_l(15, rotate_counterclockwise); //+ 45°
  // loc -> orient = South;

  // scanSteps(MEASURE_NR, arr);  
  // printa(MEASURE_NR, arr);
  // updateMap(MEASURE_NR, arr);
  // p_map();
  
  // //rotate_times(7, rotate_clockwise, read_hall_l);
  // rotate_times_photo_l(15, rotate_clockwise); //+ 45°
  // loc->orient=North;


  //------

  // test per rotazione di 45°
  // clock45();
  // delay(2000);
  // counter45();
  // delay(2000);

  // test di rotazione di 180°
  // rotate_times(7, rotate_counterclockwise, read_hall_l);
  // delay(5000);
  // rotate_times(7, rotate_clockwise, read_hall_l);
  // delay(5000);
  
  // test per sensori di hall
  // forw();
  // read_hall(2000);
  // stop();
  // delay(1000);

  // test rotazione con sensori di hall
  // rotate_times(1, forw, read_hall_r); //21 cm
  // delay(1000);

  //test per il servo
  // myservo.write(0);
  // delay(400);
  // myservo.write(90);
  // delay(400);
  // myservo.write(170);
  // delay(400);

  //test per l'encoder ottico
  // left();
  // Serial.println(analogRead(photo_pin_l));
  // delay(50);

  // test di rotazione di 45° utilizzando l'encoder ottico
  // rotate_times_photo_l(8, rotate_clockwise); //+ 45°
  // delay(1000);
  // rotate_times_photo_l(8, rotate_counterclockwise); //- 45°
  // delay(1000);
}