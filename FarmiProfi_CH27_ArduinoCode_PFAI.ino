/* Version without Serial */
#include "SimpleFOC.h"

/* PIN IN/OUT */
const int forwardButton = 10;
const int backwardButton = 11;
const int forwardValvePin = 8;
const int backwardValvePin = 9;
const int SelectorPin = 7;
const int toggleOutputPin = 12;

/* SIGNAL FILTERING */
const unsigned long RISING_EDGE_FILTER_TIME = 5000; // numero di microsecondi per il filtraggio 
const float T_F = 0.05;

/* SPEED PARAMETERS */
const float VOLTAGE_TRESHOLD = 2.0;
const float REDUCTION_PERCENTAGE = 0.05;


// SPEED-SPECIFIC CONFIGURATION
const float TARGET_SPEED_970 = 970.0;
const float MIN_SPEED_LOWER_SPEED_970 = 890.0; // 91%


const float TARGET_SPEED_LOWER_PERCENTAGE = 98.0; // percentuale della velcità che è sufficiente raggiungere dopo no stress
const unsigned long NO_STRESS_BACKWARD_TIME = 700; // tempo




unsigned long lastTimeRisingEdge = 0;
unsigned long currentTimeRisingEdge = 0;
unsigned long pulseInterval;

float convertedInterval;

volatile unsigned long difference;
volatile int counter = 0;
unsigned long RPM = 0;
unsigned long filteredRPM = 0;

int i;
int firstIndex = 0;
int lastIndex = 0;

bool flag = false;
bool gotRisingEdgeCandidate = false;
bool gotRisingEdge = false;
bool transition = true; // essere andati sotto 2.5V


float inputVoltage;
float previousVoltage;

LowPassFilter filter = LowPassFilter(T_F); // Tf = 1ms
unsigned long signal_filtered;

unsigned long saveTime;
unsigned long currentTime;

/* INPUT BUTTON */
int forwardButtonReading = 0;
int backwardButtonReading = 0;

int forwardButtonState = 0;
int backwardButtonState = 0;

int forwardButtonStateLast = 0;
int backwardButtonStateLast = 0;

long lastDebounceTimeFw = 0;  // the last time the output pin was toggled
long lastDebounceTimeBw = 0;  // the last time the output pin was toggled
long debounceDelay = 50;    // the debounce time; increase if the output flickers

int state = 0;

int forwardRequest = 0;
int backwardRequest = 0;


/* E-VALVE VARIABLES */
int valveState = 0;

// NO STRESS
int noStressState = 0;
int blockedMemory = 0;
int noStressDone = 0;

unsigned long noStressActivation = 0;

// SELETTORE LATERALE SCATOLA


int SelectorReading = 0;
int SelectorState = 0;
int SelectorStateLast = 0;
long lastDebounceTime_Selector = 0;  // the last time the output pin was toggled



float selected_speed = 0.0; // RPM
float min_speed_limit = 0.0;


/* DIGITAL OUTPUT toggle per attivare oscilloscopio */
bool toggleOutputState = false;


void setup() {

  pinMode(forwardButton, INPUT);
  pinMode(backwardButton, INPUT);

  pinMode(forwardValvePin, OUTPUT);
  pinMode(backwardValvePin, OUTPUT);

  pinMode(SelectorPin, INPUT);
  pinMode(toggleOutputPin, OUTPUT);

  // No Stress funziona solo a 970
  selected_speed = TARGET_SPEED_970;
  min_speed_limit = MIN_SPEED_LOWER_SPEED_970;
}
void loop() {
  
  int analog_value = analogRead(A0);  
  
  inputVoltage = (analog_value * 5.0) / 1024.0; 
  
  currentTime = micros();

  /*
    Sensore attivo basso.
    inputVoltage == LOW  --> lettura dell'induttore
    inputVoltage == HIGH --> vuoto

    Appena la tensione del sensore va bassa, significa che ho un falling_edge = passaggio dell'induttore.
    Considero questo come un candidato falling_edge. Salvo il tempo.
    (flag per evitare di rientrare, altrimenti ri-salverei il tempo sempre durante la lettura dell'induttore)

    candidato --> faccio un anti-debounce. Leggo tensione bassa, ok, ma voglio che lo sia per un tot di tempo.
    Se non sta bassa abbastanza, allora non è un buon candidato.

    Se dopo il tempo di anti-debounce, la tenzione è ancora bassa, allora confermo falling edge.
    Qui devo salvare il tempo che avevo registrato prima.

    Appena la tensione torna alta, allora abilito di nuovo la detection dell'edge transition = true.
    transition = il rotore sta ruotando, non leggo induttore.


  */
  
  if (inputVoltage < VOLTAGE_TRESHOLD && !gotRisingEdgeCandidate && !gotRisingEdge && transition){ // tutte condizioni che servono per abilitare la misurazione del tempo ogni volta che vedo falling.
    transition = false;
    saveTime = micros();
    gotRisingEdgeCandidate = true;
  }
  
  if (gotRisingEdgeCandidate && !gotRisingEdge){
    if ((currentTime - saveTime) > RISING_EDGE_FILTER_TIME){
      if (inputVoltage < VOLTAGE_TRESHOLD ){
        // se sono ancora su, allora confermo rising edge
        gotRisingEdge = true;
        gotRisingEdgeCandidate = false;
        lastTimeRisingEdge = currentTimeRisingEdge;
        //currentTimeRisingEdge = micros();
        currentTimeRisingEdge = saveTime;
      }
      else{
        gotRisingEdgeCandidate = false;
        gotRisingEdge = false;
      }
      
    }
  }

  if (inputVoltage >= VOLTAGE_TRESHOLD){
    transition = true;
  }

  
  /*
    Se ho notifica di rising edge correttamente detectato, allora calcolo l'intervallo fra i due rising edge consecutivi.
    convertedInterval è un float, per fare i conti.
  */
  if (gotRisingEdge){
    pulseInterval = currentTimeRisingEdge - lastTimeRisingEdge;
    convertedInterval = pulseInterval;

    // saturazione: per evitare ovf del dato float, devo saturare alla velocità massima.
    // anzi, per evitare di prendere dei rising edge frequenti, aumento il tempo di filtraggio per pulire di più il segnale in ingresso.
    RPM = (1000000 * 60) / convertedInterval;

    signal_filtered = filter(RPM); // filtraggio dei giri con la funzioncina.

    gotRisingEdge = false;
  
  }

  // BUTTON MANAGEMENT
  forwardButtonReading = digitalRead(forwardButton);

   // If the switch changed, due to noise or pressing:
  if (forwardButtonReading != forwardButtonStateLast) {
    // reset the debouncing timer
    lastDebounceTimeFw = millis();
  }

  if ((millis() - lastDebounceTimeFw) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (forwardButtonReading != forwardButtonState) {
      forwardButtonState = forwardButtonReading;

      // only toggle the LED if the new button state is HIGH
      if (forwardButtonState == HIGH) {
        ;
      }
    }
  }

  forwardButtonStateLast = forwardButtonReading;



  backwardButtonReading = digitalRead(backwardButton);

   // If the switch changed, due to noise or pressing:
  if (backwardButtonReading != backwardButtonStateLast) {
    // reset the debouncing timer
    lastDebounceTimeBw = millis();
  }

  if ((millis() - lastDebounceTimeBw) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (backwardButtonReading != backwardButtonState) {
      backwardButtonState = backwardButtonReading;

      // only toggle the LED if the new button state is HIGH
      if (backwardButtonState == HIGH) {
        ;       
      }
    }
  }
  
  backwardButtonStateLast = backwardButtonReading;

  switch (state) {
  case 0: // IDLE STATE tutti i relè a zero
    if (forwardButtonState) {
      state = 1;
    }
    if (backwardButtonState) {
      state = 2;
    }
    forwardRequest = 0;
    backwardRequest = 0;
    break;
  
  case 1: // FORWARDING
    if (!forwardButtonState) { // se ripremo forward o premo backward, entrambe le uscite vanno a zero. Fermo tutto in IDLE
      state = 0;
    }
    // CHAIN FORWARDING REQUEST
    forwardRequest = 1;
    break;

  case 2: // BACKWARDING
    if (!backwardButtonState) { // se ripremo backward o premo forward, entrambe le uscite vanno a zero. Fermo tutto in IDLE
      state = 0;
    }
    // CHAIN BACKWARDING REQUEST
    backwardRequest = 1;
    break;
  default:
    state = 0;
    break;
}

  // NO STRESS
  /*
    implementare antidebounce sulla velocità filtrata per evitare delle letture spurie.
    Vari metodi:
    1) migliorare filtraggio
    2) antidebounce
    3) se ci sono letture troppo discordanti dal valore attuale/da una breve media, allora caccia via tutto
    4) antidebounce tipo che devi avere un tot di letture di velcoità sotto un certo valore per accettare il fatto che la velocità si sta effettivamente riducendo.
  */
  switch (noStressState) {
  case 0: // IDLE check della velocità
    if (signal_filtered <= min_speed_limit){
      forwardRequest = 0;
      backwardRequest = 1; // avvio la richiesta di andare indietro
      noStressActivation = millis();
      noStressState = 1;
      noStressDone = 0;
    }
    break;
  
  case 1: // mandiamo indietro il materiale per 1500ms per liberare l'alimentazione
    forwardRequest = 0;
    if(!noStressDone){
      backwardRequest = 1;
    }    

    if ((millis() - noStressActivation) > NO_STRESS_BACKWARD_TIME && !noStressDone){
      backwardRequest = 0;
      noStressDone = 1;
      noStressState = 2;
    }
    break;

  case 2:// aspettiamo che la velocità torni al target (con un po' di margine)
    forwardRequest = 0; // mentre recupero la velocità non voglio mandare dentro altro materiale...salva in casi di materiale molto grosso
    float vel_ref = selected_speed * TARGET_SPEED_LOWER_PERCENTAGE / 100.0;
    if (signal_filtered > vel_ref){
      noStressState = 0;
      noStressDone = 0;
    }
    // potrei aggiungere un controllo di tempo: se impiego troppo tempo a ripristinare la velocità, allora fai contromisure.
    break;

  default:
    noStressState = 0;
    break;
  }
  

  // E-VALVE MANAGEMENT
  if (forwardRequest){
    digitalWrite(forwardValvePin, LOW);
  }
  else{
    digitalWrite(forwardValvePin, HIGH);
  }  
  
  if (backwardRequest){
    digitalWrite(backwardValvePin, LOW);
  }
  else{
    digitalWrite(backwardValvePin, HIGH);
  }


  // BUTTON MANAGEMENT selettore laterale della scatola
  SelectorReading = digitalRead(SelectorPin);

   // If the switch changed, due to noise or pressing:
  if (SelectorReading != SelectorStateLast) {
    // reset the debouncing timer
    lastDebounceTime_Selector = millis();
  }

  if ((millis() - lastDebounceTime_Selector) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (SelectorReading != SelectorState) {
      SelectorState = SelectorReading;

    }
  }

  SelectorStateLast = SelectorReading;

  // only toggle the LED if the new button state is HIGH
  if (SelectorState == HIGH) { // fronte positivo, allora avvio toggling di un digitalOutput
    toggleOutputState = !toggleOutputState; // il primo fronte di salita scatena acquisizione dell'oscilloscopio 
  }
  else{
    toggleOutputState = LOW;        
  }

  digitalWrite(toggleOutputPin, toggleOutputState);
}
