/* Version without Serial */
#include "SimpleFOC.h"

/* PIN IN/OUT */
const int toggleOutputPin = 13;

/* SIGNAL FILTERING */
const unsigned long RISING_EDGE_FILTER_TIME = 5000; // numero di microsecondi per il filtraggio 
const float T_F = 0.05;

/* SPEED PARAMETERS */
const float VOLTAGE_TRESHOLD = 2.0;
const float REDUCTION_PERCENTAGE = 0.05;





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


/* DIGITAL OUTPUT toggle per attivare oscilloscopio */
bool toggleOutputState = false;


void setup() {
  pinMode(toggleOutputPin, OUTPUT);

  Serial.begin(9600);
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

  Serial.print("Voltage: ");
  Serial.print(inputVoltage);
  Serial.print(" V ");
  Serial.print("   ");
  Serial.print("Speed: ")
  Serial.print(RPM);
  Serial.println(" rpm");
  

  digitalWrite(toggleOutputPin, toggleOutputState);
  toggleOutputState = !toggleOutputState;
}
