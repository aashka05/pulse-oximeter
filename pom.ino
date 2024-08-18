/*
  Project - PulseOximeter

  Parts required:
  - JHD 16x2 LCD
  - 10 kilohm resistor
  - 10 kilohm potentiometer
  - Max30100 oximeter module

*/

//-----------------------------------------------------------------------------
// include the library code:
#include <LiquidCrystal.h>
#include "MAX30100_PulseOximeter.h"

#include "lpf.h"

//-----------------------------------------------------------------------------
// defines
//-----------------------------------------------------------------------------

#define REPORTING_PERIOD_MS     (1000)
#define ledPin                  (13)

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
// initialize the library with the numbers of the interface pins
LiquidCrystal mylcd_object(12, 11, 3, 4, 5, 6);
PulseOximeter pox;
LPF lpf(10);

uint32_t tsLastReport = 0;
int timer1_counter;

//-----------------------------------------------------------------------------
// POM beat detection callback function
//-----------------------------------------------------------------------------
void onBeatDetected()
{
  //  Serial.println("Beat!");
  // toggle D13 / L1 on beat detected
  digitalWrite(ledPin, digitalRead(ledPin) ^ 1);
}

//-----------------------------------------------------------------------------
// Timer Intterupt service routine
//-----------------------------------------------------------------------------
ISR(TIMER1_OVF_vect)// interrupt service routine
{
  sei();
  TCNT1 = timer1_counter; // preload timer
  
  // Enable this to profile timer interrupt duration measurement
  // digitalWrite(ledPin, digitalRead(ledPin) ^ 1);
  
  // Make sure to call update at uniform interval through timer interrupt
  // at >> 20 msec
  pox.update();
}

//-----------------------------------------------------------------------------
// setup
//-----------------------------------------------------------------------------
void setup() {
  // set pin D13 as output
  pinMode(ledPin, OUTPUT);
  // set up the number of columns and rows on the LCD
  mylcd_object.begin(16, 2);

  // set serial monitor baudrate to 115200
  Serial.begin(115200);
  Serial.print("Initializing pulse oximeter..");

  // Initialize the PulseOximeter instance
  // Failures are generally due to an improper I2C wiring, missing power supply
  // or wrong target chip
  if (!pox.begin()) {
    Serial.println("FAILED");
    for (;;);
  }
  else {
    Serial.println("SUCCESS");
  }

  // set init curreent level
  // it is dynamically updated based on feedback of received signal strength 
  // inside pox.update
  pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);

  // Register a callback for the beat detection
  pox.setOnBeatDetectedCallback(onBeatDetected);

  // initialize timer1
  noInterrupts();// disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;

  //timer1_counter = 34286; // preload timer 65536-16MHz/256/2Hz
  timer1_counter = 64500; // 65400 preload timer 65536-16MHz/256/2Hz

  TCNT1  =  timer1_counter;  // preload timer
  TCCR1B  |=  (1  <<  CS12); // 256 prescaler
  TIMSK1  |=  (1  <<  TOIE1);// enable timer overflow interrupt
  interrupts();             // enable all interrupts
}

//-----------------------------------------------------------------------------
// loop
//-----------------------------------------------------------------------------
void loop() {
  if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
    int l_spo2 = pox.getSpO2();
    int l_bpm  = pox.getHeartRate();
    Serial.print("Heart rate:");
    Serial.print(l_bpm);
    Serial.print("bpm / SpO2:");
    Serial.print(l_spo2);
    Serial.println("%");

    // Error conditions
    if(!l_spo2) {
      // if spo2 is zero then clear bpm
      l_bpm = 0;
    } else {
      // sometime spo2 is received more than 100
      // saturate it to 100 in that case
      if(l_spo2 > 100) {
        l_spo2 = 100;
      }
    }

    // apply low pass filter
    lpf.AddNewSample(l_bpm);
    l_bpm = lpf.Filter();
    
    mylcd_object.clear();
    mylcd_object.setCursor(0, 0);
    mylcd_object.print("SPO2 : ");
    mylcd_object.print(l_spo2);
    mylcd_object.print("%");
    mylcd_object.setCursor(0, 1);
    mylcd_object.print("Pulse: ");
    mylcd_object.print(l_bpm);

    tsLastReport = millis();
  } 
}

	

 //-----------------------------------------------------------------------------
// defines
//-----------------------------------------------------------------------------

#define MAX_FILTER_SIZE (32)

//-----------------------------------------------------------------------------
// class defination
//-----------------------------------------------------------------------------
class LPF {
  private:
    int data_array_ptr[MAX_FILTER_SIZE];
    int filter_length;
    int count_after_reset;
    int wr_index;

  public:
    LPF(int a_length);
    void AddNewSample(int a_new_sample);
    int Filter(void);
};

//-----------------------------------------------------------------------------
LPF::LPF(int a_length)
//-----------------------------------------------------------------------------
{
  if (a_length >= MAX_FILTER_SIZE) {
    Serial.println("Too long filter - failed.");
    filter_length = 1;
  } else {
    filter_length = a_length;
    wr_index = 0;
  }
}

//-----------------------------------------------------------------------------
void LPF::AddNewSample(int a_new_sample) {
//-----------------------------------------------------------------------------
  if (a_new_sample < 10) {
    // if < 10 is received clear and reset filter
    count_after_reset = 0;
    wr_index = 0;
  } else {
    // write new sample in buffer
    data_array_ptr[wr_index] = a_new_sample;

    // increament index position
    wr_index++;
    // if index is reached to end of array wrap-around
    if (wr_index >= filter_length) {
      wr_index = 0;
    }

    // increament and saturate count_after_reset to length of filter
    count_after_reset++;
    if (count_after_reset >= filter_length) {
      count_after_reset = filter_length;
    }

  }
}

//-----------------------------------------------------------------------------
int LPF::Filter(void)
//-----------------------------------------------------------------------------
{
  int i;

  if (count_after_reset) {
    int sum = 0;

    // add all valid samples
    for (i = 0; i < count_after_reset; i++) {
      sum += data_array_ptr[i];
    }

    // take moving average
    sum = sum / count_after_reset;
    
    return sum;
    
  } else {
    // return zero if no valid sample available
    return 0;
  }
}
