#include <Arduino_FreeRTOS.h>
#include <math.h>

// Output pin connections
#define SEG_A A3
#define SEG_B A2
#define SEG_C 12
#define SEG_D 11
#define SEG_E 10
#define SEG_F A4
#define SEG_G A5
#define SEG_DP 13

#define NUMBER_OF_LEDS 3

const int correct_leds[2] = {2, 3};
const int low_leds[NUMBER_OF_LEDS] = {5, 7, 9};
const int high_leds[NUMBER_OF_LEDS] = {4, 6, 8};

// Clipping indicator variables
boolean clipping = 0;

// Data storage variables
byte newData = 0;
byte prevData = 0;
unsigned int time = 0;        //keeps time and sends vales to store in timer[] occasionally
int timer[10];                //storage for timing of events
int slope[10];                //storage for slope of events
unsigned int totalTimer;      //used to calculate period
unsigned int period;          //storage for period of wave
byte index = 0;               //current storage index
float frequency;              //storage for frequency calculations
int maxSlope = 0;             //used to calculate max slope as trigger point
int newSlope;                 //storage for incoming slope data

// Variables for decided whether you have a match
byte noMatch = 0;             //counts how many non-matches you've received to reset variables if it's been too long
byte slopeTol = 3;            //slope tolerance- adjust this if you need (default = 3)
int timerTol = 10;            //timer tolerance- adjust this if you need (default = 10)

// Variables for amp detection
unsigned int ampTimer = 0;
byte maxAmp = 0;
byte checkMaxAmp;
byte ampThreshold = 8;       //raise if you have a very noisy signal (default = 30) (8 -> vpp = 350mV)

// Variables for frequency to note conversion
float correctFrequency;
int octave;

// Functions forward declaration
void reset(void);
void checkClipping();

// Define two tasks for Blink & AnalogRead
// void TaskBlink(void *pvParameters);
// void TaskAnalogRead(void *pvParameters);
void TaskReadFrequency(void *pvParameters);
void TaskShowOutput(void *pvParameters);

// The setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(9600);

    // Outputs 7SEG
  pinMode(SEG_A, OUTPUT);
  pinMode(SEG_B, OUTPUT);
  pinMode(SEG_C, OUTPUT);
  pinMode(SEG_D, OUTPUT);
  pinMode(SEG_E, OUTPUT);
  pinMode(SEG_F, OUTPUT);
  pinMode(SEG_G, OUTPUT);
  pinMode(SEG_DP, OUTPUT);

  // Output LEDS
  pinMode(correct_leds[0], OUTPUT);
  pinMode(correct_leds[1], OUTPUT);
  for (int i = 0; i < NUMBER_OF_LEDS; i++) {
    pinMode(low_leds[i], OUTPUT);
    pinMode(high_leds[i], OUTPUT);
  }

  // Now set up two tasks to run independently.
  //xTaskCreate(TaskBlink, "Blink", 128, NULL, 2, NULL );
  // A name just for humans, Stack size, priority

  //xTaskCreate(TaskAnalogRead, "AnalogRead", 128, NULL, 1, NULL );
  // This stack size can be checked & adjusted by reading Highwater
  // priority

  xTaskCreate(TaskReadFrequency, "ReadFrequency", 128, NULL, 1, NULL);  //Task, Name, Stack size, PvParameters, Priority, PuxStackBuffer, pxTaskBuffer 
  xTaskCreate(TaskShowOutput, "ShowOutput", 128, NULL, 2, NULL);  //Task, Name, Stack size, PvParameters, Priority, PuxStackBuffer, pxTaskBuffer 

   
  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}

void loop()
{
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

//void TaskBlink(void *pvParameters)  // This is a task.
//{
//  (void) pvParameters;
//
//  // initialize digital pin 13 as an output.
//  pinMode(13, OUTPUT);
//
//  for (;;) // A Task shall never return or exit.
//  {
//    digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
//    vTaskDelay( 1000 / portTICK_PERIOD_MS ); // wait for one second
//    digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
//    vTaskDelay( 1000 / portTICK_PERIOD_MS ); // wait for one second
//  }
//}

//void TaskAnalogRead(void *pvParameters)  // This is a task.
//{
//  (void) pvParameters;
//
//  // initialize serial communication at 9600 bits per second:
//  Serial.begin(9600);
//
//  for (;;)
//  {
//    // read the input on analog pin 0:
//    int sensorValue = analogRead(A0);
//    // print out the value you read:
//    Serial.println(sensorValue);
//    vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
//  }
//}

void TaskReadFrequency(void *pvParameters){
  (void) pvParameters;
  // Serial.begin(9600); -> Is verplaatst naar setup
  
  // WORDEN GEBRUIKT VOOR 7SEG OP DIT MOMENT!!!
  // pinMode(13,OUTPUT);//led indicator pin
  // pinMode(12,OUTPUT);//output pin
  
  cli();//diable interrupts
  
  //set up continuous sampling of analog pin 0 at 38.5kHz
 
  //clear ADCSRA and ADCSRB registers
  ADCSRA = 0;
  ADCSRB = 0;
  
  ADMUX |= (1 << REFS0);                    //set reference voltage
  ADMUX |= (1 << ADLAR);                    //left align the ADC value- so we can read highest 8 bits from ADCH register only
  
  ADCSRA |= (1 << ADPS2) | (1 << ADPS0);    //set ADC clock with 32 prescaler- 16mHz/32=500kHz
  ADCSRA |= (1 << ADATE);                   //enabble auto trigger
  ADCSRA |= (1 << ADIE);                    //enable interrupts when measurement complete
  ADCSRA |= (1 << ADEN);                    //enable ADC
  ADCSRA |= (1 << ADSC);                    //start ADC measurements
  
  sei();                                    //enable interrupts

  for(;;){                                  //inf loop
    checkClipping();
  
    if (checkMaxAmp>ampThreshold){
      frequency = 38500/float(period);      //calculate frequency timer rate/period
      

      // print results
      Serial.print(frequency);
      Serial.println(" hz");
    }

    // NIET ZEKER OF DEZE DELAY NOODZAKELIJK IS??
    vTaskDelay(100/portTICK_PERIOD_MS); // wait for 100 ms
  }
}

void TaskShowOutput(void *pvParameters) {
  (void) pvParameters;

  for(;;) {
    checkNote(frequency);
    determineNumberOfLeds(frequency);

    // NIET ZEKER OF DEZE DELAY NOODZAKELIJK IS??
    vTaskDelay(100/portTICK_PERIOD_MS);
  }
}

/*--------------------------------------------------*/
/*-------------------- Functions -------------------*/
/*--------------------------------------------------*/
void reset(){//clean out some variables
  index = 0;                    //reset index
  noMatch = 0;                  //reset match couner
  maxSlope = 0;                 //reset slope
}

void checkClipping(){//manage clipping indication
  if (clipping){               //if currently clipping
    clipping = 0;
  }
}

// Displays a note on the 7SEG-display
void displayNote(char note, bool sharp) {
  switch (note) {
    case 'A':
      digitalWrite(SEG_A, HIGH);
      digitalWrite(SEG_B, HIGH);
      digitalWrite(SEG_C, HIGH);
      digitalWrite(SEG_D, LOW);
      digitalWrite(SEG_E, HIGH);
      digitalWrite(SEG_F, HIGH);
      digitalWrite(SEG_G, HIGH);
      sharp ? digitalWrite(SEG_DP, HIGH) : digitalWrite(SEG_DP, LOW);
      break;
    case 'B':
      digitalWrite(SEG_A, HIGH);
      digitalWrite(SEG_B, HIGH);
      digitalWrite(SEG_C, HIGH);
      digitalWrite(SEG_D, HIGH);
      digitalWrite(SEG_E, HIGH);
      digitalWrite(SEG_F, HIGH);
      digitalWrite(SEG_G, HIGH);
      digitalWrite(SEG_DP, LOW);    // B# doesn't exist
      break;
    case 'C':
      digitalWrite(SEG_A, HIGH);
      digitalWrite(SEG_B, LOW);
      digitalWrite(SEG_C, LOW);
      digitalWrite(SEG_D, HIGH);
      digitalWrite(SEG_E, HIGH);
      digitalWrite(SEG_F, HIGH);
      digitalWrite(SEG_G, LOW);
      sharp ? digitalWrite(SEG_DP, HIGH) : digitalWrite(SEG_DP, LOW);
      break;
    case 'D':
      digitalWrite(SEG_A, HIGH);
      digitalWrite(SEG_B, HIGH);
      digitalWrite(SEG_C, HIGH);
      digitalWrite(SEG_D, HIGH);
      digitalWrite(SEG_E, HIGH);
      digitalWrite(SEG_F, HIGH);
      digitalWrite(SEG_G, LOW);
      sharp ? digitalWrite(SEG_DP, HIGH) : digitalWrite(SEG_DP, LOW);
      break;
    case 'E':
      digitalWrite(SEG_A, HIGH);
      digitalWrite(SEG_B, LOW);
      digitalWrite(SEG_C, LOW);
      digitalWrite(SEG_D, HIGH);
      digitalWrite(SEG_E, HIGH);
      digitalWrite(SEG_F, HIGH);
      digitalWrite(SEG_G, HIGH);
      digitalWrite(SEG_DP, LOW);    // E# doesn't exist
      break;
    case 'F':
      digitalWrite(SEG_A, HIGH);
      digitalWrite(SEG_B, LOW);
      digitalWrite(SEG_C, LOW);
      digitalWrite(SEG_D, LOW);
      digitalWrite(SEG_E, HIGH);
      digitalWrite(SEG_F, HIGH);
      digitalWrite(SEG_G, HIGH);
      sharp ? digitalWrite(SEG_DP, HIGH) : digitalWrite(SEG_DP, LOW);
      break;
    case 'G':
      digitalWrite(SEG_A, HIGH);
      digitalWrite(SEG_B, LOW);
      digitalWrite(SEG_C, HIGH);
      digitalWrite(SEG_D, HIGH);
      digitalWrite(SEG_E, HIGH);
      digitalWrite(SEG_F, HIGH);
      digitalWrite(SEG_G, LOW);
      sharp ? digitalWrite(SEG_DP, HIGH) : digitalWrite(SEG_DP, LOW);
      break;
    default:
      digitalWrite(SEG_A, LOW);
      digitalWrite(SEG_B, LOW);
      digitalWrite(SEG_C, LOW);
      digitalWrite(SEG_D, LOW);
      digitalWrite(SEG_E, LOW);
      digitalWrite(SEG_F, LOW);
      digitalWrite(SEG_G, LOW);
      digitalWrite(SEG_DP, LOW);
      break;
  }
}

// Puts the green LEDs on or off
void powerCorrectLeds(bool on) {
  if (on) {
    digitalWrite(correct_leds[0], HIGH);
    digitalWrite(correct_leds[1], HIGH);
  } else {
    digitalWrite(correct_leds[0], LOW);
    digitalWrite(correct_leds[1], LOW);
  }
}

// Toggle a number of leds on
void setLeds(int leds[NUMBER_OF_LEDS], int number) {
  for (int i = 0; i < NUMBER_OF_LEDS; i++) {
    digitalWrite(leds[i], LOW);
  }

  for (int j = 0; j < number; j++) {
    digitalWrite(leds[j], HIGH);
  }
}

void determineNumberOfLeds(float frequency) {
  float error = 0.125*pow(2,octave);

  if (frequency < correctFrequency - error) {
    float previous_tone = correctFrequency * pow(2,-1.0/12);
    float halfway_previous_tone = calculateHalfway(previous_tone, correctFrequency - error);

    float one_third = halfway_previous_tone + 2 * (correctFrequency - error - halfway_previous_tone)/3;
    float two_thirds = halfway_previous_tone + (correctFrequency - error - halfway_previous_tone)/3;

    powerCorrectLeds(false);
    setLeds(high_leds, 0);
    if (frequency < two_thirds) {
      setLeds(low_leds, 3);
    } else if (frequency < one_third) {
      setLeds(low_leds, 2);
    } else {
      setLeds(low_leds, 1);
    }
    // Debugging
    // Serial.print(frequency);
    // Serial.print(",");
    // Serial.print(correctFrequency);
    // Serial.print(",");
    // Serial.print(one_third);
    // Serial.print(",");
    // Serial.print(two_thirds);
    // Serial.print(",");
    // Serial.println(halfway_previous_tone);
  } else if (frequency > correctFrequency + error) {
    float next_tone = correctFrequency * pow(2,1.0/12);
    float halfway_next_tone = calculateHalfway(correctFrequency + error, next_tone);

    float one_third = halfway_next_tone - 2 * (halfway_next_tone - (correctFrequency + error))/3;
    float two_thirds = halfway_next_tone - (halfway_next_tone - (correctFrequency + error))/3;

    powerCorrectLeds(false);
    setLeds(low_leds, 0);
    if (frequency > two_thirds) {
      setLeds(high_leds, 3);
    } else if (frequency > one_third) {
      setLeds(high_leds, 2);
    } else {
      setLeds(high_leds, 1);
    }
    // Debugging
    // Serial.print(frequency);
    // Serial.print(",");
    // Serial.print(correctFrequency);
    // Serial.print(",");
    // Serial.print(one_third);
    // Serial.print(",");
    // Serial.print(two_thirds);
    // Serial.print(",");
    // Serial.println(halfway_next_tone);
  } else {
    powerCorrectLeds(true);
    setLeds(low_leds, 0);
    setLeds(high_leds, 0);
  }
}

//Determine the correct frequency and display the note on 7SEG
void checkNote(float frequency){
  octave = floor((log(frequency/16.35)/log(2)));

  // float lower_b_note =  30.87*pow(2,octave-1);
  float c_note =        16.35*pow(2,octave);
  float c_sharp_note =  17.32*pow(2,octave);
  float d_note =        18.35*pow(2,octave);
  float d_sharp_note =  19.45*pow(2,octave);
  float e_note =        20.60*pow(2,octave);
  float f_note =        21.83*pow(2,octave);
  float f_sharp_note =  23.12*pow(2,octave);
  float g_note =        24.50*pow(2,octave);
  float g_sharp_note =  25.96*pow(2,octave);
  float a_note =        27.50*pow(2,octave);
  float a_sharp_note =  29.14*pow(2,octave);
  float b_note =        30.87*pow(2,octave);
  float higher_c_note = 16.35*pow(2,octave+1);
  
  if (frequency > calculateHalfway(c_note, c_sharp_note)) {
    if (frequency > calculateHalfway(c_sharp_note, d_note)) {
      if (frequency > calculateHalfway(d_note, d_sharp_note)) {
        if (frequency > calculateHalfway(d_sharp_note, e_note)) {
          if (frequency > calculateHalfway(e_note, f_note)) {
            if (frequency > calculateHalfway(f_note, f_sharp_note)) {
              if (frequency > calculateHalfway(f_sharp_note, g_note)) {
                if (frequency > calculateHalfway(g_note, g_sharp_note)) {
                  if (frequency > calculateHalfway(g_sharp_note,a_note)) {
                    if (frequency > calculateHalfway(a_note, a_sharp_note)) {
                      if (frequency > calculateHalfway(a_sharp_note, b_note)) {
                        if (frequency > calculateHalfway(b_note, higher_c_note)) {
                          // C
                          displayNote('C', false);
                          correctFrequency = higher_c_note;
                        } else {
                          // B
                          displayNote('B', false);
                          correctFrequency = b_note;
                        }
                      } else {
                        // A#
                        displayNote('A', true);
                        correctFrequency = a_sharp_note;
                      }
                    } else {
                      // A
                      displayNote('A', false);
                      correctFrequency = a_note;
                    }
                  } else {
                    // G#
                    displayNote('G', true);
                    correctFrequency = g_sharp_note;
                  }
                } else {
                  // G
                  displayNote('G', false);
                  correctFrequency = g_note;
                }
              } else {
                // F#
                displayNote('F', true);
                correctFrequency = f_sharp_note;
              }
            } else {
              // F
              displayNote('F', false);
              correctFrequency = f_note;
            }
          } else {
            // E
            displayNote('E', false);
            correctFrequency = e_note;
          }
        } else {
          // D#
          displayNote('D', true);
          correctFrequency = d_sharp_note;
        }
      } else {
        // D
        displayNote('D', false);
        correctFrequency = d_note;
      }
    } else {
      // C#
      displayNote('C', true);
      correctFrequency = c_sharp_note;
    }
  } else {
    // C
    displayNote('C', false);
    correctFrequency = c_note;
  }
}

float calculateHalfway(float low, float high) {
  float halfway = low + (high - low)/2;
  return halfway;
}
/*--------------------------------------------------*/
/*----------------------- ISR ----------------------*/
/*--------------------------------------------------*/
ISR(ADC_vect) {                                 //when new ADC value ready
  PORTB &= B11101111;                           //set pin 12 low
  prevData = newData;                           //store previous value
  newData = ADCH;                               //get value from A0
  if (prevData < 127 && newData >=127){         //if increasing and crossing midpoint
    newSlope = newData - prevData;              //calculate slope
    if (abs(newSlope-maxSlope)<slopeTol){       //if slopes are ==
      //record new data and reset time
      slope[index] = newSlope;
      timer[index] = time;
      time = 0;
      if (index == 0){                          //new max slope just reset
        PORTB |= B00010000;                     //set pin 12 high
        noMatch = 0;
        index++;                                //increment index
      }
      else if (abs(timer[0]-timer[index])<timerTol && abs(slope[0]-newSlope)<slopeTol){//if timer duration and slopes match
        //sum timer values
        totalTimer = 0;
        for (byte i=0; i<index; i++){
          totalTimer+=timer[i];
        }
        period = totalTimer;                    //set period
        //reset new zero index values to compare with
        timer[0] = timer[index];
        slope[0] = slope[index];
        index = 1;                              //set index to 1
        PORTB |= B00010000;                     //set pin 12 high
        noMatch = 0;
      }
      else{                                     //crossing midpoint but not match
        index++;                                //increment index
        if (index > 9){
          reset();
        }
      }
    }
    else if (newSlope>maxSlope){                //if new slope is much larger than max slope
      maxSlope = newSlope;
      time = 0;                                 //reset clock
      noMatch = 0;
      index = 0;                                //reset index
    }
    else{//slope not steep enough
      noMatch++;//increment no match counter
      if (noMatch>9){
        reset();
      }
    }
  }
    
  if (newData == 0 || newData == 1023){          //if clipping
    clipping = 1;                                //currently clipping
    Serial.println("clipping");
  }
  
  time++;//increment timer at rate of 38.5kHz (ADC inputbandwwidth = 38.5KHz)
  
  ampTimer++;//increment amplitude timer
  if (abs(127-ADCH)>maxAmp){
    maxAmp = abs(127-ADCH);
  }
  if (ampTimer==1000){
    ampTimer = 0;
    checkMaxAmp = maxAmp;
    maxAmp = 0;
  }

}
