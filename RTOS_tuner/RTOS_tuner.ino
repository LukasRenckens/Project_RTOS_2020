#include <Arduino_FreeRTOS.h>

//clipping indicator variables
boolean clipping = 0;

//data storage variables
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

//variables for decided whether you have a match
byte noMatch = 0;             //counts how many non-matches you've received to reset variables if it's been too long
byte slopeTol = 3;            //slope tolerance- adjust this if you need (default = 3)
int timerTol = 10;            //timer tolerance- adjust this if you need (default = 10)

//variables for amp detection
unsigned int ampTimer = 0;
byte maxAmp = 0;
byte checkMaxAmp;
byte ampThreshold = 8;       //raise if you have a very noisy signal (default = 30) (8 -> vpp = 350mV)

//Functions forward declaration
void reset(void);
void checkClipping();

// define two tasks for Blink & AnalogRead
void TaskBlink(void *pvParameters);
void TaskAnalogRead(void *pvParameters);
void TaskReadFrequency(void *pvParameters);

// the setup function runs once when you press reset or power the board
void setup() {

  // Now set up two tasks to run independently.
  //xTaskCreate(TaskBlink, "Blink", 128, NULL, 2, NULL );
  // A name just for humans, Stack size, priority

  //xTaskCreate(TaskAnalogRead, "AnalogRead", 128, NULL, 1, NULL );
  // This stack size can be checked & adjusted by reading Highwater
  // priority

  xTaskCreate(TaskReadFrequency, "ReadFrequency", 128, NULL, 1, NULL);  //Task, Name, Stack size, PvParameters, Priority, PuxStackBuffer, pxTaskBuffer 
   
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
  Serial.begin(9600);
  
  pinMode(13,OUTPUT);//led indicator pin
  pinMode(12,OUTPUT);//output pin
  
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
      
      //print results
      Serial.print(frequency);
      Serial.println(" hz");
    }
    vTaskDelay( 100 / portTICK_PERIOD_MS ); // wait for 100 ms
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
