#include <Arduino_FreeRTOS.h>
#include <AudioFrequencyMeter.h>

// define two tasks for Blink & AnalogRead
void TaskBlink(void *pvParameters);
void TaskAnalogRead(void *pvParameters);
void TaskReadFrequency(void *pvParameters);

AudioFrequencyMeter meter;

// the setup function runs once when you press reset or power the board
void setup() {

  // Now set up two tasks to run independently.
  xTaskCreate(TaskBlink, "Blink", 128, NULL, 2, NULL );
  // A name just for humans, Stack size, priority

  //xTaskCreate(TaskAnalogRead, "AnalogRead", 128, NULL, 1, NULL );
  // This stack size can be checked & adjusted by reading Highwater
  // priority

  xTaskCreate(TaskReadFrequency, "ReadFrequency", 128, NULL, 1, NULL);
   
  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}

void loop()
{
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskBlink(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  // initialize digital pin 13 as an output.
  pinMode(13, OUTPUT);

  for (;;) // A Task shall never return or exit.
  {
    digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
    vTaskDelay( 1000 / portTICK_PERIOD_MS ); // wait for one second
    digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
    vTaskDelay( 1000 / portTICK_PERIOD_MS ); // wait for one second
  }
}

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
  
  Serial.begin(115200);
  meter.setBandwidth(70.00, 1500);    // Ignore frequency out of this range
  meter.begin(A0, 45000);             // Intialize A0 at sample rate of 45kHz

  for(;;){
    // put your main code here, to run repeatedly:
    float frequency = meter.getFrequency();
    if (frequency > 0)
    {
      Serial.print(frequency);
      Serial.println(" Hz");
    }
  }
}
