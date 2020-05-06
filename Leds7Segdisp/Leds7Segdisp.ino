#include <math.h>

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

float frequency;
float correctFrequency;
int octave;

String readString = "";

void setup() {
  Serial.begin(9600); // sets serial port for communication

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

}

void loop() {
  // Test code
  while (Serial.available()) {
    if (Serial.available() >0) {
      char c = Serial.read();  //gets one byte from serial buffer
      readString += c; //makes the string readString
    }

    if (!Serial.available()) {
      delay(1);
    }
  }

  if (readString.length() > 0) {
    frequency = readString.toFloat();
    readString = "";
  }

  checkNote(frequency);
  determineNumberOfLeds(frequency);

  delay(100);
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
    Serial.print(frequency);
    Serial.print(",");
    Serial.print(correctFrequency);
    Serial.print(",");
    Serial.print(one_third);
    Serial.print(",");
    Serial.print(two_thirds);
    Serial.print(",");
    Serial.println(halfway_previous_tone);
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
    Serial.print(frequency);
    Serial.print(",");
    Serial.print(correctFrequency);
    Serial.print(",");
    Serial.print(one_third);
    Serial.print(",");
    Serial.print(two_thirds);
    Serial.print(",");
    Serial.println(halfway_next_tone);
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
