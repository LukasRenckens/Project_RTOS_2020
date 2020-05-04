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
  displayNote('A', false);
  toggleNumberOfLeds(low_leds, 3);
  delay(500);
  displayNote('A', true);
  toggleNumberOfLeds(low_leds, 2);
  delay(500);
  displayNote('B', false);
  toggleNumberOfLeds(low_leds, 1);
  delay(500);
  displayNote('C', false);
  toggleNumberOfLeds(low_leds, 0);
  toggleCorrectLeds(true);
  delay(500);
  displayNote('C', true);
  toggleCorrectLeds(false);
  toggleNumberOfLeds(high_leds, 1);
  delay(500);
  displayNote('D', false);
  toggleNumberOfLeds(high_leds, 2);
  delay(500);
  displayNote('D', true);
  toggleNumberOfLeds(high_leds, 3);
  delay(500);
  displayNote('E', false);
  toggleNumberOfLeds(high_leds, 2);
  delay(500);
  displayNote('F', false);
  toggleNumberOfLeds(high_leds, 1);
  delay(500);
  displayNote('F', true);
  toggleNumberOfLeds(high_leds, 0);
  toggleCorrectLeds(true);
  delay(500);
  displayNote('G', false);
  toggleCorrectLeds(false);
  toggleNumberOfLeds(low_leds, 1);
  delay(500);
  displayNote('G', true);
  toggleNumberOfLeds(low_leds, 2);
  delay(500);
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
void toggleCorrectLeds(bool on) {
  if (on) {
    digitalWrite(correct_leds[0], HIGH);
    digitalWrite(correct_leds[1], HIGH);
  } else {
    digitalWrite(correct_leds[0], LOW);
    digitalWrite(correct_leds[1], LOW);
  }
}

// 
void toggleNumberOfLeds(int leds[NUMBER_OF_LEDS], int number) {
  for (int i = 0; i < NUMBER_OF_LEDS; i++) {
    digitalWrite(leds[i], LOW);
  }

  for (int j = 0; j < number; j++) {
    digitalWrite(leds[j], HIGH);
  }
}
