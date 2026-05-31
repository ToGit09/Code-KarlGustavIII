#include <Arduino.h>

// Architektur: 6 Inputs -> 8 Hidden -> 6 Hidden -> 5 Outputs
const int N_INPUTS   = 6;
const int N_H1       = 8;
const int N_H2       = 6;
const int N_OUTPUTS  = 5;

// Beispiel-Gewichte (Dummywerte, später durch Training ersetzen)
float W_input_h1[N_H1][N_INPUTS] = {
  {0.1, -0.2, 0.05, 0.3, -0.1, 0.2},
  {-0.3, 0.4, 0.1, -0.2, 0.05, 0.1},
  {0.2, 0.1, -0.3, 0.2, 0.1, -0.1},
  {0.05, -0.1, 0.2, 0.1, -0.2, 0.3},
  {0.3, 0.1, -0.2, 0.05, 0.4, -0.1},
  {-0.1, 0.05, 0.2, -0.3, 0.3, 0.2},
  {0.2, 0.2, 0.2, 0.2, 0.2, -0.2},
  {-0.2, -0.1, 0.3, 0.1, 0.05, 0.1}
};
float B_h1[N_H1] = {0.1, -0.05, 0.0, 0.1, -0.1, 0.05, 0.0, 0.1};

float W_h1_h2[N_H2][N_H1] = {
  {0.1, -0.2, 0.05, 0.3, -0.1, 0.2, 0.1, -0.2},
  {-0.3, 0.4, 0.1, -0.2, 0.05, 0.1, 0.2, -0.1},
  {0.2, 0.1, -0.3, 0.2, 0.1, -0.1, 0.3, 0.05},
  {0.05, -0.1, 0.2, 0.1, -0.2, 0.3, -0.1, 0.2},
  {0.3, 0.1, -0.2, 0.05, 0.4, -0.1, 0.2, 0.1},
  {-0.1, 0.05, 0.2, -0.3, 0.3, 0.2, -0.2, 0.1}
};
float B_h2[N_H2] = {0.05, -0.02, 0.0, 0.05, -0.05, 0.02};

float W_h2_out[N_OUTPUTS][N_H2] = {
  {0.2, -0.3, 0.1, 0.05, 0.2, -0.1},   // move_dir
  {0.1, 0.2, -0.1, 0.3, -0.2, 0.05},   // move_speed
  {-0.2, 0.1, 0.3, -0.1, 0.05, 0.2},   // orientation
  {0.05, -0.2, 0.1, 0.2, -0.1, 0.3},   // dribbler_dir
  {0.3, 0.1, -0.2, 0.05, 0.2, -0.1}    // kick
};
float B_out[N_OUTPUTS] = {0.05, 0.0, 0.1, -0.05, 0.0};

// Aktivierungsfunktionen
float relu(float x) { return (x > 0) ? x : 0; }
float sigmoid(float x) { return 1.0f / (1.0f + exp(-x)); }

// Forward-Pass
void forward(float input[N_INPUTS], float output[N_OUTPUTS]) {
  float h1[N_H1];
  float h2[N_H2];

  // Input -> Hidden1
  for (int h = 0; h < N_H1; h++) {
    float sum = B_h1[h];
    for (int i = 0; i < N_INPUTS; i++) {
      sum += W_input_h1[h][i] * input[i];
    }
    h1[h] = relu(sum);
  }

  // Hidden1 -> Hidden2
  for (int h = 0; h < N_H2; h++) {
    float sum = B_h2[h];
    for (int i = 0; i < N_H1; i++) {
      sum += W_h1_h2[h][i] * h1[i];
    }
    h2[h] = relu(sum);
  }

  // Hidden2 -> Output
  for (int o = 0; o < N_OUTPUTS; o++) {
    float sum = B_out[o];
    for (int h = 0; h < N_H2; h++) {
      sum += W_h2_out[o][h] * h2[h];
    }
    if (o == 4) { // Kicker-Ausgabe binär
      output[o] = sigmoid(sum);
    } else {
      output[o] = sum; // kontinuierliche Werte
    }
  }
}

void setup() {
  Serial.begin(115200);
}

void loop() {
  // Beispiel-Eingaben: [x, y, theta_goal, ball_dir, ball_dist, ball_present]
  float input[N_INPUTS] = {0.2, 0.5, -0.1, 0.3, 0.4, 1.0};
  float output[N_OUTPUTS];

  forward(input, output);

  Serial.print("Move Dir: "); Serial.print(output[0], 3);
  Serial.print(" | Speed: "); Serial.print(output[1], 3);
  Serial.print(" | Orientation: "); Serial.print(output[2], 3);
  Serial.print(" | Dribbler: "); Serial.print(output[3], 3);
  Serial.print(" | Kick: "); Serial.println(output[4], 3);

  delay(20); // ~50 Hz
}