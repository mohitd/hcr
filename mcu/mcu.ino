// Copyright (c) 2020. Mohit Deshpande.
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//  
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

// HARDWARE SETUP
//  
// encoder 1 schematic
// C1: Encoder Phase A (trigger)  -> Pin 2
// C2: Encoder Phase B            -> Pin 12

// encoder 2 schematic
// C1: Encoder Phase A (trigger)  -> Pin 3
// C2: Encoder Phase B            -> Pin 11

// **** GLOBAL VARS **** //

// encoders (L4 + R4), bump (L1 + C1 + R1), ToF (L2 + C2 + R2)
unsigned char sensor_packet[17];
volatile long encoders[2] = {0L, 0L};

volatile long pid_encoders[2] = {0L, 0L};

unsigned long last_command_stamp = 0;

// **** CONSTANTS **** //
// motor pins
static const byte MOTOR_L_PWM = 5;
static const byte MOTOR_L_DIR = 4;
static const byte MOTOR_R_PWM = 6;
static const byte MOTOR_R_DIR = 7;

// encoder pins
static const byte ENCODER_L_A = 2;
static const byte ENCODER_L_B = 12;
static const byte ENCODER_R_A = 3;
static const byte ENCODER_R_B = 11;

// bump switch pins
static const byte BUMP_R = 8;
static const byte BUMP_C = 9;
static const byte BUMP_L = 10;

// TOF pins
static const auto TOF_R = A5;
static const auto TOF_C = A4;
static const auto TOF_L = A6;

// floating-point epsilon
static const float FP_EPS = 0.01;

// PID gains
static const float kp = 1.0;
static const float kd = 0.0;
static const float ki = 0.0;

// PWM bounds
static const byte PWM_MIN = 120;
static const byte PWM_MAX = 255;

// robot configuration
static const float WHEEL_BASE = 0.275f;
static const float WHEEL_RADIUS = 0.065f;
static const float WHEEL_CIRCUMFERENCE = 2 * 3.1415f * WHEEL_RADIUS;

static const int COUNTS_PER_REV_L = 1475;
static const int COUNTS_PER_REV_R = 1446;

static const float DISTANCE_PER_COUNT_L = WHEEL_CIRCUMFERENCE / COUNTS_PER_REV_L;
static const float DISTANCE_PER_COUNT_R = WHEEL_CIRCUMFERENCE / COUNTS_PER_REV_R;
static const float DISTANCE_PER_COUNT   = 0.5 * (DISTANCE_PER_COUNT_L + DISTANCE_PER_COUNT_R);

// velocity bounds
static const float V_MAX = 1.0;
static const float V_MIN = -1.0;
static const float V_EPS = 0.10;

static const float W_MAX = 3.14;
static const float W_MIN = -3.14;
static const float W_EPS = 0.10;

// manual override
static const int NUM_BUTTONS = 5;
static const int ADC_KEY_VALUES[5] = {30, 150, 360, 535, 760};
int button_press = 0;

// **** FUNCTIONS **** //
void init_encoders();
void encoder_l_interrupt();
void encoder_r_interrupt();

void init_bump();

void init_tof();

void init_drivetrain();
void drivetrain_command(float v, float w);

void button_override();
int get_button(unsigned int input);

float mapf(float value, float a1, float a2, float b1, float b2);

void setup() 
{
    init_drivetrain();
    init_encoders();
    init_bump();
    init_tof();
    
    Serial.begin(19200);
    Serial.println("Initialized MCU");
}
 
void loop()
{
    memset(&sensor_packet, 0, sizeof(sensor_packet));
    static unsigned char cmd_vel_buffer[8];
    static float cmd_vel[2];

    // **** RECEIVE COMMAND VELOCITIES **** //
    if (Serial.available() > 0)
    {
        last_command_stamp = millis();
        while (Serial.read() != 255);
        size_t num_read = Serial.readBytes(cmd_vel_buffer, sizeof(cmd_vel_buffer));
        if (num_read == sizeof(cmd_vel_buffer))
        {
            memcpy(cmd_vel, &cmd_vel_buffer, sizeof(cmd_vel_buffer));
            drivetrain_command(cmd_vel[0], cmd_vel[1]);
        }
        // else the packet was corrupted
    }
    
    // **** MANUAL MCU OVERRIDE **** //
    button_override();

    // **** SAFETY TIMEOUT **** //
    if (millis() - last_command_stamp > 100)
    {
        drivetrain_command(0., 0.);
    }

    // **** READ SENSORS **** //
    unsigned char bl = digitalRead(BUMP_L);
    unsigned char bc = digitalRead(BUMP_C);
    unsigned char br = digitalRead(BUMP_R);
    memcpy(&sensor_packet[8], bl, sizeof(bl));
    memcpy(&sensor_packet[9], bc, sizeof(bc));
    memcpy(&sensor_packet[10], br, sizeof(br));

    unsigned short tof_l = analogRead(TOF_L);
    unsigned short tof_c = analogRead(TOF_C);
    unsigned short tof_r = analogRead(TOF_R);
    memcpy(&sensor_packet[11], tof_l, sizeof(tof_l));
    memcpy(&sensor_packet[13], tof_c, sizeof(tof_c));
    memcpy(&sensor_packet[15], tof_r, sizeof(tof_r));

    // **** READ WHEEL ENCODERS **** //
    memcpy(&sensor_packet[0], &encoders, sizeof(encoders));

    // **** SEND SENSOR PACKET**** //
    Serial.write(254);
    Serial.write(sensor_packet, sizeof(sensor_packet));
    encoders[0] = 0;
    encoders[1] = 0;

    delay(50);
}

void init_drivetrain()
{
    pinMode(MOTOR_L_DIR, OUTPUT);
    pinMode(MOTOR_R_DIR, OUTPUT);
    digitalWrite(MOTOR_L_DIR, LOW);
    digitalWrite(MOTOR_R_DIR, LOW);
}

// drive a commanded linear and angular velocity
void drivetrain_command(float v, float w)
{
    static unsigned long pid_time = millis();
    // fast reaction for immediately cutting off power
    if (abs(v) < V_EPS && abs(w) < W_EPS)
    {
        Serial.print("Sending (0->0, 0->0)");
        analogWrite(MOTOR_L_PWM, 0);
        analogWrite(MOTOR_R_PWM, 0);
        pid_time = millis();
        return;
    }
    
    // small (v,w) -> 0 and rest get constrained to [MIN, MAX]
    v = abs(v) < V_EPS ? 0.0f : max(V_MIN, min(v, V_MAX));
    w = abs(w) < W_EPS ? 0.0f : max(W_MIN, min(w, W_MAX));

    // right-hand rule: a positive w => turning left, robot-relative
    float w_l = ((2 * v) - (w * WHEEL_BASE)) / (2*WHEEL_RADIUS);
    float w_r = ((2 * v) + (w * WHEEL_BASE)) / (2*WHEEL_RADIUS);


    // **** PID Control **** //
    float pid_dt = (millis() - pid_time) / 1000.0f;
    pid_time = millis();

    float feedback_l = 2 * 3.1415 * pid_encoders[0] / COUNTS_PER_REV_L / pid_dt;
    float feedback_r = 2 * 3.1415 * pid_encoders[1] / COUNTS_PER_REV_R / pid_dt;

    // dead region to account for noisy encoders
    //if (abs(pid_encoders[0]) < 0.001 * COUNTS_PER_REV_L) feedback_l = 0.0f;
    //if (abs(pid_encoders[1]) < 0.001 * COUNTS_PER_REV_R) feedback_r = 0.0f;
    
    pid_encoders[0] = 0;
    pid_encoders[1] = 0;

    // proportion
    float error_l = w_l - feedback_l;
    float error_r = w_r - feedback_r;

    // integral
    static float ierror_l = 0;
    static float ierror_r = 0;
    ierror_l += error_l * pid_dt;
    ierror_r += error_r * pid_dt;

    // derivative
    static float prev_error_l = 0;
    static float prev_error_r = 0;
    float derror_l = (error_l - prev_error_l) / pid_dt;
    float derror_r = (error_r - prev_error_r) / pid_dt;
    prev_error_l = error_l;
    prev_error_r = error_r;

    // PID correction
    //w_l = kp * error_l + ki * ierror_l + kd * derror_l;
    //w_r = kp * error_r + ki * ierror_l + kd * derror_r;

    // direction of motors
    digitalWrite(MOTOR_L_DIR, w_l > 0 ? LOW : HIGH);
    digitalWrite(MOTOR_R_DIR, w_r > 0 ? LOW : HIGH);

    // map velocities to positive PWM
    //int pwm_l = abs(w_l) < FP_EPS ? 0 : (int) mapf(abs(w_l), 0, 15.38, PWM_MIN, PWM_MAX);
    //int pwm_r = abs(w_r) < FP_EPS ? 0 : (int) mapf(abs(w_r), 0, 15.38, PWM_MIN, PWM_MAX);
    int pwm_l = (int) mapf(abs(w_l), 0, 15.38, PWM_MIN, PWM_MAX);
    int pwm_r = (int) mapf(abs(w_r), 0, 15.38, PWM_MIN, PWM_MAX);
    
    // special case for 0
    //pwm_l = pwm_l == 0 ? 0 : constrain(pwm_l, PWM_MIN, PWM_MAX);
    //pwm_r = pwm_r == 0 ? 0 : constrain(pwm_r, PWM_MIN, PWM_MAX);
    
    Serial.print("Sending (");
    Serial.print(w_l); Serial.print("->"); Serial.print(pwm_l);
    Serial.print(", ");
    Serial.print(w_r); Serial.print("->"); Serial.print(pwm_r);
    Serial.println(")"); 

    analogWrite(MOTOR_L_PWM, pwm_l);
    analogWrite(MOTOR_R_PWM, pwm_r);
}

void init_encoders()
{
    pinMode(ENCODER_L_B, INPUT);
    pinMode(ENCODER_R_B, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENCODER_L_A), encoder_l_interrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_R_A), encoder_r_interrupt, CHANGE);
}

void encoder_l_interrupt()
{
    static byte prev_state = digitalRead(ENCODER_L_A);
    static boolean forward = true;
    int state = digitalRead(ENCODER_L_A);
    if (prev_state == LOW && state == HIGH)
    {
        int val = digitalRead(ENCODER_L_B);
        if (val == LOW && forward)
        {
            forward = false;
        }
        else if (val == HIGH && !forward)
        {
            forward = true;
        }
    }
    prev_state = state;

    if (!forward)
    {
        ++encoders[0];
        ++pid_encoders[0];
    }
    else
    {
        --encoders[0];
        --pid_encoders[0];
    }
}

void encoder_r_interrupt()
{
    static byte prev_state = digitalRead(ENCODER_R_A);
    static boolean forward = true;
    int state = digitalRead(ENCODER_R_A);
    if (prev_state == LOW && state == HIGH)
    {
        int val = digitalRead(ENCODER_R_B);
        if (val == LOW && forward)
        {
            forward = false;
        }
        else if (val == HIGH && !forward)
        {
            forward = true;
        }
    }
    prev_state = state;

    // invert direction for the right wheel
    if (!forward)
    {
        --encoders[1];
        --pid_encoders[1];
    }
    else
    {
        ++encoders[1];
        ++pid_encoders[1];
    }
}

void init_bump()
{
    pinMode(BUMP_R, INPUT);
    pinMode(BUMP_C, INPUT);
    pinMode(BUMP_L, INPUT);
}

void init_tof()
{
    pinMode(TOF_R, INPUT);
    pinMode(TOF_C, INPUT);
    pinMode(TOF_L, INPUT);
}

void button_override()
{
    int adc_button = analogRead(7);
    button_press = get_button(adc_button);
    if (button_press >= 0)
    {
        last_command_stamp = millis();
        switch (button_press)
        {
        case 0:
            // up button
            drivetrain_command(0.5, 0.0);
            break;
        case 1:
            // left button
            drivetrain_command(0.0, 0.785);
            break;
        case 2:
            // down button
            drivetrain_command(-0.5, 0.0);
            break;
        case 3:
            // right button
            drivetrain_command(0.0, -0.785);
            break;
        default:
            break;
        }
    }
}

int get_button(unsigned int input)
{
    for (int i = 0; i < NUM_BUTTONS; ++i)
    {
        if (input < ADC_KEY_VALUES[i])
        {
            return i;
        }
    }
    return -1;
}

float mapf(float value, float a1, float a2, float b1, float b2)
{
    return b1 + ((value - a1) * (b2 - b1))/(a2 - a1);
}
