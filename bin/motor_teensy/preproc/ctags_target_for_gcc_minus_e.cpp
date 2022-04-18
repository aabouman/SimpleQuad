# 1 "/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/motor_teensy/motor_teensy.ino"
// #define DEBUG







# 10 "/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/motor_teensy/motor_teensy.ino" 2
# 11 "/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/motor_teensy/motor_teensy.ino" 2
# 12 "/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/motor_teensy/motor_teensy.ino" 2
# 13 "/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/motor_teensy/motor_teensy.ino" 2

// Setting up the filter
Filter::EKF ekf = Filter::EKF(1.0, 0.1);
Filter::state_t<float> filt_state(0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
Filter::input_t<float> filt_input(0, 0, 0, 0, 0, 0);
Filter::measurement_t<float> filt_meas(0, 0, 0, 1, 0, 0, 0);

// Setting up the controller
Control::state_t<float> cont_state(0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0);
Control::input_t<float> cont_input(0, 0, 0, 0);

// Setting up the motor command structure
Control::motors_t motors;
int last_command_time = micros();

// Startup
void setup()
{
    pinMode(13, 1);
# 42 "/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/motor_teensy/motor_teensy.ino"
    // Setting the initial state of the quadrotor
    ekf.set_state(filt_state);

    // Setup Motor ESC
    { if (Serial) { Serial.println("Initializing motors"); } };
    Control::initialize_motors(&motors, (2), (3),
                               (4), (5));
    // Start up comunications with feather
    initFeatherPacket();
    while (getFeatherPacket(&filt_input, &filt_meas) <= 0);
}

void loop()
{
    float dt = getFeatherPacket(&filt_input, &filt_meas);

    if (dt > 0.0)
    {
        // Run mEKF
        ekf.prediction(filt_input, dt);
        ekf.update(filt_meas);

        // Compute inputs from th econtroller state
        Filter::state_t<float> tmp_state = ekf.get_state();
        filt_to_cont(tmp_state, filt_input, &cont_state);
        cont_input = Control::get_control(cont_state);
        cont_input = Control::clamp_control(cont_input);
# 96 "/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/motor_teensy/motor_teensy.ino"
        // Command motors
        Control::command_motors(&motors, cont_input);
        last_command_time = micros();
    }

    // Kill if haven't received a message in awhile
    float time_since_last_command = (micros() - last_command_time) / 1E6;
    if (time_since_last_command > 0.1)
    {
        kill(&motors);
    }
}
