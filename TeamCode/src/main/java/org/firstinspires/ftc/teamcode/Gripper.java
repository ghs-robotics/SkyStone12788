package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Gripper {
    private static final String SERVO_NAME = "gripperServo";
    private static final Double OPEN_POSITION   = 0.05;
    private static final Double CLOSED_POSITION = 0.50;
    private boolean state = false; // false = open, true = closed
    private Servo servo;
    private Gamepad gamepad;

    public Gripper(HardwareMap hwmap, Gamepad gamepad) {
        servo = hwmap.get(Servo.class, SERVO_NAME);
        this.gamepad = gamepad;
    }

    public void update() {
        if (gamepad.left_bumper) {
            state = false;
        }

        if (gamepad.right_bumper) {
            state = true;
        }

        servo.setPosition(state ? CLOSED_POSITION : OPEN_POSITION);
    }
}
