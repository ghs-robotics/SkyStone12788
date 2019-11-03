package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Servo Test")
public class ServoTest extends OpMode {
    private Servo servo;
    private boolean leftBumperWasPressed = false;
    private boolean rightBumperWasPressed = false;
    private double currentPosition = 0.5;


    @Override
    public void init() {
        servo = (Servo)hardwareMap.get("wristServo");
    }

    // Grabber positions, determined by empirical testing:
    // Open:   0.05
    // Closed: 0.50
    @Override
    public void loop() {
        telemetry.addData("Servo Position", currentPosition);

        if (gamepad1.left_bumper && !leftBumperWasPressed) {
            currentPosition += 0.05;

            if (currentPosition >= 1.0)
                currentPosition = 1.0;
        }

        if (gamepad1.right_bumper && !rightBumperWasPressed) {
            currentPosition -= 0.05;

            if (currentPosition <= 0.0)
                currentPosition = 0.0;
        }


        rightBumperWasPressed = gamepad1.right_bumper;
        leftBumperWasPressed = gamepad1.left_bumper;

        servo.setPosition(currentPosition);
    }
}
