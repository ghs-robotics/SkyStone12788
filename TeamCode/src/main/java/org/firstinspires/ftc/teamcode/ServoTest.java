package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Servo Test")
public class ServoTest extends OpMode {
    private Servo servo;
//    private ElapsedTime timer;
//    // true  = waiting for 1
//    // false = waiting for 0
//    private boolean state = true;
    private boolean aWasPressed = false;
    private boolean bWasPressed = false;
    private double currentPosition = 0.5;


    @Override
    public void init() {
        servo = (Servo)hardwareMap.get("testServo");
    }

    // Grabber positions, determined by empirical testing:
    // Open:   0.05
    // Closed: 0.50
    @Override
    public void loop() {
        telemetry.addData("Servo Position", currentPosition);

        if (gamepad1.a && !aWasPressed) {
            currentPosition += 0.01;

            if (currentPosition >= 1.0)
                currentPosition = 1.0;
        }

        if (gamepad1.b && !bWasPressed) {
            currentPosition -= 0.01;

            if (currentPosition <= 0.0)
                currentPosition = 0.0;
        }


        bWasPressed = gamepad1.b;
        aWasPressed = gamepad1.a;

        servo.setPosition(currentPosition);


//        if (timer == null) {
//            servo.setPosition(0);
//            timer = new ElapsedTime();
//            timer.reset();
//        } else if (state && timer.seconds() >= 3) {
//            servo.setPosition(1);
//            state = false;
//            timer.reset();
//        } else if (!state && timer.seconds() >= 3) {
//            servo.setPosition(0);
//            state = true;
//            timer.reset();
//        }
    }
}
