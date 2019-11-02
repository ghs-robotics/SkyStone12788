package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Arm Test")
public class ArmTest extends OpMode {
    private ArmControl arm;
    private boolean aWasPressed = false;
    private boolean bWasPressed = false;
    private double currentPosition = 0.5;

    @Override
    public void init() {
        arm = new ArmControl(hardwareMap, telemetry, gamepad1, true);
    }

    // Arm positions, determined by empirical testing:
    //
    //
    @Override
    public void loop() {
        telemetry.addData("Arm Position", currentPosition);

        if (gamepad1.a && !aWasPressed) {
            currentPosition += 0.05;

            if (currentPosition >= 1.0)
                currentPosition = 1.0;
        }

        if (gamepad1.b && !bWasPressed) {
            currentPosition -= 0.05;

            if (currentPosition <= 0.0)
                currentPosition = 0.0;
        }


        bWasPressed = gamepad1.b;
        aWasPressed = gamepad1.a;

        arm.setPosition(currentPosition);
    }
}
