package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Arm Test")
public class ArmTest extends OpMode {
    private ArmControl arm;
    private boolean aWasPressed = false;
    private boolean bWasPressed = false;
    private double currentPosition = 0.0;
    private ElapsedTime et;
    private Gripper grip;
    private boolean tempPosition;
    //private DcMotor motor;

    @Override
    public void init() {
        arm = new ArmControl(hardwareMap, telemetry, gamepad1, true);
//        motor = hardwareMap.get(DcMotor.class, "lowerArm");
//        motor.setDirection(DcMotor.Direction.REVERSE);
//        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        et = new ElapsedTime();
        grip = new Gripper(hardwareMap, gamepad1);
    }

    // Arm positions, determined by empirical testing:
    //
    //
    @Override
    public void loop() {
        telemetry.addData("Arm Position", currentPosition);
        telemetry.addData("is done", arm.isDone);
//        if (gamepad1.a && !aWasPressed) {
//            currentPosition += 0.05;
//
//            if (currentPosition >= 1.0)
//                currentPosition = 1.0;
//        }
//
//        if (gamepad1.b && !bWasPressed) {
//            currentPosition -= 0.05;
//
//            if (currentPosition <= 0.0)
//                currentPosition = 0.0;
//        }
//        if(tempPosition) {
//            currentPosition = .8;
//        } else {
//            currentPosition = .2;
//        }

//        aWasPressed = gamepad1.a;
//        bWasPressed = gamepad1.b;
//        if(gamepad1.a) {
//            tempPosition = true;
//        } else if(gamepad1.b) {
//            tempPosition = false;
//        }

        arm.setPosition(currentPosition);



        arm.update();
        grip.update();
        //motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //motor.setPower(gamepad1.right_stick_y * .25);
        //motor.setPower(.15);
        //telemetry.addData("Reported Motor Position: ", motor.getCurrentPosition());

    }
}
