package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Calibrator")
public class Calibrator extends OpMode {
    private ArmControl arm;
    private boolean aWasPressed = false;
    private boolean bWasPressed = false;
    private double currentPosition = 0.0;
    private double currentMainArmPosition = 0;
    private ElapsedTime et;
    private Gripper grip;
    private boolean tempPosition;
    private DcMotor motor;
    private Servo servo;
    private boolean leftBumperWasPressed = false;
    private boolean rightBumperWasPressed = false;
    private boolean backWasPressed = false;
    private boolean startWasPressed = false;
    //private double currentPosition = 0.5;

    @Override
    public void init() {
        arm = new ArmControl(hardwareMap, telemetry, gamepad1, true);
//        motor = hardwareMap.get(DcMotor.class, "lowerArm");
//        motor.setDirection(DcMotor.Direction.REVERSE);
//        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motor.setPower(1);
        et = new ElapsedTime();
        servo = (Servo)hardwareMap.get("wristServo");
        grip = new Gripper(hardwareMap, gamepad2);
    }

    // Arm positions, determined by empirical testing:
    //
    //
    @Override
    public void loop() {
        telemetry.addData("Arm Position", currentPosition);
        telemetry.addData("is done", arm.isDone());

        armLogic();




        telemetry.addData("Servo Position", currentPosition);
        telemetry.addData("Main Arm Position", currentMainArmPosition);

        if (gamepad2.back && !backWasPressed) {
            currentMainArmPosition += .05;

            if (currentMainArmPosition >= 1)
                currentMainArmPosition = 1;
        }

        if (gamepad2.start && !startWasPressed) {
            currentMainArmPosition -= .05;

            if (currentMainArmPosition <= 0)
                currentMainArmPosition = 0;
        }

        backWasPressed = gamepad2.back;
        startWasPressed = gamepad2.start;

//        if (gamepad2.left_bumper && !leftBumperWasPressed) {
//            currentPosition += 0.05;
//
//            if (currentPosition >= 1.0)
//                currentPosition = 1.0;
//        }
//
//        if (gamepad2.right_bumper && !rightBumperWasPressed) {
//            currentPosition -= 0.05;
//
//            if (currentPosition <= 0.0)
//                currentPosition = 0.0;
//        }


//        rightBumperWasPressed = gamepad2.right_bumper;
//        leftBumperWasPressed = gamepad2.left_bumper;

        arm.mode = ArmControl.Mode.CONTROLLER;
        arm.setPosition(currentMainArmPosition);

        arm.update();

        servo.setPosition(currentPosition);
        grip.update();
//        motor.setPower(gamepad1.right_stick_y * .25);
//        telemetry.addData("power ", gamepad1.right_stick_y * .25);
    }

    void armLogic() {
        if(gamepad2.a) {
            arm.setTargetState(ArmControl.State.DUCK);
        }
        if(gamepad2.b) {
            arm.setTargetState(ArmControl.State.RESET);
        }
        if(gamepad2.x) {
            arm.setTargetState(ArmControl.State.INTAKE);
        }
        if(gamepad2.y) {
            arm.setTargetState(ArmControl.State.PLACING);
        }

        if(gamepad2.dpad_right) {
            arm.setTargetState(ArmControl.Placing.PLACING_0);
        }
        if(gamepad2.dpad_up) {
            arm.setTargetState(ArmControl.Placing.PLACING_1);
        }
        if(gamepad2.dpad_left) {
            arm.setTargetState(ArmControl.Placing.PLACING_2);
        }
        if(gamepad2.dpad_down) {
            arm.setTargetState(ArmControl.Placing.PLACING_3);
        }

        if (gamepad2.left_bumper && !leftBumperWasPressed) {
            currentPosition += 0.05;

            if (currentPosition >= 1.0)
                currentPosition = 1.0;
        }

        if (gamepad2.right_bumper && !rightBumperWasPressed) {
            currentPosition -= 0.05;

            if (currentPosition <= 0.0)
                currentPosition = 0.0;
        }


        rightBumperWasPressed = gamepad2.right_bumper;
        leftBumperWasPressed = gamepad2.left_bumper;
    }
}
