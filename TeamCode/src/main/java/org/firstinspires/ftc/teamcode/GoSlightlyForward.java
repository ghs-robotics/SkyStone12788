package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Go Slightly Forward")
public class GoSlightlyForward extends LinearOpMode {
    DcMotor frDrive, flDrive, blDrive, brDrive, lowerArm;
    Servo wristServo, gripperServo;

    public void runOpMode() {
        frDrive = hardwareMap.get(DcMotor.class, "frDrive");
        flDrive = hardwareMap.get(DcMotor.class, "flDrive");
        blDrive = hardwareMap.get(DcMotor.class, "blDrive");
        brDrive = hardwareMap.get(DcMotor.class, "brDrive");
        lowerArm = hardwareMap.get(DcMotor.class, "lowerArm");
        wristServo = hardwareMap.get(Servo.class, "wristServo");
        gripperServo = hardwareMap.get(Servo.class, "gripperServo");

        frDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        blDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lowerArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lowerArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frDrive.setDirection(DcMotor.Direction.FORWARD);
        brDrive.setDirection(DcMotor.Direction.FORWARD);
        flDrive.setDirection(DcMotor.Direction.REVERSE);
        blDrive.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        wristServo.setPosition(0);
        gripperServo.setPosition(.05);

        lowerArm.setPower(.2);
        sleep(250);
        lowerArm.setPower(0);

        sleep(2000);

        driveXYR(0, .25, 0);
        sleep(500);
        driveXYR(0, 0, 0);
    }

    public void driveXYR(double x, double y, double r) {
        double frPower = y - x + r;
        double flPower = y + x - r;
        double blPower = y - x - r;
        double brPower = y + x + r;

        frDrive.setPower(frPower);
        flDrive.setPower(flPower);
        blDrive.setPower(blPower);
        brDrive.setPower(brPower);
    }
}
