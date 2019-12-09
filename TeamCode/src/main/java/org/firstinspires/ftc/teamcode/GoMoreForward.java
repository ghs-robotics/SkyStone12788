package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Go More Forward")
public class GoMoreForward extends LinearOpMode {
    DcMotor frDrive, flDrive, blDrive, brDrive;
    private final double LEFT_FGRIPPER_CLOSED = .8, RIGHT_FGRIPPER_CLOSED = .4, LEFT_FGRIPPER_OPEN = .5, RIGHT_FGRIPPER_OPEN = .75;
    private Servo hookLeftServo, hookRightServo;

    public void runOpMode() {
        frDrive = hardwareMap.get(DcMotor.class, "frDrive");
        flDrive = hardwareMap.get(DcMotor.class, "flDrive");
        blDrive = hardwareMap.get(DcMotor.class, "blDrive");
        brDrive = hardwareMap.get(DcMotor.class, "brDrive");
        hookLeftServo = hardwareMap.get(Servo.class, "r");
        hookRightServo = hardwareMap.get(Servo.class, "l");

        frDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frDrive.setDirection(DcMotor.Direction.FORWARD);
        brDrive.setDirection(DcMotor.Direction.FORWARD);
        flDrive.setDirection(DcMotor.Direction.REVERSE);
        blDrive.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        sleep(2000);
        driveXYR(-.25, 0, 0);
        sleep(2000);
        driveXYR(0, 0, 0);
        sleep(1000);
        driveXYR(0, .25, 0);
        sleep(1000);
        driveXYR(0, 0, 0);

//        sleep(2000);
//
//        hookLeftServo.setPosition(LEFT_FGRIPPER_OPEN);
//        hookRightServo.setPosition(RIGHT_FGRIPPER_OPEN);
//
//        driveXYR(0, .25, 0);
//        sleep(1900);
//        driveXYR(0, 0, 0);
//
//        sleep(500);
//        hookLeftServo.setPosition(LEFT_FGRIPPER_CLOSED);
//        hookRightServo.setPosition(RIGHT_FGRIPPER_CLOSED);
//        sleep(500);
//
//        driveXYR(0, -.5, 0);
//        sleep(6000);
//        telemetry.addData("foo", "foo");
//
////        driveXYR(0, 0, 0);
////        sleep(4000);
//
//        hookLeftServo.setPosition(LEFT_FGRIPPER_OPEN);
//        hookRightServo.setPosition(RIGHT_FGRIPPER_OPEN);
//
//        sleep(500);
//
////        driveXYR(0, .5, 0);
////        sleep(250);
//
//        driveXYR(-.25, .05, .025);
//        sleep(6000);
//        driveXYR(0, 0, 0);
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
