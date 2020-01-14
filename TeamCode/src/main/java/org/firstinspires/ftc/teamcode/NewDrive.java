package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class NewDrive {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private Gamepad gamepad;
    private DcMotorEx
            frDrive, flDrive, blDrive, brDrive;
    private double
            frPower, flPower, blPower, brPower;
    private double
            x, y, r;

    NewDrive(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.gamepad = gamepad;

        frDrive = (DcMotorEx) hardwareMap.get(DcMotor.class, "frDrive");
        flDrive = (DcMotorEx) hardwareMap.get(DcMotor.class, "flDrive");
        blDrive = (DcMotorEx) hardwareMap.get(DcMotor.class, "blDrive");
        brDrive = (DcMotorEx) hardwareMap.get(DcMotor.class, "brDrive");

        frDrive.setDirection(DcMotor.Direction.FORWARD);
        brDrive.setDirection(DcMotor.Direction.FORWARD);
        flDrive.setDirection(DcMotor.Direction.REVERSE);
        blDrive.setDirection(DcMotor.Direction.REVERSE);

        frDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    void update() {
        x = gamepad.left_stick_x;
        y = gamepad.left_stick_y * -1;
        r = gamepad.right_stick_x;
        calculatePowers();
        setPowers();
    }

    void calculatePowers() {
        frPower = y - x + r;
        flPower = y + x - r;
        blPower = y - x - r;
        brPower = y + x + r;
    }

    void setPowers() {
        frDrive.setPower(frPower);
        flDrive.setPower(flPower);
        blDrive.setPower(blPower);
        brDrive.setPower(brPower);
    }
}
