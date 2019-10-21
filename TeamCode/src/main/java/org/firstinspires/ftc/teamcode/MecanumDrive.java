package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

class MecanumDrive {
    private Telemetry telemetry;
    private DcMotor frDrive, flDrive, blDrive, brDrive;

    MecanumDrive(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        frDrive = hardwareMap.get(DcMotor.class, "frDrive");
        flDrive = hardwareMap.get(DcMotor.class, "flDrive");
        blDrive = hardwareMap.get(DcMotor.class, "blDrive");
        brDrive = hardwareMap.get(DcMotor.class, "brDrive");

        frDrive.setDirection(DcMotor.Direction.FORWARD);
        brDrive.setDirection(DcMotor.Direction.FORWARD);
        flDrive.setDirection(DcMotor.Direction.REVERSE);
        blDrive.setDirection(DcMotor.Direction.REVERSE);
    }

    void driveUsingGamepad(Gamepad gamepad) {
        double x =  gamepad.left_stick_x;
        double y = -gamepad.left_stick_y;
        double r = -gamepad.right_stick_x;

        driveXYR(x, y, r);
    }

    private void driveXYR(double x, double y, double r) {
        double frPower = 0, flPower = 0, blPower = 0, brPower = 0;

        // +x right, +y forward, +r counterclockwise
        frPower += y - x + r;
        flPower += y + x - r;
        blPower += y - x - r;
        brPower += y + x + r;

        double scale = Math.max(Math.max(Math.abs(frPower), Math.abs(flPower)),
                Math.max(Math.abs(blPower), Math.abs(brPower)));

        scale = 1 / scale;

        if (scale < 1) {
            frPower *= scale;
            flPower *= scale;
            blPower *= scale;
            brPower *= scale;
        }


        telemetry.addData("Motors", "fr (%.2f), fl (%.2f), bl (%.2f), br (%.2f), ",
                frPower, flPower, blPower, brPower);

        frDrive.setPower(frPower);
        flDrive.setPower(flPower);
        blDrive.setPower(blPower);
        brDrive.setPower(brPower);
    }
}
