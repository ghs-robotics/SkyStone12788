package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

class MecanumDrive {
    Mode mode;
    private Telemetry telemetry;
    private Gamepad gamepad;
    private DcMotor frDrive, flDrive, blDrive, brDrive;
    private double x, y, r, targx, targy, targr;
    private double lastx, lasty, lastr, xvel, yvel, rvel;
    private double frPower = 0, flPower = 0, blPower = 0, brPower = 0;
    private int frLastTicks, flLastTicks, blLastTicks, brLastTicks;
    private final double
            WHEEL_CIRCUMFERENCE = 3.93701 * Math.PI,
            COUNTS_PER_ROTATION = 723.24,
            DISTANCE_PER_TICK = WHEEL_CIRCUMFERENCE / COUNTS_PER_ROTATION,
            AXIS_COMPONENT = 0.5,
            TRANSLATION_P = 1,
            TRANSLATION_D = 1,
            ROTATION_P = 1,
            ROTATION_D = 1;

    boolean fake;

    MecanumDrive(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad, boolean fake) {
        this.telemetry = telemetry;
        this.gamepad = gamepad;
        this.fake = fake;

        if (fake)
            return;

        frDrive = hardwareMap.get(DcMotor.class, "frDrive");
        flDrive = hardwareMap.get(DcMotor.class, "flDrive");
        blDrive = hardwareMap.get(DcMotor.class, "blDrive");
        brDrive = hardwareMap.get(DcMotor.class, "brDrive");

        frDrive.setDirection(DcMotor.Direction.FORWARD);
        brDrive.setDirection(DcMotor.Direction.FORWARD);
        flDrive.setDirection(DcMotor.Direction.REVERSE);
        blDrive.setDirection(DcMotor.Direction.REVERSE);

        frDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        blDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        brDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        resetLastTicks();
    }

    // calculates powers according to drive mode and updates hardware
    void updateDrive() {
        if (fake) {
            driveUsingGamepad();
            telemetrizePowers();
            return;
        }

        updateVelocities();
        resetLastTicks();
        switch (mode) {
            case CONTROLLER:
                driveUsingGamepad();
                break;
            case AUTO_TRANSLATE:
                driveAutoTranslate();
                break;
            case AUTO_ROTATE:
                driveAutoRotate();
                break;
            case E_STOP:
                driveEStop();
                break;
            default:
                driveEStop();
                break;
        }
        telemetrizePowers();
        setPowers();
    }

    // sets drivebase mode
    void setMode(Mode mode) {
        this.mode = mode;
    }

    // sets the position on the field to a known position
    void resetPosition(double x, double y) {
        this.x = x;
        this.y = y;
    }

    // resets orientation (counterclockwise from +y)
    void resetRotation(double r) {
        this.r = r;
    }

    // sets AUTO_TRANSLATE mode target x and y
    void setTarget(double x, double y) {
        this.targx = x;
        this.targy = y;
    }

    // sets target orientation (counterclockwise from +y)
    void setTarget(double r) {
        this.targr = r;
    }

    // telemetrizes calculated motor powers
    private void telemetrizePowers() {
        telemetry.addData("Motors", "fr (%.2f), fl (%.2f), bl (%.2f), br (%.2f), ",
                frPower, flPower, blPower, brPower);
    }

    // writes motor powers to motors
    private void setPowers() {
        frDrive.setPower(frPower);
        flDrive.setPower(flPower);
        blDrive.setPower(blPower);
        brDrive.setPower(brPower);
    }

    // sets RunMode for all drivebase motors
    private void setMotorModes(DcMotor.RunMode runMode) {
        if (fake)
            return;

        frDrive.setMode(runMode);
        flDrive.setMode(runMode);
        blDrive.setMode(runMode);
        brDrive.setMode(runMode);
    }

    // resets last known ticks to current ticks for all motors
    private void resetLastTicks() {
        frLastTicks = frDrive.getCurrentPosition();
        flLastTicks = flDrive.getCurrentPosition();
        blLastTicks = blDrive.getCurrentPosition();
        brLastTicks = brDrive.getCurrentPosition();
    }

    // drives using x, y, and r powers in range of -1 to 1
    private void driveXYR(double x, double y, double r) {

        setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);

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
    }

    // calculates the mecanum inverse translation transform of 4 wheel values
    private double[] inverseXY(double a, double b, double c, double d) {
        double x = 0, y = 0;
        x -= AXIS_COMPONENT * 0.5 * (a + c);
        x += AXIS_COMPONENT * 0.5 * (b + d);
        y += AXIS_COMPONENT * 0.5 * (a + c);
        y -= AXIS_COMPONENT * 0.5 * (b + d);
        return new double[] {x, y};
    }

    // calculates the mecanum inverse rotation tranform of 4 wheel values
    private double inverseR(double a, double b, double c, double d) {
        double r = 0;
        r += a + d;
        r -= b + c;
        return r;
    }

    // updates x, y, and r velocities, resets last variables
    private void updateVelocities() {
        double frTickDiff = frDrive.getCurrentPosition() - frLastTicks;
        double flTickDiff = flDrive.getCurrentPosition() - flLastTicks;
        double blTickDiff = blDrive.getCurrentPosition() - blLastTicks;
        double brTickDiff = brDrive.getCurrentPosition() - brLastTicks;

        lastx = x;
        lasty = y;
        lastr = r;

        double[] inverseXY = inverseXY(frTickDiff, flTickDiff, blTickDiff, brTickDiff);
        double inverseR = inverseR(frTickDiff, flTickDiff, blTickDiff, brTickDiff);
        x = inverseXY[0];
        y = inverseXY[1];
        r = inverseR;

        x *= DISTANCE_PER_TICK;
        y *= DISTANCE_PER_TICK;
        r *= DISTANCE_PER_TICK;  // r is unitless value in ballpark of x and y

        xvel = x - lastx;
        yvel = y - lasty;
        rvel = r - lastr;
    }

    // controller drive: left stick translation, right stick rotation
    private void driveUsingGamepad() {
        setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);

        double x =  gamepad.left_stick_x;
        double y = -gamepad.left_stick_y;
        double r = -gamepad.right_stick_x;

        driveXYR(x, y, r);
    }

    // updates motor power suggestions for moving to a location
    private void driveAutoTranslate() {
        setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);

        // TODO
    }

    // updates motor power suggestions for turning to a heading
    private void driveAutoRotate() {
        setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);

        // TODO
    }

    // emergency stop--coasts with no power
    private void driveEStop() {
        setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        blDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        brDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        frPower = 0;
        flPower = 0;
        blPower = 0;
        brPower = 0;
    }

    // various driving states/modes the drivebase can be in
    enum Mode {
        CONTROLLER, AUTO_TRANSLATE, AUTO_ROTATE, E_STOP
    }
}
