package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

class MecanumDrive {
    Mode mode;

    private ElapsedTime deltaTime;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private Gamepad gamepad;
    private IMUWrangler imuWrangler;
    private VuforiaWrangler vuforiaWrangler;
    private DcMotor frDrive, flDrive, blDrive, brDrive;
    // looking out from red alliance side
    // X axis: left to right (centered)
    // Y axis: near to far (centered)
    // r should be ccw rotation from +X towards +Y
    private double x, lastx, xvel, targx;
    private double y, lasty, yvel, targy;
    private double r, roffset, rvel, targr;

    private double frPower = 0, flPower = 0, blPower = 0, brPower = 0;
    private int frLastTicks, flLastTicks, blLastTicks, brLastTicks;
    private final double
            WHEEL_CIRCUMFERENCE = 3.93701 * Math.PI,
            COUNTS_PER_ROTATION = 723.24,
            DISTANCE_PER_TICK = WHEEL_CIRCUMFERENCE / COUNTS_PER_ROTATION,
            ROTATION_RATIO = DISTANCE_PER_TICK,
            AXIS_COMPONENT = 0.5,
            TRANSLATION_P = .065,
            TRANSLATION_D = 0.3,
            ROTATION_P = .013,
            ROTATION_D = .0017,
            ROTATION_SPEED = 0.5,
            MAX_TRANSLATION_ACCEL = 10;

    boolean fake, useEncoders;

    boolean rotdone, transdone, done;

    private double translationVel = 0;

    MecanumDrive(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad,
                 IMUWrangler imuWrangler, VuforiaWrangler vuforiaWrangler, boolean fake, boolean useEncoders) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.gamepad = gamepad;
        this.imuWrangler = imuWrangler;
        this.vuforiaWrangler = vuforiaWrangler;
        this.fake = fake;
        this.useEncoders = useEncoders;

        deltaTime = new ElapsedTime();

        setupMotorHardware();
    }

    private void setupMotorHardware() {
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
        rotdone = false;
        transdone = false;
        done = false;
        updateLocationRotationVelocity();
        resetLastTicks();

        if (mode == null)
            mode = Mode.E_STOP;

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
            case AUTO_TRANSLATE_ROTATE:
                driveAutoTranslateAndRotate();
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

        telemetry.addData("Delta Time", deltaTime.seconds());
        deltaTime.reset();
    }

    // sets drivebase mode
    void setMode(Mode mode) {
        this.mode = mode;
    }

    Mode getMode() {
        return mode;
    }

    double getTranslationError() {
        /*
        if (mode == Mode.AUTO_TRANSLATE)
            return Math.sqrt((targx - x) * (targx - x)
                           + (targy - y) * (targy - y));
        if (mode == Mode.AUTO_ROTATE)
            return Math.abs(targr - r) / 10;
        return 0;
         */
        return Math.sqrt((targx - x) * (targx - x)
                + (targy - y) * (targy - y));
    }

    boolean isDone() {
        return done;
    }

    // sets the position on the field to a known position
    void resetPosition(double x, double y) {
        this.x = x;
        this.y = y;
    }

    // resets orientation (counterclockwise from +y)
    void proposeRotation(double proposedR) {
        double currentAdjustedR = (getAdjustedRotation() % 360 + 360) % 360;
        if (currentAdjustedR > 180)
            currentAdjustedR -= 360;

        proposedR = (proposedR % 360 + 360) % 360;
        if (proposedR > 180)
            proposedR -= 360;

        double proposedCorrection = proposedR - currentAdjustedR;

        this.roffset += proposedCorrection;

//        this.roffset = (this.roffset % 360 + 360) % 360;
//        if (this.roffset > 180)
//            this.roffset -= 360;
    }

    private double getAdjustedRotation() {
        return r + roffset;
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
        if (fake)
            return;

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
        if (fake)
            return;

        frLastTicks = frDrive.getCurrentPosition();
        flLastTicks = flDrive.getCurrentPosition();
        blLastTicks = blDrive.getCurrentPosition();
        brLastTicks = brDrive.getCurrentPosition();
    }

    // drives using x, y, and r powers in range of -1 to 1
    private void driveXYR(double x, double y, double r) {

        if (useEncoders)
            setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
        else
            setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // +x right, +y forward, +r counterclockwise
        frPower = y - x + r;
        flPower = y + x - r;
        blPower = y - x - r;
        brPower = y + x + r;

        double scale = Math.max(Math.max(Math.abs(frPower), Math.abs(flPower)),
                Math.max(Math.abs(blPower), Math.abs(brPower)));

        scale = 1 / scale;


        telemetry.addData("scale", scale);
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("frPower", frPower);
        telemetry.addData("flPower", flPower);
        telemetry.addData("blPower", blPower);
        telemetry.addData("brPower", brPower);

        if (scale < 1) {
            frPower *= scale;
            flPower *= scale;
            blPower *= scale;
            brPower *= scale;
        }

//        if (scale > 1 / .1)
//            scale = 0;
//
////        scale *= .15;
//
//        scale = 10000;



//        double powerSet = .15;
//
//        frPower = Range.clip(frPower * scale, -powerSet, powerSet);
//        flPower = Range.clip(flPower * scale, -powerSet, powerSet);
//        blPower = Range.clip(blPower * scale, -powerSet, powerSet);
//        brPower = Range.clip(brPower * scale, -powerSet, powerSet);
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
    private void updateLocationRotationVelocity() {
        lastx = x;
        lasty = y;

        if (useEncoders) {
            double frTickDiff = frDrive.getCurrentPosition() - frLastTicks;
            double flTickDiff = flDrive.getCurrentPosition() - flLastTicks;
            double blTickDiff = blDrive.getCurrentPosition() - blLastTicks;
            double brTickDiff = brDrive.getCurrentPosition() - brLastTicks;

            double[] inverseXY = inverseXY(frTickDiff, flTickDiff, blTickDiff, brTickDiff);
            x = inverseXY[0];
            y = inverseXY[1];

            x *= DISTANCE_PER_TICK;
            y *= DISTANCE_PER_TICK;

            xvel = x - lastx;
            yvel = y - lasty;
        }

        telemetry.addData("hasNewInfo", vuforiaWrangler.hasNewInfo());
        telemetry.addData("isTargetStone", vuforiaWrangler.isTargetStone());
        telemetry.addData("isTargetVisible", vuforiaWrangler.isTargetVisible());

        // in real usage, it would be if the target is not a stone
        if (vuforiaWrangler.hasNewInfo() && !vuforiaWrangler.isTargetStone() && vuforiaWrangler.isTargetVisible()) {
            x = vuforiaWrangler.getX();
            y = vuforiaWrangler.getY();

            proposeRotation(vuforiaWrangler.getHeading());

            //xvel = x - lastx;
            //yvel = y - lasty;
        }

        if (fake)
            return;

        r = imuWrangler.getNiceHeading();
        rvel = imuWrangler.getHeadingVelocity();
    }

    private double[] getXYforAutoTranslate() {
        double p = Math.sqrt(Math.pow(targx - x, 2) + Math.pow(targy - y, 2));
        double d = Math.sqrt(Math.pow(xvel, 2) + Math.pow(yvel, 2));

        p *= TRANSLATION_P;
        d *= TRANSLATION_D;
        double combined = p - d;

        translationVel = combined;

        double minSpeed = .15;
        double translationDeadzone = 0.05;
        double maxSpeed = .5;

        if (getTranslationError() > 1 && Math.abs(translationVel) < minSpeed && Math.abs(translationVel) > translationDeadzone)
            translationVel *= minSpeed / Math.abs(translationVel);

        if (Math.abs(translationVel) <= translationDeadzone && getTranslationError() <= 1) {
            transdone = true;
            translationVel = 0;
        }

        if (translationVel < -1000 || translationVel > 1000)
            translationVel = 0;

        translationVel = Range.clip(translationVel, -maxSpeed, maxSpeed);

        double mover = Math.atan2(targy - y, targx - x);
        mover += Math.PI * 0.5;
        mover -= getAdjustedRotation() / 180 * Math.PI;

        telemetry.addData("P", p);
        telemetry.addData("D", d);
        telemetry.addData("translationVel", translationVel);
        telemetry.addData("mover", mover);



        return new double[] {Math.cos(mover) * translationVel, Math.sin(mover) * translationVel};
    }

    private double getRforAutoRotate() {
        double p = targr - getAdjustedRotation();
        double d = rvel;
        p *= ROTATION_P;
        d *= ROTATION_D;
        double combined = p - d;

        telemetry.addData("rotation", r);
        telemetry.addData("adjusted rotation", getAdjustedRotation());
        telemetry.addData("rotation velocity", rvel);
        telemetry.addData("combined", combined);

        double minSpeed = .11;

        if (Math.abs(targr - getAdjustedRotation()) > 1 && Math.abs(combined) < minSpeed && Math.abs(combined) > .01)
            combined *= minSpeed / Math.abs(combined);

        combined = Range.clip(combined, -ROTATION_SPEED, ROTATION_SPEED);
        if (Math.abs(targr - getAdjustedRotation()) <= 1 && Math.abs(rvel) < .5) {
            combined = 0;
            rotdone = true;
        }
        telemetry.addData("updated combined", combined);
        return combined;
    }

    // controller drive: left stick translation, right stick rotation
    private void driveUsingGamepad() {
        double x =  gamepad.left_stick_x;
        double y = -gamepad.left_stick_y;
        double r = -gamepad.right_stick_x;

        driveXYR(x, y, r);
    }

    // updates motor power suggestions for moving to a location
    private void driveAutoTranslate() {
        double[] driveInfo = getXYforAutoTranslate();
        driveXYR(driveInfo[0], driveInfo[1], 0);
        if (transdone)
            done = true;
    }

    // updates motor suggestions for turning to a heading
    private void driveAutoRotate() {
        double rotateInfo = getRforAutoRotate();
        driveXYR(0, 0, rotateInfo);
        if (rotdone)
            done = true;
    }

    private void driveAutoTranslateAndRotate() {
        double[] driveInfo = getXYforAutoTranslate();
        double rotateInfo = getRforAutoRotate();
        driveXYR(driveInfo[0], driveInfo[1], rotateInfo);
        if (rotdone && transdone)
            done = true;
    }

    // emergency stop--coasts with no power
    private void driveEStop() {
        setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (!fake) {
            frDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            flDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            blDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            brDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        frPower = 0;
        flPower = 0;
        blPower = 0;
        brPower = 0;
    }

    // various driving states/modes the drivebase can be in
    enum Mode {
        CONTROLLER, AUTO_TRANSLATE, AUTO_ROTATE, AUTO_TRANSLATE_ROTATE, E_STOP
    }
}
