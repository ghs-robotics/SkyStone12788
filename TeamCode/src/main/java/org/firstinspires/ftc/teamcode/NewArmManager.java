package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class NewArmManager {

    private final String LOWER_MOTOR_NAME = "lowerMotor";
    private final String UPPER_MOTOR_NAME = "upperMotor";
    private final String WRIST_SERVO_NAME = "wristServo";
    private final String GRIP_SERVO_NAME = "gripperServo";

    private DcMotorEx lowerMotor;
    private DcMotorEx upperMotor;
    private Servo wristServo;
    private Servo gripServo;

    private final double LOWER_TICKS_PER_REVOLUTION = 3892;
    private final double UPPER_TICKS_PER_REVOLUTION = 1425.2 * 2;

    private final double LOWER_STARTING_POS = Math.PI * (40.0 / 180.0);
    private final double UPPER_STARTING_POS = Math.PI * (-130.0 / 180.0);
    private final double LOWER_RESTING_POS = Math.PI * (80.0 / 180.0);
    private final double UPPER_RESTING_POS = Math.PI * (0.0 / 180.0);

    private final double LOWER_ARM_LENGTH = 21.075;
    private final double UPPER_ARM_LENGTH = 17.625;
    private final double ARM_POWER = 1;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private Gamepad gamepad;
    private ElapsedTime deltaTime;

    private final double
            CONTROL_P_UPPER = 10,
            CONTROL_I_UPPER = .0,
            CONTROL_D_UPPER = 4,
            CONTROL_P_LOWER = 10,
            CONTROL_I_LOWER = .0,
            CONTROL_D_LOWER = 4,
            CONTROL_F_UPPER_MULT = -3,
            CONTROL_F_LOWER_MULT = -3;

    private final double
            GRIP_SERVO_ZERO_POSITION = -Math.PI,
            GRIP_SERVO_MOTION_RANGE = Math.PI * 280.0 / 180.0,
            GRIP_OPEN = .1, // TODO: tune this one
            GRIP_STONE = .8,
            GRIP_CAPSTONE = .9;


    private double
            controlFLower = 0,
            controlFUpper = 0;

    private double
            stackServoCorrection = 0,
            intakeServoCorrection = 0;

    private double
            lowerArmZeroPosition,
            upperArmZeroPosition,
            lowerArmZeroPositionRadians,
            upperArmZeroPositionRadians;

    private double lowerAngle, upperAngle;
    private double targetX, targetY;
    private double gripperOrientation;
    private GripperState gripperState;
    private ArmState armState = ArmState.FLAT;
    private boolean stateInitialized = false;
    private int stackHeight = 0;
    private ElapsedTime unstowClock, flattenClock;

    NewArmManager(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.gamepad = gamepad;
        deltaTime = new ElapsedTime();
        lowerMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, LOWER_MOTOR_NAME);
        upperMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, UPPER_MOTOR_NAME);
        lowerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        upperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lowerMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        setCoefficients();
        lowerMotor.setPower(ARM_POWER);
        upperMotor.setPower(ARM_POWER);
        lowerMotor.setMotorDisable();
        upperMotor.setMotorDisable();
        wristServo = hardwareMap.get(Servo.class, WRIST_SERVO_NAME);
        gripServo = hardwareMap.get(Servo.class, GRIP_SERVO_NAME);
    }

    void update() {
        updateFeedForward();
        setCoefficients();
        updateServoCorrection();
        updateWrist();
        updateGripperControl();
        updateGripper();
        updateArmControl();
        updateArm();
        deltaTime.reset();
        telemetry.addData("armState: ", armState);
    }

    void updateFeedForward() {
        controlFLower = Math.cos(lowerAngle) * CONTROL_F_LOWER_MULT;
        controlFUpper = Math.cos(upperAngle) * CONTROL_F_UPPER_MULT;
    }

    void setCoefficients() {
        lowerMotor.setVelocityPIDFCoefficients(CONTROL_P_LOWER, CONTROL_I_LOWER, CONTROL_D_LOWER, controlFLower);
        upperMotor.setVelocityPIDFCoefficients(CONTROL_P_UPPER, CONTROL_I_UPPER, CONTROL_D_UPPER, controlFUpper);
    }

    void updateServoCorrection() {
        if (armState == ArmState.INTAKING) {
            intakeServoCorrection += .2 * deltaTime.seconds() * gamepad.left_trigger;
            intakeServoCorrection -= .2 * deltaTime.seconds() * gamepad.right_trigger;
        }
        if (armState == ArmState.STACKING) {
            stackServoCorrection -= .2 * deltaTime.seconds() * gamepad.left_trigger;
            stackServoCorrection += .2 * deltaTime.seconds() * gamepad.right_trigger;
        }

    }

    void updateWrist() {
        double gripperRelativeOrientation = gripperOrientation - upperAngle;
        gripperRelativeOrientation -= GRIP_SERVO_ZERO_POSITION;

        if (armState == ArmState.STOWED || armState == ArmState.UNSTOWING)
            return;
        if (armState == ArmState.FLAT || armState == ArmState.FLATTING)
            setGripServoPositionRadians(1.5 * Math.PI);
        if (armState == ArmState.INTAKING)
            setGripServoPositionRadians(gripperRelativeOrientation + intakeServoCorrection);
        if (armState == ArmState.STACKING)
            setGripServoPositionRadians(gripperRelativeOrientation + stackServoCorrection);
    }

    void setGripServoPositionRadians(double radians) {
        double servoUnits = radians / GRIP_SERVO_MOTION_RANGE;
        servoUnits = Range.clip(servoUnits, 0, 1);
        gripServo.setPosition(servoUnits);
    }

    void updateGripperControl() {
        if (gamepad.a)
            gripperState = GripperState.OPEN;
        if (gamepad.b)
            gripperState = GripperState.STONE;
        if (gamepad.x)
            gripperState = GripperState.CAPSTONE;
    }

    void updateGripper() {
        if (gripperState == GripperState.OPEN)
            gripServo.setPosition(GRIP_OPEN);
        if (gripperState == GripperState.STONE)
            gripServo.setPosition(GRIP_STONE);
        if (gripperState == GripperState.CAPSTONE)
            gripServo.setPosition(GRIP_CAPSTONE);
    }

    void updateArmControl() {
        if (armState == ArmState.UNSTOWING || armState == ArmState.FLATTING)
            return;
        if (gamepad.dpad_up) {
            armState = ArmState.UNSTOWING;
            stateInitialized = false;
        }
        if (gamepad.dpad_down) {
            armState = ArmState.FLATTING;
            stateInitialized = false;
        }
        if (gamepad.dpad_left) {
            armState = ArmState.INTAKING;
            stateInitialized = false;
        }
        if (gamepad.dpad_right) {
            armState = ArmState.STACKING;
            stateInitialized = false;
        }
        if (armState == ArmState.INTAKING || armState == ArmState.STACKING) {
            targetX += gamepad.left_stick_x * 5 * deltaTime.seconds();
            targetY -= gamepad.right_stick_y * 5 * deltaTime.seconds();
            telemetry.addData("targetX", targetX);
            telemetry.addData("targetY", targetY);
        }
        if (gamepad.left_bumper)
            stackHeight = Range.clip(stackHeight - 1, 0, 6);
        if (gamepad.right_bumper)
            stackHeight = Range.clip(stackHeight + 1, 0, 6);
        telemetry.addData("current lower encoder", lowerMotor.getCurrentPosition());
        telemetry.addData("current upper encoder", upperMotor.getCurrentPosition());
        telemetry.addData("lower disabled", lowerMotor.isMotorEnabled());
        telemetry.addData("upper disabled", upperMotor.isMotorEnabled());
    }

    void updateArm() {
        telemetry.addData("armState 2: ", armState);
        if (armState == ArmState.STOWED) {
            lowerMotor.setMotorDisable();
            upperMotor.setMotorDisable();
            rezeroStowed();
        }
        if (armState == ArmState.FLAT) {
            lowerMotor.setMotorDisable();
            upperMotor.setMotorDisable();
            rezeroFlat();
        }
        if (armState == ArmState.INTAKING) {
            lowerMotor.setMotorEnable();
            upperMotor.setMotorEnable();
            runIntake();
        }
        if (armState == ArmState.STACKING) {
            lowerMotor.setMotorEnable();
            upperMotor.setMotorEnable();
            telemetry.addData("lower disabled 2", lowerMotor.isMotorEnabled());
            telemetry.addData("upper disabled 2", upperMotor.isMotorEnabled());
            runStack();
        }
        if (armState == ArmState.UNSTOWING) {
            lowerMotor.setMotorEnable();
            upperMotor.setMotorEnable();
            lowerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            upperMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            unstow();
        }
        if (armState == ArmState.FLATTING) {
            lowerMotor.setMotorEnable();
            upperMotor.setMotorEnable();

            telemetry.addData("at flatten", "foo");
            flatten();
        }
    }

    void rezeroStowed() {
        lowerArmZeroPosition = lowerMotor.getCurrentPosition();
        upperArmZeroPosition = upperMotor.getCurrentPosition();
        lowerArmZeroPositionRadians = LOWER_STARTING_POS;
        upperArmZeroPositionRadians = UPPER_STARTING_POS;
    }

    void rezeroFlat() {
        lowerArmZeroPosition = lowerMotor.getCurrentPosition();
        upperArmZeroPosition = upperMotor.getCurrentPosition();
        lowerArmZeroPositionRadians = LOWER_RESTING_POS;
        upperArmZeroPositionRadians = UPPER_RESTING_POS;
    }

    void runIntake() {
        if (!stateInitialized) {
            targetX = -24;
            targetY = 6;
            lowerMotor.setTargetPosition(lowerMotor.getCurrentPosition());
            upperMotor.setTargetPosition(upperMotor.getCurrentPosition());
            lowerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            upperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            stateInitialized = true;
        }
        runByIK();
    }

    void runStack() {
        if (!stateInitialized) {
            targetX = 10;
            targetY = 6 + 4 * stackHeight;
            lowerMotor.setTargetPosition(lowerMotor.getCurrentPosition());
            upperMotor.setTargetPosition(upperMotor.getCurrentPosition());
            lowerMotor.setPower(ARM_POWER);
            upperMotor.setPower(ARM_POWER);
            lowerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            upperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            stateInitialized = true;
            telemetry.addData("lower disabled 3", lowerMotor.isMotorEnabled());
            telemetry.addData("upper disabled 3", upperMotor.isMotorEnabled());
        }
        runByIK();
    }

    void unstow() {
        if (!stateInitialized) {
            unstowClock = new ElapsedTime();
            stateInitialized = true;
        }
        if (unstowClock.seconds() < 3) {
            lowerMotor.setPower(-ARM_POWER);
            upperMotor.setPower(-ARM_POWER * .5);
        } else {
            stateInitialized = false;
            armState = ArmState.FLATTING;
        }
    }

    void flatten() {
        if (!stateInitialized) {
            flattenClock = new ElapsedTime();
            lowerAngle = Math.PI * (170.0 / 180.0);
            upperAngle = 0;
            lowerMotor.setTargetPosition(lowerMotor.getCurrentPosition());
            upperMotor.setTargetPosition(upperMotor.getCurrentPosition());
            lowerMotor.setPower(ARM_POWER);
            upperMotor.setPower(ARM_POWER);
            lowerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            upperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            stateInitialized = true;
        }
        telemetry.addData("flatten clock:", flattenClock.seconds());
        telemetry.addData("lower target:", lowerMotor.getTargetPosition());
        telemetry.addData("upper target:", upperMotor.getTargetPosition());
        telemetry.addData("lower position:", lowerMotor.getCurrentPosition());
        telemetry.addData("upper position:", upperMotor.getCurrentPosition());
        if (flattenClock.seconds() < 2) {
            runToPosition();
        } else if (flattenClock.seconds() < 4) {

//            lowerMotor.setMotorDisable();
//            upperMotor.setMotorDisable();
        } else {
            armState = ArmState.FLAT;
        }
    }

    void runToPosition() {
        telemetry.addData("lowerAngle", lowerAngle);
        telemetry.addData("upperAngle", upperAngle);
        telemetry.addData("lowerArmZeroPositionRadians", lowerArmZeroPositionRadians);
        telemetry.addData("upperArmZeroPositionRadians", upperArmZeroPositionRadians);
        double lowerAngleFromZero = Math.PI * .5 - (lowerAngle - Math.PI * .5) - lowerArmZeroPositionRadians;
        double upperAngleFromZero = upperAngle - upperArmZeroPositionRadians;
        double lowerTicksFromZero = lowerAngleFromZero / (Math.PI * 2) * LOWER_TICKS_PER_REVOLUTION;
        double upperTicksFromZero = upperAngleFromZero / (Math.PI * 2) * UPPER_TICKS_PER_REVOLUTION;
        int finalLowerTicks = (int) (lowerArmZeroPosition - lowerTicksFromZero - LOWER_TICKS_PER_REVOLUTION * .25);
        int finalUpperTicks = (int) (upperTicksFromZero + upperArmZeroPosition);
        telemetry.addData("finalLowerTicks", finalLowerTicks);
        telemetry.addData("finalUpperTicks", finalUpperTicks);
        lowerMotor.setTargetPosition(finalLowerTicks);
        upperMotor.setTargetPosition(finalUpperTicks);
    }

    void runByIK() {
        double[] jointRotations = convertIKSolveToJointRotations(
                solveAndChooseIK(0, 0, targetX, targetY, LOWER_ARM_LENGTH, UPPER_ARM_LENGTH),
                0, 0, targetX, targetY);
        lowerAngle = jointRotations[0];
        upperAngle = jointRotations[1];
        runToPosition();
    }

    double[] solveAndChooseIK(double x1, double y1, double x2, double y2, double r1, double r2) {
        double dx = x2-x1;
        double dy = y2-y1;
        double d = Math.sqrt(dx*dx+dy*dy);
        if (d > r1+r2)
            return null;
        if (d < Math.abs(r1-r2))
            return null;
        if (d == 0 && r1 == r2)
            return null;
        double a = (r1*r1-r2*r2+d*d)/(2*d);
        double h = Math.sqrt(r1*r1-a*a);
        double xm = x1 + a*dx/d;
        double ym = y1 + a*dy/d;
        double xs1 = xm + h*dy/d;
        double xs2 = xm - h*dy/d;
        double ys1 = ym - h*dx/d;
        double ys2 = ym + h*dx/d;

        return ys1 > ys2 ? new double[] {xs1, ys1} : new double[] {xs2, ys2};
    }

    double[] convertIKSolveToJointRotations(double[] ikSolve, double x1, double y1, double x2, double y2) {
        double lowerJoint = Math.atan2(ikSolve[1] - x1, ikSolve[0] - y1);
        double upperJoint = Math.atan2(y2 - ikSolve[1], x2 - ikSolve[0]);

        if (x2 < x1 && lowerJoint < 0)
            lowerJoint += 2 * Math.PI;
        if (x2 < x1 && upperJoint < 0)
            upperJoint += 2 * Math.PI;
        return new double[] {lowerJoint, upperJoint};
    }

    enum GripperState {
        OPEN, STONE, CAPSTONE
    }

    enum ArmState {
        STOWED, UNSTOWING, INTAKING, STACKING, FLAT, FLATTING
    }

}
