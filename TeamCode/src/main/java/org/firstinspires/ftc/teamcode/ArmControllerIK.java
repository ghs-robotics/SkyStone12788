package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmControllerIK {
    //hardware config data
    private final String MOTOR_NAME_LOWER = "lowerMotor";
    private final String MOTOR_NAME_UPPER = "upperMotor";
    private final String SERVO_NAME_WRIST = "wristServo";
    private final String SERVO_NAME_GRIPPER="gripperServo";
    // This is the number of encoder ticks in a full rotation, passed into setTargetPosition.
    private final double TICKS_PER_REVOLUTION_LOWER = 3892;
    private final double TICKS_PER_REVOLUTION_UPPER = 1425.2 * 2;
    // the position the arm will think it is in on start.
    private final double STARTING_POS_RAD_UPPER = Math.PI * (-130.0 / 180.0),
                        STARTING_POS_RAD_LOWER = Math.PI * (40.0 / 180.0);

    // the lengths of the two sections of arm.
    private final double LOWER_ARM_LENGTH = 21.075, UPPER_ARM_LENGTH = 17.625;


    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private Gamepad gamepad;
    private ElapsedTime deltaTime;

    private boolean makeHardwareCalls;
    public DcMotorEx motorLower;
    public DcMotorEx motorUpper;
    private Servo wristServo;
    private Servo gripServo;

    // PIDf coefficients for each arm section
    private final double
            CONTROL_P_UPPER = 10,
            CONTROL_I_UPPER = .0,
            CONTROL_D_UPPER = 4,
            CONTROL_P_LOWER = 10,
            CONTROL_I_LOWER = .0,
            CONTROL_D_LOWER = 4,
            CONTROL_F_UPPER_MULT = -3,
            CONTROL_F_LOWER_MULT = -3;

    private final double IK_POSITION_TOLERANCE = .1;

    // vars used to calculate feed-forward power values
    private double
            fUpper = 0,
            fLower = 0;


    // an offset to make the gripper stay level. todo: fix number & add reverse number
    private double servoOffset = 0.2899318878;
    private double servoOffsetReverse = -0.910444602;

    // the current angle each section of the arm is targeting, radians.
    // 0 is horizontal towards the foundation gripper end of the bot.
    private double lowerAngle, upperAngle;
    // x,y coords of target point, inches. X+ is towards the arm-end of the bot.
    private double targetX, targetY;
    //used to toggle gripper position
    private boolean gripperClosed = false;
    //for input handling.
    private boolean gripButtonWasPressed = false;

    public boolean doServo = false;

    /**
     * If `gamepad` is null, don't use the gamepad; in this case, setPosition will be used to
     * set the position.
     */
    public ArmControllerIK(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad, boolean makeHardwareCalls, boolean resetEncoders) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.gamepad = gamepad;
        this.makeHardwareCalls = makeHardwareCalls;

        deltaTime = new ElapsedTime();

//        servo = (Servo)hardwareMap.get("wristServo");


        if(makeHardwareCalls)
            setUpMotorHardware(resetEncoders);
    }

    private void setUpMotorHardware(boolean resetEncoders) {
        if(!makeHardwareCalls) {
            throw new IllegalStateException("setUpMotorHardware called, but makeHardwareCalls is false!");
        }

        motorLower = (DcMotorEx)hardwareMap.get(DcMotor.class, MOTOR_NAME_LOWER);
        motorLower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorUpper = (DcMotorEx)hardwareMap.get(DcMotor.class, MOTOR_NAME_UPPER);
        motorUpper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if(resetEncoders) {
            motorLower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorLower.setTargetPosition(0);
            motorLower.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motorUpper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorUpper.setTargetPosition(0);
            motorUpper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else {
//            motorLower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            motorLower.setTargetPosition(0);
//            motorLower.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            motorUpper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            motorUpper.setTargetPosition(0);
//            motorUpper.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //maybe something should happen here?
        }

        motorUpper.setDirection(DcMotor.Direction.REVERSE);

        motorUpper.setVelocityPIDFCoefficients(CONTROL_P_UPPER, CONTROL_I_UPPER, CONTROL_D_UPPER, fUpper);
        motorLower.setVelocityPIDFCoefficients(CONTROL_P_LOWER, CONTROL_I_LOWER, CONTROL_D_LOWER, fLower);

        //TODO: remove this when state machine implemented
        motorLower.setPower(1);
        motorUpper.setPower(1);



        wristServo = hardwareMap.get(Servo.class, SERVO_NAME_WRIST);
        gripServo = hardwareMap.get(Servo.class, SERVO_NAME_GRIPPER);
    }

    public void updateCooefficents() {
        fUpper = Math.cos(upperAngle) * CONTROL_F_UPPER_MULT;
        fLower = Math.cos(lowerAngle) * CONTROL_F_LOWER_MULT;

        telemetry.addData("current lower angle", getLowerAngleRad());
        telemetry.addData("current upper angle", getUpperAngleRad());

        telemetry.addData("lower IK angle", lowerAngle);
        telemetry.addData("upper IK angle", upperAngle);

        motorUpper.setVelocityPIDFCoefficients(CONTROL_P_UPPER, CONTROL_I_UPPER, CONTROL_D_UPPER, fUpper);
        motorLower.setVelocityPIDFCoefficients(CONTROL_P_LOWER, CONTROL_I_LOWER, CONTROL_D_LOWER, fLower);
    }

    public void update() {
        updateCooefficents();


        // keeping the gripper assembly level
        double servoAngle = (3 * Math.PI / 2) - getUpperAngleRad();
//        servoAngle = Math.min(1.0, Math.max(((Math.toDegrees(servoAngle) / 280)) % 360, 0.0));
        double servoAngleDegrees = Math.toDegrees(servoAngle);
        double servoAngleServounits = servoAngleDegrees / 280;
        servoAngleServounits += (targetX > 0) ? servoOffset : servoOffsetReverse;

        while(servoAngle < 0.0) {
            servoAngle += (360/280);
        }
        while(servoAngle > 1.0) {
            servoAngle -= (360/280);
        }


        telemetry.addData("lower target angle", lowerAngle);
        telemetry.addData("current lower encoder angle (rad)", getLowerAngleRad());
        telemetry.addData("upper target angle", upperAngle);
        telemetry.addData("current upper encoder angle (rad)", getUpperAngleRad());





        if(makeHardwareCalls && doServo) wristServo.setPosition(servoAngleServounits);

        if(gamepad != null) {
            // changing the gripper angle using the D-pad
            if (targetX > 0) {
                if (gamepad.dpad_left) {
                    servoOffset += .2 * deltaTime.seconds();
                } else if (gamepad.dpad_right) {
                    servoOffset -= .2 * deltaTime.seconds();
                }
            } else {
                if (gamepad.dpad_left) {
                    servoOffsetReverse -= .2 * deltaTime.seconds();
                } else if (gamepad.dpad_right) {
                    servoOffsetReverse += .2 * deltaTime.seconds();
                }
            }

            //gripper control
            if (makeHardwareCalls && doServo) {
                if (!gripperClosed) {
                    gripServo.setPosition(.1);
                } else {
                    gripServo.setPosition(.8);
                }
            }
            if (gamepad.right_bumper && !gripButtonWasPressed) {
                gripperClosed = !gripperClosed;
            }

            gripButtonWasPressed = gamepad.right_bumper;
        }

        deltaTime.reset();

        telemetry.addData("servo angle", servoAngleServounits);
        telemetry.addData("servo offset", servoOffset);
        telemetry.addData("servo reverse offset", servoOffsetReverse);
    }

    public boolean isDone() {
        double fky = Math.sin(getLowerAngleRad()) + Math.sin(getUpperAngleRad());
        double fkx = Math.cos(getLowerAngleRad()) + Math.cos(getUpperAngleRad());

        if(Math.abs(fky - targetX) <= IK_POSITION_TOLERANCE && Math.abs(fkx - targetX) <= IK_POSITION_TOLERANCE) {
            return true;
        } else {
            return false;
        }
    }

    // PRE: X and Y are sane coordinates for a target position for the arm. If the given coordinates
    //      are out of the reach of the arm, nothing will happen.
    //
    // POST: takes double X,Y for a target position in inches. X+ is towards the foundation gripper
    //       end of the robot. Uses iterative inverse kinematics algorithm to find the correct/
    //       optimal arm pose for the given position and sets the position on the motors to go to
    //       the target position.
    public void setPositionIK(double x, double y) {
        targetX = x;
        targetY = y;

        x *= -1;

        try {
            //     this can error VVV
            PVector[] positions = doIKForDist(x, y, IK_POSITION_TOLERANCE);
            //                    ^^^

            lowerAngle = Math.atan2(positions[0].y, positions[0].x);
            upperAngle = Math.atan2(positions[1].y, positions[1].x);

            if (x < 0) {
                while (upperAngle < 0)
                    upperAngle += Math.PI * 2;


                while (upperAngle > Math.PI * 2)
                    upperAngle -= Math.PI * 2;
            }

            while(lowerAngle < 0) {
                lowerAngle += 2 * Math.PI;
            }

            if(checkCollision(lowerAngle, upperAngle)) {
                setLowerTargetAngle(lowerAngle);
                setUpperTargetAngle(upperAngle);
            }
        } catch (IllegalArgumentException e) {

        }
    }

    // takes double lowerAngle, upperAngle for radian angles for each arm segment. returns false if
    // the given coordinates would make the arm hit the bot, otherwise returns true.
    public boolean checkCollision(double lowerAngle, double upperAngle) {
        return true;
//        double fky = Math.sin(lowerAngle) + Math.sin(upperAngle);
//        double fkx = Math.cos(lowerAngle) + Math.cos(upperAngle);
//
//        if(fky < 0.0 && (fkx < 0.0 && fkx > -18.0)) {
//            return false;
//        }
//        //add more constraints here
//        return true;
    }

    // takes a double, radians and moves the lower section of the arm to this angle. 0 is towards
    // the non-arm (foundation gripper) end of the bot, pi is towards the arm end.
    public void setLowerTargetAngle(double radians) {
        if(lowerAngle != radians)
            lowerAngle = radians;

        telemetry.addData("lower target angle", radians);
        telemetry.addData("current encoder angle (rad)", getLowerAngleRad());

        //radians *= -1;
        radians -= STARTING_POS_RAD_LOWER;
        double revolutions = radians / (2 * Math.PI);
        int ticks = (int)Math.round(revolutions * TICKS_PER_REVOLUTION_LOWER);
        motorLower.setTargetPosition(ticks);
    }

    // takes a double, radians and moves the upper section of the arm to this angle. 0 is towards
    // the non-arm (foundation gripper) end of the bot, pi is towards the arm end.
    public void setUpperTargetAngle(double radians) {
        if(lowerAngle != radians)
            lowerAngle = radians;


        telemetry.addData("upper target angle", radians);
        telemetry.addData("current encoder angle (rad)", getLowerAngleRad());

        //radians *= -1;
        radians -= STARTING_POS_RAD_UPPER;
        double revolutions = radians / (2 * Math.PI);
        int ticks = (int)Math.round(revolutions * TICKS_PER_REVOLUTION_UPPER);
        motorUpper.setTargetPosition(ticks);
    }

    // returns the current angle of the lower arm section in radians. 0 is towards
    // the non-arm (foundation gripper) end of the bot, pi is towards the arm end.
    public double getLowerAngleRad() {
        int ticks = motorLower.getCurrentPosition();
        double revolutions = ticks / TICKS_PER_REVOLUTION_LOWER;
        double radians = revolutions * 2 * Math.PI;
        return radians + STARTING_POS_RAD_LOWER;
    }

    // returns the current angle of the upper arm section in radians. 0 is towards
    // the non-arm (foundation gripper) end of the bot, pi is towards the arm end.
    public double getUpperAngleRad() {
        int ticks = motorUpper.getCurrentPosition();
        double revolutions = ticks / TICKS_PER_REVOLUTION_UPPER;
        double radians = revolutions * 2 * Math.PI;
        return radians + STARTING_POS_RAD_UPPER;
    }


    // PRE: double dist > 0. throws IllegalArgumentException if dist is <= 0.
    //      X,Y coordinates double x, double y are within the reach of the arm (length1 - length2 <
    //      distance < length1 + length2)
    //
    // POST: takes a position (double x, y) and a distance tolerance (double dist), both in inches.
    //       positive x is in the direction of the arm end of the robot. returns an array of PVectors,
    //       both 2d and of the length of the lower and upper arm segments rotated to have the
    //       rotation of the respective segments. the IK solution found is always the one with the
    //       higher middle point.
    private PVector[] doIKForDist(double x, double y, double dist) {
        if(distance(x, y) < LOWER_ARM_LENGTH - UPPER_ARM_LENGTH || distance(x, y) > LOWER_ARM_LENGTH + UPPER_ARM_LENGTH) {
            throw new IllegalArgumentException("cannot reach given coordinates");
        } else if (dist <= 0) {
            throw new IllegalArgumentException("IK distance tolerance of " + dist + " is invalid.");
        }

        double angle1;
        double angle2;
        PVector middle = new PVector(LOWER_ARM_LENGTH, 0);
        PVector end = new PVector(UPPER_ARM_LENGTH, 0);

        if (x < 0) {
            angle1 = 0;
        } else {
            angle1 = Math.PI;
        }
        //angle2 = Math.atan2(y - LOWER_ARM_LENGTH, x);
        while (distance(PVector.add(end, middle), x, y) > dist) {
            if (x > 0) {
                angle1 -= Math.PI / 360;
            } else {
                angle1 += Math.PI / 360;
            }
            angle2 = Math.atan2(y - Math.sin(angle1) * LOWER_ARM_LENGTH, x - LOWER_ARM_LENGTH * Math.cos(angle1));

            //update positions
            middle.rotate(angle1 - middle.heading());
            end.rotate(angle2 - end.heading());
        }
        return new PVector[] {middle, end};
    }

    private double distance(double x, double y) {
        return Math.sqrt(x*x + y*y);
    }

    private double distance(PVector p, double x, double y) {
        return PVector.sub(p, new PVector(x, y)).mag();
    }

//
//
//    enum Mode {
//        CONTROLLER, E_STOP, AUTO
//    }
//
//    enum State {
//        RESET, DUCK, INTAKE, PLACING
//    }
//
//    enum Placing {
//        PLACING_0, PLACING_1, PLACING_2, PLACING_3
//    }
}