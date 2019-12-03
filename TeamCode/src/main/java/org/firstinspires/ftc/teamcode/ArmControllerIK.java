package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import sun.awt.motif.X11CNS11643;

public class ArmControllerIK {
    private final String MOTOR_NAME_LOWER = "lowerMotor";
    private final String MOTOR_NAME_UPPER = "upperMotor";
    private final String SERVO_NAME_WRIST = "wristServo";
    private final String SERVO_NAME_GRIPPER="gripperServo";
    // This is the number of encoder ticks in a full rotation, passed into setTargetPosition.
    private final double TICKS_PER_REVOLUTION_LOWER = 3892;
    private final double TICKS_PER_REVOLUTION_UPPER = 1425.2 * 2;
    // We only want to ever rotate to HALF_ROTATION, or we'll collide with the floor.
//    private final double HALF_ROTATION = TICKS_PER_REVOLUTION_LOWER / 2;

    //note: 0 is horizontal pointing in the direction of the non arm side of the bot.
//
//    private final double STARTING_POS_RAD_LOWER = 0.34906585,
//                        STARTING_POS_RAD_UPPER = -1.134464014;

    private final double STARTING_POS_RAD_LOWER = 0,
                        STARTING_POS_RAD_UPPER = Math.PI;

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

    private double servoOffset = 0.37315;
    private double servoOffsetReverse;

    private final double
            CONTROL_P_UPPER = 10,
            CONTROL_I_UPPER = 0,
            CONTROL_D_UPPER = 4,
            CONTROL_P_LOWER = 10,
            CONTROL_I_LOWER = 0,
            CONTROL_D_LOWER = 4,
            CONTROL_F_UPPER_MULT = -3,
            CONTROL_F_LOWER_MULT = -3;

    private double
            fUpper = 0,
            fLower = 0;

    private double lowerAngle, upperAngle;
    private double targetX, targetY;
    private boolean gripperClosed = false;
    private boolean gripButtonWasPressed = false;

//    private Mode mode;
//    private State currentState;
//    private State targetState;
//    private Placing placingState;

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

        wristServo.setPosition(servoAngleServounits);


        if(targetX > 0) {
            if (gamepad.dpad_left) {
                servoOffset += .2 * deltaTime.seconds();
            } else if (gamepad.dpad_right) {
                servoOffset -= .2 * deltaTime.seconds();
            }
        } else {
            if (gamepad.dpad_left) {
                servoOffsetReverse += .2 * deltaTime.seconds();
            } else if (gamepad.dpad_right) {
                servoOffsetReverse -= .2 * deltaTime.seconds();
            }
        }

        if(!gripperClosed) {
            gripServo.setPosition(0);
        } else {
            gripServo.setPosition(.38);
        }

        if(gamepad.right_bumper && !gripButtonWasPressed) {
            gripperClosed = !gripperClosed;
        }

        gripButtonWasPressed = gamepad.right_bumper;
        deltaTime.reset();

        telemetry.addData("servo angle", servoAngleServounits);
        telemetry.addData("servo offset", servoOffset);
        telemetry.addData("servo reverse offset", servoOffsetReverse);
    }

    public void setPositionIK(double x, double y) {

        targetX = x;
        targetY = y;

        x *= -1;

        try {
            PVector[] positions = doIKForDist(x, y, 0.1);

            lowerAngle = Math.atan2(positions[0].y, positions[0].x);
            upperAngle = Math.atan2(positions[1].y, positions[1].x);

            while (upperAngle < -Math.PI / 2)
                upperAngle += Math.PI * 2;

            while (upperAngle > Math.PI * 3 / 2)
                upperAngle -= Math.PI * 2;

            if(checkCollision(lowerAngle, upperAngle)) {

                setLowerTargetAngle(lowerAngle);
                setUpperTargetAngle(upperAngle);
            }
        } catch (IllegalArgumentException e) {

        }
    }

    public boolean checkCollision(double lowerAngle, double upperAngle) {

        double fky = Math.sin(lowerAngle) + Math.sin(upperAngle);
        double jkx = Math.cos(lowerAngle) + Math.cos(upperAngle);

//        if (lowerAngle )

//        if(fky < 0) {
//            return false;
//        }

        return true;
    }

    public void setLowerTargetAngle(double radians) {
        //radians *= -1;
        radians -= STARTING_POS_RAD_LOWER;
        double revolutions = radians / (2 * Math.PI);
        int ticks = (int)Math.round(revolutions * TICKS_PER_REVOLUTION_LOWER);
        motorLower.setTargetPosition(ticks);
    }

    public void setUpperTargetAngle(double radians) {
        //radians *= -1;
        radians -= STARTING_POS_RAD_UPPER;
        double revolutions = radians / (2 * Math.PI);
        int ticks = (int)Math.round(revolutions * TICKS_PER_REVOLUTION_UPPER);
        motorUpper.setTargetPosition(ticks);
    }

    public double getLowerAngleRad() {
        int ticks = motorLower.getCurrentPosition();
        double revolutions = ticks / TICKS_PER_REVOLUTION_LOWER;
        double radians = revolutions * 2 * Math.PI;
        return radians + STARTING_POS_RAD_LOWER;
    }

    public double getUpperAngleRad() {
        int ticks = motorUpper.getCurrentPosition();
        double revolutions = ticks / TICKS_PER_REVOLUTION_UPPER;
        double radians = revolutions * 2 * Math.PI;
        return radians + STARTING_POS_RAD_UPPER;
    }

    private PVector[] doIKForDist(double x, double y, double dist) {
        if(distance(x, y) < LOWER_ARM_LENGTH - UPPER_ARM_LENGTH || distance(x, y) > LOWER_ARM_LENGTH + UPPER_ARM_LENGTH) {
            throw new IllegalArgumentException("cannot reach given coordinates");
        }

//        long before = millis();
        double angle1;
        double angle2;
        PVector middle = new PVector(LOWER_ARM_LENGTH, 0);
        PVector end = new PVector(UPPER_ARM_LENGTH, 0);

        if (x < 0) {
            angle1 = 0;
        } else {
            angle1 = Math.PI;
        }
        angle2 = Math.atan2(y - LOWER_ARM_LENGTH, x);
        int i = 0;
        while (distance(PVector.add(end, middle), x, y) > dist) {
        /*println("iter", frameCount, ":", middle, PVector.add(middle, end), x);
         println("iter", frameCount, ":", angle1, angle2, distance(PVector.add(end, middle), x, y));
         println();*/
            if (x > 0) {
                angle1 -= Math.PI / 360;
            } else {
                angle1 += Math.PI / 360;
            }
            angle2 = Math.atan2(y - Math.sin(angle1) * LOWER_ARM_LENGTH, x - LOWER_ARM_LENGTH * Math.cos(angle1));

            //update positions
            middle.rotate(angle1 - middle.heading());
            end.rotate(angle2 - end.heading());
            i++;
        }
//        println("millis:", millis() - before);
//        println("iterations taken:", i);
//        println(middle, end);
        return new PVector[] {middle, end};
    }

    double distance(double x, double y) {
        return Math.sqrt(x*x + y*y);
    }

    double distance(PVector p, double x, double y) {
        return PVector.sub(p, new PVector(x, y)).mag();
    }



    enum Mode {
        CONTROLLER, E_STOP, AUTO
    }

    enum State {
        RESET, DUCK, INTAKE, PLACING
    }

    enum Placing {
        PLACING_0, PLACING_1, PLACING_2, PLACING_3
    }
}