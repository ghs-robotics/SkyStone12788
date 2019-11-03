package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxUsbUtil;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmControl {
    private final String MOTOR_NAME = "lowerArm";
    // This is the number of encoder ticks in a full rotation, passed into setTargetPosition.
    private final double FULL_ROTATION = 3892;
    // We only want to ever rotate to HALF_ROTATION, or we'll collide with the floor.
    private final double HALF_ROTATION = FULL_ROTATION / 2;
    // Specific to TeleOp; this is a value for how sensitive the gamepad is
    private final double POSITION_PER_SECOND = 0.2;
    // If the operator is left handed
    private final boolean USE_LEFT_STICK = false;
    //the angle the arm starts at. note that this is with 0 being vertical and positive values towards the "front" (arm end) of the bot.
    private final double START_ANGLE = 0.0;

    private final double FULL_POWER = 1.0;
    private final double CAREFUL_POWER = 0.1;

    private final double CONTROL_P = 10.0,
                        CONTROL_I = 0.05,
                        CONTROL_D = 5.0;


    public Mode mode = Mode.AUTO;
    public State currentState = State.RESET;
    public State targetState = State.RESET;
    public Placing currentPlacingState = Placing.PLACING_3;
    public Placing targetPlacingState = Placing.PLACING_0;

//    public boolean isDone;


    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private Gamepad gamepad;
    private DcMotorEx lowerMotor;
    private ElapsedTime deltaTime;

    private boolean makeHardwareCalls;
    // A value from 0.0 to 1.0
    private double targetPosition = 0.0;
    private double currentPower;
//    private boolean alreadyInPlacing = false;

    /**
     * If `gamepad` is null, don't use the gamepad; in this case, setPosition will be used to
     * set the position.
     */
    public ArmControl(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad, boolean makeHardwareCalls) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.gamepad = gamepad;
        this.makeHardwareCalls = makeHardwareCalls;

        deltaTime = new ElapsedTime();

        if(makeHardwareCalls)
            setUpMotorHardware();
    }

    private void setUpMotorHardware() {
        if(!makeHardwareCalls) {
            throw new IllegalStateException("setUpMotorHardware called, but makeHardwareCalls is false!");
        }

        lowerMotor = (DcMotorEx)hardwareMap.get(DcMotor.class, MOTOR_NAME);
        lowerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //lowerMotor = (DcMotorEx)hardwareMap.get(DcMotor.class, "lowerArm");

        //PIDFCoefficients pidOrig = lowerArmMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);

        //PIDFCoefficients newPID = new PIDFCoefficients(10.0, 0.05, 5.0, 0);
        lowerMotor.setVelocityPIDFCoefficients(CONTROL_P, CONTROL_I, CONTROL_D, 0.0);

        //PIDFCoefficients modified = lowerArmMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);

        lowerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lowerMotor.setTargetPosition(0);
        lowerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void update() {

        switch (mode) {
            case CONTROLLER:
                armControllerControl();
                break;
            case AUTO:
                armControllerAutonomous();
                break;
            default:
                armEStop();
                break;
        }

        if (gamepad != null) {
            if (gamepad.b) {
                mode = Mode.E_STOP;
            } else if (gamepad.a) {
                mode = Mode.CONTROLLER;
            }
        }
    }

    public boolean isDone() {
        if(Math.abs(lowerMotor.getCurrentPosition() - (int) Math.round(targetPosition * HALF_ROTATION)) <= ArmConstants.DONE_TOLERANCE_TICKS) {
            return true;
        } else {
            return false;
        }
    }

    public void setTargetState(State newTarget, Placing newPlacingTarget) {
        targetState = newTarget;
        targetPlacingState = newPlacingTarget;
    }

    public void setTargetState(State newTarget) {
        targetState = newTarget;
    }

    public void setTargetState(Placing newPlacingTarget) {
        targetPlacingState = newPlacingTarget;
    }


    /**
     * Pass in a number between 0.0 and 1.0
     */
    public void setPosition(double position) {
        targetPosition = position;
    }

    private void armControllerControl() {
        lowerMotor.setPower(currentPower);

        if (gamepad != null) {
            if (!USE_LEFT_STICK) {
                targetPosition += gamepad.right_stick_y * POSITION_PER_SECOND * deltaTime.seconds();
            } else {
                targetPosition += gamepad.left_stick_y  * POSITION_PER_SECOND * deltaTime.seconds();
            }
            deltaTime.reset();
        }

        runMotors();

//        isDone = !lowerMotor.isBusy();
    }

    private void armControllerAutonomous() {
        //lowerMotor.setPower(FULL_POWER);

        updateStateMachine();

        setTargetPosition();

        runMotors();
//        isDone = !lowerMotor.isBusy();
    }

    private void updateStateMachine() {
        if(!isDone())
            return;

        switch (targetState) {
            case DUCK:
                moveTowardsDuck();
                break;
            case RESET:
                moveTowardsReset();
                break;
            case INTAKE:
                moveTowardsIntake();
                break;
            case PLACING:
                moveTowardsPlacing();
                break;
            default:
                mode = Mode.E_STOP;
        }
    }

    private void moveTowardsDuck() {
        if(!isDone())
            return;

        currentState =  State.DUCK;
    }

    private void moveTowardsReset() {
        if(!isDone())
            return;

        if(currentState == State.DUCK || currentState == State.RESET) {
            currentState = State.RESET;
        } else {
            currentState = State.DUCK;
        }
    }

    private void moveTowardsIntake() {
        if(!isDone())
            return;

        if(currentState == State.DUCK || currentState == State.INTAKE) {
            currentState = State.INTAKE;
        } else {
            currentState = State.DUCK;
        }
    }

    private void moveTowardsPlacing() {
        if(!isDone())
            return;

        if(currentState == State.DUCK || currentState == State.PLACING) {
            currentState = State.PLACING;
            currentPlacingState = targetPlacingState;
        } else {
            currentState = State.DUCK;
        }
    }

    private void runMotors() {
        lowerMotor.setPower(currentPower);

        if (targetPosition > 1.0)
            targetPosition = 1.0;
        if (targetPosition < 0.0)
            targetPosition = 0.0;

        telemetry.addData("target position: (%.2f)", targetPosition);

        if (makeHardwareCalls) {
            lowerMotor.setTargetPosition((int) Math.round(targetPosition * HALF_ROTATION));
        }
    }

    private void setTargetPosition() {
        switch (currentState) {
            case RESET:
                targetPosition = ArmConstants.ARM_RESET_POSITION;
                currentPower = ArmConstants.POWER_TO_RESET;
                break;
            case DUCK:
                targetPosition = ArmConstants.ARM_DUCK_POSITION;
                currentPower = ArmConstants.POWER_TO_DUCK;
                break;
            case INTAKE:
                targetPosition = ArmConstants.ARM_INTAKE_POSITION;
                currentPower = ArmConstants.POWER_TO_INTAKE;
                break;
            case PLACING:
                handlePlacing(currentPlacingState);
                break;
            default:
                mode = Mode.E_STOP;
        }
    }

    private void handlePlacing(Placing placingState) {
        switch (placingState) {
            case PLACING_0:
                targetPosition = ArmConstants.PLACING_0;
                break;
            case PLACING_1:
                targetPosition = ArmConstants.PLACING_1;
                break;
            case PLACING_2:
                targetPosition = ArmConstants.PLACING_2;
                break;
            case PLACING_3:
                targetPosition = ArmConstants.PLACING_3;
                break;
        }

//        if(!alreadyInPlacing) {
//            currentPower = ArmConstants.POWER_TO_PLACING;
//        } else {
//            currentPower = ArmConstants.POWER_IN_PLACING;
//        }

        currentPower = ArmConstants.POWER_TO_PLACING;
    }




    private void armEStop() {
        lowerMotor.setPower(0);

       lowerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       lowerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
       lowerMotor.setPower(0);

       telemetry.addLine("E-Stop activated");
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