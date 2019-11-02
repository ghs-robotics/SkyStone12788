package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmControl {
    private final String MOTOR_NAME = "lowerArm";
    // This is the number of encoder ticks in a full rotation, passed into setTargerPosition.
    private final double FULL_ROTATION = 972; //3892
    // We only want to ever rotate to HALF_ROTATION, or we'll collide with the floor.
    private final double HALF_ROTATION = FULL_ROTATION / 2;
    // Specific to TeleOp; this is a value for how sensitive the gamepad is
    private final double POSITION_PER_SECOND = 0.14;
    // If the operator is left handed
    private final boolean USE_LEFT_STICK = false;

    public Mode mode = Mode.GO;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private Gamepad gamepad;
    private DcMotor lowerMotor;
    private ElapsedTime deltaTime;

    private boolean makeHardwareCalls;
    // A value from 0.0 to 1.0
    private double targetPosition = 0.0;

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
        lowerMotor = hardwareMap.get(DcMotor.class, MOTOR_NAME);
        lowerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lowerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lowerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void update() {
        switch (mode) {
            case GO:
                armGo();
                break;
            case E_STOP:
            default:
                armEStop();
                break;
        }

        if (gamepad != null) {
            if (gamepad.b) {
                mode = Mode.E_STOP;
            } else if (gamepad.a) {
                mode = Mode.GO;
            }
        }
    }

    /**
     * Pass in a number between 0.0 and 1.0
     */
    public void setPosition(double position) {
        targetPosition = position;
    }

    private void armGo() {
        if (gamepad != null) {
            if (!USE_LEFT_STICK) {
                targetPosition += gamepad.right_stick_y * POSITION_PER_SECOND * deltaTime.seconds();
            } else {
                targetPosition += gamepad.left_stick_y  * POSITION_PER_SECOND * deltaTime.seconds();
            }
            deltaTime.reset();
        }

        if (targetPosition > 1.0)
            targetPosition = 1.0;
        if (targetPosition < 0.0)
            targetPosition = 0.0;

        telemetry.addData("target position: (%.2f)", targetPosition);

        if(makeHardwareCalls) {
            lowerMotor.setTargetPosition((int) Math.round(targetPosition * HALF_ROTATION));
        }
    }

    private void armEStop() {
       lowerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       lowerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
       lowerMotor.setPower(0);

       telemetry.addLine("E-Stop activated");
    }

    enum Mode {
        GO, E_STOP
    }
}