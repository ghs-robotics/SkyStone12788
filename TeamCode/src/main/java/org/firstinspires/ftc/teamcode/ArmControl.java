package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmControl {
    private final String MOTOR_NAME = "lowerArm";
    // A constant *specific to our motor*, to pass to setTargerPosition. If we use a different
    // motor, this must be updated.
    private final double TICK_MAX = 973;
    private final double DEGREES_PER_TICK = TICK_MAX / 360.0;
    // Specific to TeleOp; this can be tweaked depending on how sensitive we want the arm to move.
    private final int DEGREES_PER_SECOND = 50;
    // If the operator is left handed
    private final boolean USE_LEFT_STICK = false;

    public Mode mode = Mode.GO;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private Gamepad gamepad;
    private DcMotor lowerMotor;
    private ElapsedTime deltaTime;

    private boolean makeHardwareCalls;
    // TODO: determine suitable resting position.
    private double targetDegrees = 180;

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
        targetDegrees = position * 360;
    }

    private void armGo() {
        if (gamepad != null) {
            if (!USE_LEFT_STICK) {
                targetDegrees += gamepad.right_stick_y * DEGREES_PER_SECOND * deltaTime.seconds();
            } else {
                targetDegrees += gamepad.left_stick_y  * DEGREES_PER_SECOND * deltaTime.seconds();
            }
            deltaTime.reset();
        }

        telemetry.addData("target degrees: (%.2f)", targetDegrees);

        if(makeHardwareCalls) {
            lowerMotor.setTargetPosition((int) Math.round(targetDegrees / DEGREES_PER_TICK));
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
