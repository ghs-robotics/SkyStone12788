package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmControl {
    private final String MOTOR_NAME = "lowerArm";
    private final double DEGREES_PER_TICK = 973 / 360.0;
    private final int DEGREES_PER_SECOND = 50;
    //for if the operator is left handed i guess
    private final boolean USE_LEFT_STICK = false;

    private Mode mode;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private Gamepad gamepad;
    private DcMotor lowerMotor;
    private ElapsedTime deltaTime;

    private boolean makeHardwareCalls;
    private double targetDegrees;
    private int targetTicks;

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
                armEStop();
                break;
            default:
                armEStop();
                break;
        }

        if(gamepad.b) {
            mode = Mode.E_STOP;
        } else if(gamepad.a) {
            mode = Mode.GO;
        }

    }

    public void setMode(Mode mode) { this.mode = mode; }

    public Mode getMode() {return mode;};

    private void armGo() {
        if (!USE_LEFT_STICK) {
            targetDegrees += gamepad.right_stick_y * DEGREES_PER_SECOND * deltaTime.seconds();
        } else {
            targetDegrees += gamepad.left_stick_y * DEGREES_PER_SECOND * deltaTime.seconds();
        }
        deltaTime.reset();

        telemetry.addData("target degrees: (%.2f)", targetDegrees);

        if(makeHardwareCalls) {
            lowerMotor.setTargetPosition((int) Math.round(targetDegrees / DEGREES_PER_TICK));
        }
    }

    private void armEStop() {
        telemetry.addLine("E-Stop activated");
    }

    enum Mode {
        GO, E_STOP
    }
}
