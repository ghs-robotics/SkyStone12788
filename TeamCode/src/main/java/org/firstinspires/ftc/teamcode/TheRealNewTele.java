
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Actual Real Tele", group="Iterative Opmode")

public class TheRealNewTele extends OpMode {


    private NewDrive newDrive;

    private final double
            WHEEL_CIRCUMFERENCE = 3.93701 * Math.PI,
            COUNTS_PER_ROTATION = 723.24,
            DISTANCE_PER_TICK = WHEEL_CIRCUMFERENCE / COUNTS_PER_ROTATION,
            ROTATION_RATIO = DISTANCE_PER_TICK,
            AXIS_COMPONENT = 0.5,
            TRANSLATION_P = .02,
            TRANSLATION_D = 0.015,
            ROTATION_P = .005,
            ROTATION_D = .002,
            ROTATION_SPEED = 0.5,
            MAX_TRANSLATION_ACCEL = 10,
            ACCEPTABLE_POSITION_ERROR = 0.5,
            TRANSLATION_POWER_DEADZONE = 0.005,
            TRANSLATION_VELOCITY_DEADZONE = 1.5,
            ACCEPTABLE_ROTATION_ERROR = 1,
            ROTATION_POWER_DEADZONE = .005,
            ROTATION_VELOCITY_DEADZONE = 40,
            MINIMUM_TRANSLATION_POWER = .10,
            MINIMUM_ROTATION_POWER = .10,
            MAX_TRANSLATION_POWER = .5,
            MAX_ROTATION_POWER = .5,
            MAX_TOTAL_POWER = .5;

    private ElapsedTime deltaTime = new ElapsedTime();
    private ElapsedTime elapsedTime = new ElapsedTime();


    private ArmControllerIK arm;
    private double targx = 8;
    private double targy = 20;
    int placingPos = 1;


    private FoundationGripper foundationGripper;

    private Gamepad driveGP, operatorGP;
    private boolean upButtonWasDown, downButtonWasDown, yButtonWasDown, xButtonWasDown, aButtonWasDown, bButtonWasDown, hookButtonWasDown;
    private boolean isPlacing, isDucking;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        newDrive = new NewDrive(hardwareMap, telemetry, gamepad1);
        foundationGripper = new FoundationGripper(
                hardwareMap.get(Servo.class, "l"),
                hardwareMap.get(Servo.class, "r")
        );
        arm = new ArmControllerIK(hardwareMap, telemetry, gamepad2, true, true);
        driveGP = gamepad1;
        operatorGP = gamepad2;

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        deltaTime.reset();
        elapsedTime.reset();
        arm.doServo = false;
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("doservo", arm.doServo);
        if(elapsedTime.seconds() < 4.0) {
            if(elapsedTime.seconds() < 2.0) {
                arm.setLowerTargetAngle(Math.PI * (90.0/180.0));
                arm.setUpperTargetAngle(Math.PI * (-130.0 / 180.0));
            } else {
                arm.setLowerTargetAngle(Math.PI * (90.0/180.0));
                arm.setUpperTargetAngle(Math.PI);
            }
        } else {
            arm.doServo = true;
            //==========================================
            // clarity vars
            boolean onDpadUpPressed = operatorGP.dpad_up && !upButtonWasDown;
            boolean onDpadDownPressed = operatorGP.dpad_down && !downButtonWasDown;
            boolean onAButtonPressed = operatorGP.a && !aButtonWasDown;
            boolean onXButtonPressed = operatorGP.x && !xButtonWasDown;
            boolean onYButtonPressed = operatorGP.y && !yButtonWasDown;
            boolean onBButtonPressed = operatorGP.b && !bButtonWasDown;
            boolean onHookButtonPressed = (driveGP.left_bumper && !hookButtonWasDown) || (operatorGP.left_bumper && !hookButtonWasDown);
            //==========================================
            // button press logic

            if (onDpadUpPressed) {
                placingPos++;
                if (isPlacing) {
                    targx = 10.0;
                    targy = 5.0 + 5.0 * placingPos;
                }
            } else if (onDpadDownPressed) {
                placingPos--;
                if (isPlacing) {
                    targx = 10.0;
                    targy = 5.0 + 4.0 * placingPos;
                }
            }

            if (onYButtonPressed) {
                // go to placing position
                isPlacing = true;
                isDucking = false;
                targx = -25.0;
                targy = 5.0 + 5.0 * placingPos;
            } else if (onXButtonPressed) {
                // go to duck position
                isPlacing = false;
                isDucking = true;
//            targx = 1.0;
//            targy = 5.0;
                arm.setLowerTargetAngle(Math.PI / 8);
                arm.setUpperTargetAngle(Math.PI - Math.PI * 1 / 8);

            } else if (onAButtonPressed) {
                // go to intake position
                isPlacing = false;
                isDucking = false;
                targx = 10.0;
                targy = 15.0;
            }


            //==========================================
            upButtonWasDown = operatorGP.dpad_up;
            downButtonWasDown = operatorGP.dpad_down;
            yButtonWasDown = operatorGP.y;
            xButtonWasDown = operatorGP.x;
            aButtonWasDown = operatorGP.a;
            bButtonWasDown = operatorGP.b;
            hookButtonWasDown = operatorGP.left_bumper || driveGP.left_bumper;
            //==========================================
            // other input logic
            if (!isDucking) {
                double armBoost = (operatorGP.left_trigger * 2) + 1;
                arm.setPositionIK(targx, targy);
                targy -= operatorGP.left_stick_y * 20 * armBoost * (deltaTime.seconds());
                telemetry.addData("targy", targy);
                targx -= operatorGP.left_stick_x * 20 * armBoost * (deltaTime.seconds());
                telemetry.addData("targx", targx);
            }
            //==========================================


            arm.update();

            deltaTime.reset();

        }

        if (gamepad1.a)
            foundationGripper.update(FoundationGripper.GripperState.FOUNDATION_CLOSE);
        if (gamepad1.b)
            foundationGripper.update(FoundationGripper.GripperState.FOUNDATION_OPEN);
        if (gamepad1.x)
            foundationGripper.update(FoundationGripper.GripperState.BLOCK_CLOSE);
        if (gamepad1.y)
            foundationGripper.update(FoundationGripper.GripperState.BLOCK_OPEN);
        if (gamepad1.dpad_down)
            foundationGripper.update(FoundationGripper.GripperState.CAPSTONE_CLOSE);

        foundationGripper.update();
        newDrive.update();

//        frDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        flDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        blDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        brDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addData("Status", "loops per second: " + 1/ deltaTime.seconds());
    }

    private double adjust(double value, double deadzone, double minimum, double clip, boolean bump) {
        double magnitude = Math.abs(value);
        if (magnitude < deadzone)
            return 0;
        if (magnitude < minimum && bump)
            value *= minimum / magnitude;
        value = Range.clip(value, -clip, clip);
        return value;
    }



    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
