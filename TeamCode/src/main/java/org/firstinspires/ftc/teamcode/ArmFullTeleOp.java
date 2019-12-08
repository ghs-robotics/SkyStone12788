
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="full teleop", group="Iterative Opmode")

public class ArmFullTeleOp extends OpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private final String LEFT_FOUNDATION_GRIPPER_NAME = "foundationLeft", RIGHT_FOUNDATION_GRIPPER_NAME = "foundationRight";
    private Servo hookLeftServo, hookRightServo;
    private final double LEFT_FGRIPPER_CLOSED = 0, RIGHT_FGRIPPER_CLOSED = 0, LEFT_FGRIPPER_OPEN = 0, RIGHT_FGRIPPER_OPEN = 0;
    private boolean fGripperOpen = false;

    private ArmControllerIK arm;
    private double targx = 8;
    private double targy = 20;
    int placingPos = 1;


    private Gamepad driveGP, operatorGP;
    private boolean upButtonWasDown, downButtonWasDown, yButtonWasDown, xButtonWasDown, aButtonWasDown, bButtonWasDown, hookButtonWasDown;
    private boolean isPlacing;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        arm = new ArmControllerIK(hardwareMap, telemetry, gamepad2, true, true);
        hookLeftServo = hardwareMap.get(Servo.class, LEFT_FOUNDATION_GRIPPER_NAME);
        hookLeftServo = hardwareMap.get(Servo.class, RIGHT_FOUNDATION_GRIPPER_NAME);
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
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

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

        if(onDpadUpPressed) {
            placingPos++;
            if(isPlacing) {
                targx = 10.0;
                targy = 5.0 + 5.0*placingPos;
            }
        } else if(onDpadDownPressed) {
            placingPos--;
            if(isPlacing) {
                targx = 10.0;
                targy = 5.0 + 4.0*placingPos;
            }
        }

        if(onYButtonPressed) {
            isPlacing = true;
            targx = 10.0;
            targy = 5.0 + 5.0*placingPos;
        } else if (onXButtonPressed) {
            isPlacing = false;
            targx = 1.0;
            targy = 5.0;
        } else if(onAButtonPressed) {
            isPlacing = false;
            targx = -25.0;
            targy = 15.0;
        }

        if(onHookButtonPressed) {
            fGripperOpen = !fGripperOpen;
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

        double armBoost = (operatorGP.left_trigger * 2) + 1;

        arm.setPositionIK(targx, targy);
        targy -= operatorGP.left_stick_y * 20 * armBoost * (runtime.seconds());
        telemetry.addData("targy", targy);
        targx -= operatorGP.left_stick_x * 20 * armBoost * (runtime.seconds());
        telemetry.addData("targx", targx);
        //==========================================

        if(fGripperOpen) {
            hookLeftServo.setPosition(LEFT_FGRIPPER_OPEN);
            hookRightServo.setPosition(RIGHT_FGRIPPER_OPEN);
        } else {
            hookLeftServo.setPosition(LEFT_FGRIPPER_CLOSED);
            hookRightServo.setPosition(RIGHT_FGRIPPER_CLOSED);
        }

        arm.update();

        runtime.reset();

        telemetry.addData("Status", "loops per second: " + 1/runtime.seconds());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
