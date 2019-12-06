
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="full teleop", group="Iterative Opmode")

public class ArmFullTeleOp extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private ArmControllerIK arm;
    private double targx = 8;
    private double targy = 20;
    int placingPos = 1;

    private boolean upButtonWasDown, downButtonWasDown, yButtonWasDown, xButtonWasDown, aButtonWasDown, bButtonWasDown;
    private boolean isPlacing;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        arm = new ArmControllerIK(hardwareMap, telemetry, gamepad1, true, true);
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
        boolean onDpadUpPressed = gamepad1.dpad_up && !upButtonWasDown;
        boolean onDpadDownPressed = gamepad1.dpad_down && !downButtonWasDown;
        boolean onAButtonPressed = gamepad1.a && !aButtonWasDown;
        boolean onXButtonPressed = gamepad1.x && !xButtonWasDown;
        boolean onYButtonPressed = gamepad1.y && !yButtonWasDown;
        boolean onBButtonPressed = gamepad1.b && !bButtonWasDown;
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

        //==========================================
        upButtonWasDown = gamepad1.dpad_up;
        downButtonWasDown = gamepad1.dpad_down;
        yButtonWasDown = gamepad1.y;
        xButtonWasDown = gamepad1.x;
        aButtonWasDown = gamepad1.a;
        bButtonWasDown = gamepad1.b;
        //==========================================
        // other input logic

        double armBoost = (gamepad1.left_trigger * 2) + 1;

        arm.setPositionIK(targx, targy);
        targy -= gamepad1.left_stick_y * 20 * armBoost * (runtime.seconds());
        telemetry.addData("targy", targy);
        targx -= gamepad1.left_stick_x * 20 * armBoost * (runtime.seconds());
        telemetry.addData("targx", targx);

        //==========================================
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
