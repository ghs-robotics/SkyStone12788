
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="new arm control tester", group="Iterative Opmode")
public class IKArmControllerTester extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private ArmControllerIK arm;
    private double targx = 8;
    private double targy = 20;
    private double lastTime = 0;

    boolean go = true;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        arm = new ArmControllerIK(hardwareMap, telemetry, gamepad1, true);
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
        lastTime = runtime.seconds();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        //angle-based
//        telemetry.addData("data a: ", arm.motorLower.getTargetPosition());
//        telemetry.addData("data b: ", arm.motorLower.getCurrentPosition());
//        telemetry.addData("data c: ", arm.motorLower.getMode());
//        telemetry.addData("get angle", arm.getLowerAngleRad());
////        telemetry.
//        if(go) {
//            if (runtime.seconds() < 3) {
//                //go to stow
//                arm.setUpperTargetAngle(Math.PI);
//                arm.setLowerTargetAngle(1 * Math.PI / 8);
//            } else if (runtime.seconds() > 6) {
//                runtime.reset();
//            } else {
//                //go to upleft ish
//                arm.setUpperTargetAngle(3 * Math.PI / 4);
//                arm.setLowerTargetAngle(5 * Math.PI / 8);
//            }
//        }
//
//        if(gamepad1.b) go = false;

//        arm.setPositionIK(4, 8 + 10 + Math.sin(runtime.seconds() * 2) * 10);
        arm.setPositionIK(targx, targy);
        targy -= gamepad1.left_stick_y * 20 * (runtime.seconds());
        telemetry.addData("targy", targy);
        targx -= gamepad1.left_stick_x * 20 * (runtime.seconds());
        telemetry.addData("targx", targx);


//        lastTime = runtime.seconds();

        arm.updateCooefficents();

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
