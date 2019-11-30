package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="proof of concept arm auto", group="Iterative Opmode")
public class ArmProofOfConceptAuto extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private ArmControllerIK arm;
    private double targx = 8;
    private double targy = 20;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override    public void init() {
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

        if(runtime.seconds() < 5) {
            arm.setPositionIK(5, 10);
        } else if(runtime.seconds() < 10) {
            arm.setPositionIK(5,0);
        } else {
            arm.setPositionIK(5, -1);
        }

        arm.update();

        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
