package org.firstinspires.ftc.teamcode;

import android.sax.TextElementListener;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="show PID coefficients", group="Linear Opmode")
public class TunePID extends LinearOpMode {
    //final double NEW_P =


    @Override
    public void runOpMode() {


        DcMotorEx lowerArmMotor = (DcMotorEx)hardwareMap.get(DcMotor.class, "lowerArm");

        waitForStart();

        PIDFCoefficients pidOrig = lowerArmMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);

        PIDCoefficients newPID = new PIDCoefficients(10.0, 0.05, 5.0);
        lowerArmMotor.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, newPID);

        PIDFCoefficients modified = lowerArmMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);

        while(opModeIsActive()) {
            //PIDCoefficients PIDnew = new PIDCoefficients(NEW_P, NEW_I, NEW_D)
            // Show the elapsed game time and wheel power.
            //telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("old P", pidOrig.p);
            telemetry.addData("old I", pidOrig.i);
            telemetry.addData("old D", pidOrig.d);
            telemetry.addData("old F", pidOrig.f);

            telemetry.addData("new p", modified.p);
            telemetry.addData("new i", modified.i);
            telemetry.addData("new d", modified.d);
            telemetry.addData("new f", modified.f);

            telemetry.update();
        }
        //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);


    }
}
