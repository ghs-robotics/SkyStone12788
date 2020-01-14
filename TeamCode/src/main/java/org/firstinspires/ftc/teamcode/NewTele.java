/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="New Tele", group="Iterative Opmode")
//@Disabled
public class NewTele extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private NewDrive newDrive;
    private FoundationGripper foundationGripper;
    private DcMotorEx lowerMotor;
    private DcMotorEx upperMotor;


//    private NewArmManager armManager;

    @Override
    public void init() {
//        armManager = new NewArmManager(hardwareMap, telemetry, gamepad2);
        newDrive = new NewDrive(hardwareMap, telemetry, gamepad1);
        foundationGripper = new FoundationGripper(
                hardwareMap.get(Servo.class, "l"),
                hardwareMap.get(Servo.class, "r")
        );

        lowerMotor = (DcMotorEx)hardwareMap.get(DcMotor.class, "lowerMotor");
        upperMotor = (DcMotorEx)hardwareMap.get(DcMotor.class, "upperMotor");
        lowerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        upperMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        if (gamepad1.a)
            foundationGripper.update(FoundationGripper.GripperState.FOUNDATION_CLOSE);
        if (gamepad1.b)
            foundationGripper.update(FoundationGripper.GripperState.FOUNDATION_OPEN);
        if (gamepad1.x)
            foundationGripper.update(FoundationGripper.GripperState.BLOCK_CLOSE);
        if (gamepad1.y)
            foundationGripper.update(FoundationGripper.GripperState.BLOCK_OPEN);

        lowerMotor.setMotorDisable();
        upperMotor.setMotorDisable();
//
//        if (gamepad2.right_bumper) {
//            lowerMotor.setTargetPosition(0);
//            upperMotor.setTargetPosition(500);
//            lowerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            upperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            lowerMotor.setPower(1);
//            upperMotor.setPower(1);
//            lowerMotor.setMotorEnable();
//            upperMotor.setMotorEnable();
//
//        }

        lowerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        upperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (gamepad2.a) {
            lowerMotor.setMotorEnable();
            lowerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lowerMotor.setPower(.6);
        }

        if (gamepad2.b) {
            lowerMotor.setMotorEnable();
            lowerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lowerMotor.setPower(-.6);
        }

        if (gamepad2.x) {
            upperMotor.setMotorEnable();
            upperMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            upperMotor.setPower(.6);
        }

        if (gamepad2.y) {
            upperMotor.setMotorEnable();
            upperMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            upperMotor.setPower(-.6);
        }

        foundationGripper.update();
        newDrive.update();
//        armManager.update();
    }

    @Override
    public void stop() {
    }

}
