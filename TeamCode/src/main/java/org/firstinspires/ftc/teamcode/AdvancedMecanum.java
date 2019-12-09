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
//import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Advanced Mecanum", group="Iterative Opmode")
@SuppressWarnings("unused")
public class AdvancedMecanum extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private IMUWrangler imuWrangler;
    private MecanumDrive mecanumDrive;
//    private VuforiaWrangler vuforiaWrangler;
    private Navigator navigator;
    private double currentPosition = 0.0;
    private ArmControl arm;
    private Gripper grip;
    private Servo servo;
    private boolean leftBumperWasPressed = false;
    private boolean rightBumperWasPressed = false;

    public void init() {
        imuWrangler = new IMUWrangler(hardwareMap);
//        vuforiaWrangler = new VuforiaWrangler(hardwareMap, telemetry, PhoneInfoPackage.getPhoneInfoPackage());
        mecanumDrive = new MecanumDrive(hardwareMap, telemetry, gamepad2, imuWrangler, null,false, false);
        telemetry.addData("Status", "Initialized");
        navigator = new Navigator(mecanumDrive, null, null, telemetry);
        navigator.init();
        arm = new ArmControl(hardwareMap, telemetry, gamepad2, true);
        servo = (Servo)hardwareMap.get("wristServo");
        grip = new Gripper(hardwareMap, gamepad2);
    }

    //temp to sizing cube
    public void initLoop() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        imuWrangler.update();
//        vuforiaWrangler.update();
//        navigator.update();
        mecanumDrive.setMode(MecanumDrive.Mode.CONTROLLER);
        mecanumDrive.updateDrive();

//        telemetry.addData("Arm Position", currentPosition);
        telemetry.addData("Arm Is Done", arm.isDone());
        logic();
        arm.setTargetState(ArmControl.State.INTAKE);
        arm.update();
        telemetry.addData("Servo Position", currentPosition);
        servo.setPosition(currentPosition);
        grip.update();
    }

    @Override
    public void start() {
        runtime.reset();

        imuWrangler.start();
//        vuforiaWrangler.flashlight(true);
    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        imuWrangler.update();
//        vuforiaWrangler.update();
//        navigator.update();
        mecanumDrive.setMode(MecanumDrive.Mode.CONTROLLER);
        mecanumDrive.updateDrive();

//        telemetry.addData("Arm Position", currentPosition);
        telemetry.addData("Arm Is Done", arm.isDone());
        logic();
        arm.update();
        telemetry.addData("Servo Position", currentPosition);
        servo.setPosition(currentPosition);
        grip.update();
    }

    @Override
    public void stop() {
//        vuforiaWrangler.close();
    }

    private void logic() {
        if(gamepad2.a) {
            arm.setTargetState(ArmControl.State.DUCK);
        }
        if(gamepad2.b) {
            arm.setTargetState(ArmControl.State.RESET);
        }
        if(gamepad2.x) {
            arm.setTargetState(ArmControl.State.INTAKE);
        }
        if(gamepad2.y) {
            arm.setTargetState(ArmControl.State.PLACING);
        }

        if(gamepad2.dpad_right) {
            arm.setTargetState(ArmControl.Placing.PLACING_0);
        }
        if(gamepad2.dpad_up) {
            arm.setTargetState(ArmControl.Placing.PLACING_1);
        }
        if(gamepad2.dpad_left) {
            arm.setTargetState(ArmControl.Placing.PLACING_2);
        }
        if(gamepad2.dpad_down) {
            arm.setTargetState(ArmControl.Placing.PLACING_3);
        }

        if (gamepad2.left_bumper && !leftBumperWasPressed) {
            currentPosition += 0.05;

            if (currentPosition >= 1.0)
                currentPosition = 1.0;
        }

        if (gamepad2.right_bumper && !rightBumperWasPressed) {
            currentPosition -= 0.05;

            if (currentPosition <= 0.0)
                currentPosition = 0.0;
        }


        rightBumperWasPressed = gamepad2.right_bumper;
        leftBumperWasPressed = gamepad2.left_bumper;
    }
}
