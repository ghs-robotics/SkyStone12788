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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Mecanum and Arm", group="Iterative Opmode")
@Disabled
public class MecanumAndArm extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private IMUWrangler imuWrangler;
    private MecanumDrive mecanumDrive;
    private VuforiaWrangler vuforiaWrangler;
    private Navigator navigator;
    private ArmControllerIK armControl;

    public void init() {
        armControl = new ArmControllerIK(hardwareMap, telemetry, gamepad2, false, false);
        imuWrangler = new IMUWrangler(hardwareMap);
        vuforiaWrangler = new VuforiaWrangler(hardwareMap, telemetry, PhoneInfoPackage.getPhoneInfoPackage());
        mecanumDrive = new MecanumDrive(hardwareMap, telemetry, gamepad1, imuWrangler, vuforiaWrangler,false, false);
        telemetry.addData("Status", "Initialized");
        navigator = new Navigator(mecanumDrive, armControl, vuforiaWrangler, telemetry);
        navigator.init();
    }

    @Override
    public void start() {
        runtime.reset();
        imuWrangler.start();
        vuforiaWrangler.flashlight(true);
//        armControl.mode = ArmControl.Mode.CONTROLLER;
    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        imuWrangler.update();
        vuforiaWrangler.update();
        /*
        if (gamepad1.a)
            navigator.update();
        else if (gamepad1.b) {
            mecanumDrive.setMode(MecanumDrive.Mode.AUTO_ROTATE);
            mecanumDrive.setTarget(0);
        } else
            mecanumDrive.setMode(MecanumDrive.Mode.CONTROLLER);*/
        navigator.update();
        mecanumDrive.updateDrive();
        armControl.update();

    }

    @Override
    public void stop() {
        vuforiaWrangler.close();
    }
}