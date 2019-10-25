package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;

//import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

class IMUWrangler {
    private BNO055IMU imu;
    private AngularVelocity angularVelocity;
    private Orientation angles;
    //private Acceleration gravity;  // no use for now
    private double niceHeading;

    IMUWrangler(HardwareMap hardwareMap) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        /*
        // the calibration file loading maybe (?) works, so the values are hardcoded
        // it really isn't needed unless the robot starts the IMU while driving or something
        // therefore, this section is commented out
        //parameters.calibrationDataFile = "AdafruitIMUCalibration.json";
        parameters.calibrationData = new BNO055IMU.CalibrationData();

        BNO055IMU.CalibrationData cd = parameters.calibrationData;
        cd.dxAccel = 0;
        cd.dxGyro = -2;
        cd.dxMag = 0;
        cd.dyAccel = -107;
        cd.dyGyro = -1;
        cd.dyMag = 0;
        cd.dzAccel = 14;
        cd.dzGyro = -1;
        cd.dzMag = 0;
        cd.radiusAccel = 1000;
        cd.radiusMag = 480;
        */

        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    void start() {
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        updateSensorInfo();
        niceHeading = 0;
    }

    void update() {
        double lastHeading = getHeading();
        updateSensorInfo();
        double currentHeading = getHeading();
        if (Math.abs(currentHeading - lastHeading) > 180)
            niceHeading += currentHeading + lastHeading;
        else
            niceHeading += currentHeading - lastHeading;
    }

    private void updateSensorInfo() {
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //gravity  = imu.getGravity();
        angularVelocity = imu.getAngularVelocity();
    }

    String getStatus() {
        return imu.getSystemStatus().toShortString();
    }

    String getCalibration() {
        return imu.getCalibrationStatus().toString();
    }

    double getHeading() {
        return AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
    }

    double getRoll() {
        return AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.secondAngle);
    }

    double getPitch() {
        return AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.thirdAngle);
    }

    double getNiceHeading() {
        return niceHeading;
    }

    double getHeadingVelocity() {
        return angularVelocity.toAngleUnit(AngleUnit.DEGREES).xRotationRate;
    }

    double getRollVelocity() {
        return angularVelocity.toAngleUnit(AngleUnit.DEGREES).yRotationRate;
    }

    double getPitchVelocity() {
        return angularVelocity.toAngleUnit(AngleUnit.DEGREES).zRotationRate;
    }
}
