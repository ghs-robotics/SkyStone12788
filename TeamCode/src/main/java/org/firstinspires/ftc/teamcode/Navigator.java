package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

import javax.crypto.ExemptionMechanism;

public class Navigator {
    private MecanumDrive mecanumDrive;
    private ArmControllerIK arm;
    private VuforiaWrangler vuforiaWrangler;
    private List<Pose> waypoints;
    private int index;
    private Telemetry telemetry;
    private ElapsedTime elapsedTime;

    public Navigator(MecanumDrive mecanumDrive, ArmControllerIK arm, VuforiaWrangler vuforiaWrangler, Telemetry telemetry) {
        this.mecanumDrive = mecanumDrive;
        this.vuforiaWrangler = vuforiaWrangler;
        this.telemetry = telemetry;
        this.arm = arm;
        waypoints = new ArrayList<>();
        index = 0;
        elapsedTime = new ElapsedTime();
    }

    void init() {
        waypoints.add(new MecanumPose(45, -38, 0));
        waypoints.add(new MecanumPose(true, 1));
        waypoints.add(new MecanumPose(45, -28, 0));
        waypoints.add(new MecanumPose(true, 1));
        waypoints.add(new MecanumPose(55, -28, 0));
        waypoints.add(new MecanumPose(true, 1));
        waypoints.add(new MecanumPose(55, -38, 0));
        waypoints.add(new MecanumPose(true, 1));
    }

    void update() {
        Pose current = waypoints.get(index);

        telemetry.addData("mode", mecanumDrive.getMode());
        telemetry.addData("index", index);

        if(current instanceof MecanumPose) {
            MecanumPose mecCurrent = (MecanumPose)current;
            telemetry.addData("motion", mecCurrent.motion);
            if (mecCurrent.waitForStable && !mecanumDrive.isDone())
                elapsedTime.reset();

            if ((mecCurrent.motion != Motion.PAUSE && mecanumDrive.isDone())
                    || (mecCurrent.motion == Motion.PAUSE && elapsedTime.seconds() > current.time)) {
                index++;
                if (index >= waypoints.size())
                    index = 0;
                elapsedTime.reset();
            }
        } else if(current instanceof ArmPose && arm != null) {
            ArmPose armCurrent = (ArmPose)current;
//            telemetry.addData("arm Mode", armCurrent.mode);
//            telemetry.addData("arm state", armCurrent.state);
//            telemetry.addData("arm placing state", armCurrent.placingState);
            if(!arm.isDone()) {
                elapsedTime.reset();
            }

            if(arm.isDone() && elapsedTime.seconds() > armCurrent.time) {
                index++;
                if(index >= waypoints.size())
                    index = 0;
                elapsedTime.reset();
            }
        }
        current = waypoints.get(index);
        if(current instanceof MecanumPose) {
            MecanumPose mecCurrent = (MecanumPose)current;
            switch (mecCurrent.motion) {
                case TRANSLATION:
                    runPosition(mecCurrent);
                    break;
                case ROTATION:
                    runRotation(mecCurrent);
                    break;
                case TRANSLATION_AND_ROTATION:
                    runPositionAndRotation(mecCurrent);
                    break;
                case TIME_MOVE:
                    runTimeMove(mecCurrent);
                case PAUSE:

                    break;
            }
        } else if(current instanceof ArmPose && arm != null) {
            runArm((ArmPose)current);
        }
    }

    private void runPosition(MecanumPose pose) {
        mecanumDrive.setMode(MecanumDrive.Mode.AUTO_TRANSLATE);
        if (vuforiaWrangler.isTargetVisible() && vuforiaWrangler.isTargetStone())
            mecanumDrive.setTarget(pose.x, pose.y);
        else
            mecanumDrive.setMode(MecanumDrive.Mode.E_STOP);
    }

    private void runRotation(MecanumPose pose) {
        mecanumDrive.setMode(MecanumDrive.Mode.AUTO_ROTATE);
        mecanumDrive.setTarget(pose.r);
    }

    private void runPositionAndRotation(MecanumPose pose) {
        mecanumDrive.setMode(MecanumDrive.Mode.AUTO_TRANSLATE_ROTATE);

        mecanumDrive.setTarget(pose.r);

        double[] test = new double[] {1, 2, 3};

        if (vuforiaWrangler.isTargetVisible() && !vuforiaWrangler.isTargetStone())
            mecanumDrive.setTarget(pose.x, pose.y);
        else
            mecanumDrive.setMode(MecanumDrive.Mode.E_STOP);
    }

    private void runTimeMove(MecanumPose pose) {
        mecanumDrive.driveXYR(pose.x, pose.y, pose.r, true);
    }

    private void runArm(ArmPose pose) {
//        arm.targetState = pose.state;
//        if (pose.placingState != null){
//            arm.targetPlacingState = pose.placingState;
//        }
//        arm.mode = pose.mode;
        arm.setPositionIK(pose.x, pose.y);
    }

    class Pose {
        public double time;
    }

    class MecanumPose extends Pose{
        public double x, y;
        public double r;
//        public double time;
        public Motion motion;
        public boolean waitForStable = false;

        MecanumPose(double x, double y) {
            this.x = x;
            this.y = y;
            motion = Motion.TRANSLATION;
        }
        MecanumPose(double r) {
            this.r = r;
            motion = Motion.ROTATION;
        }
        MecanumPose(double x, double y, double r) {
            this.x = x;
            this.y = y;
            this.r = r;
            motion = Motion.TRANSLATION_AND_ROTATION;
        }

        MecanumPose(double x, double y, double r, double time) {
            this.x = x;
            this.y = y;
            this.r = r;
            this.time = time;
            this.motion = Motion.TIME_MOVE;
        }

        MecanumPose(boolean waitForStable, double time) {
            this.time = time;
            this.waitForStable = waitForStable;
            motion = Motion.PAUSE;
        }
    }

    class ArmPose extends Pose{
//        public ArmControl.Mode mode;
//        public ArmControl.State state;
//        public ArmControl.Placing placingState;
        public double x, y;

        public ArmPose(double X, double Y, double time) {
            this.x = X;
            this.y = Y;
            this.time = time;
        }
    }

    enum Motion {TRANSLATION, ROTATION, TRANSLATION_AND_ROTATION, TIME_MOVE, PAUSE}
}
