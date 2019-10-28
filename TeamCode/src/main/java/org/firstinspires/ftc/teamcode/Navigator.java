package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

public class Navigator {
    private MecanumDrive mecanumDrive;
    private VuforiaWrangler vuforiaWrangler;
    private List<Pose> waypoints;
    private int index;
    private Telemetry telemetry;
    private ElapsedTime elapsedTime;

    public Navigator(MecanumDrive mecanumDrive, VuforiaWrangler vuforiaWrangler, Telemetry telemetry) {
        this.mecanumDrive = mecanumDrive;
        this.vuforiaWrangler = vuforiaWrangler;
        this.telemetry = telemetry;
        waypoints = new ArrayList<>();
        index = 0;
        elapsedTime = new ElapsedTime();
    }

    void init() {
        waypoints.add(new Pose(45, -38, 0));
        waypoints.add(new Pose(true, 1));
        waypoints.add(new Pose(45, -28, 0));
        waypoints.add(new Pose(true, 1));
        waypoints.add(new Pose(55, -28, 0));
        waypoints.add(new Pose(true, 1));
        waypoints.add(new Pose(55, -38, 0));
        waypoints.add(new Pose(true, 1));
    }

    void update() {
        Pose current = waypoints.get(index);

        telemetry.addData("mode", mecanumDrive.getMode());
        telemetry.addData("index", index);
        telemetry.addData("motion", current.motion);

        if (current.waitForStable && !mecanumDrive.isDone())
            elapsedTime.reset();

        if ((current.motion != Motion.PAUSE && mecanumDrive.isDone())
                || (current.motion == Motion.PAUSE && elapsedTime.seconds() > current.time)) {
            index++;
            if (index >= waypoints.size())
                index = 0;
            elapsedTime.reset();
        }

        current = waypoints.get(index);

        switch (current.motion) {
            case TRANSLATION:
                runPosition(current);
                break;
            case ROTATION:
                runRotation(current);
                break;
            case TRANSLATION_AND_ROTATION:
                runPositionAndRotation(current);
                break;
            case PAUSE:

                break;
        }
    }

    private void runPosition(Pose pose) {
        mecanumDrive.setMode(MecanumDrive.Mode.AUTO_TRANSLATE);
        if (vuforiaWrangler.isTargetVisible() && vuforiaWrangler.isTargetStone())
            mecanumDrive.setTarget(pose.x, pose.y);
        else
            mecanumDrive.setMode(MecanumDrive.Mode.E_STOP);
    }

    private void runRotation(Pose pose) {
        mecanumDrive.setMode(MecanumDrive.Mode.AUTO_ROTATE);
        mecanumDrive.setTarget(pose.r);
    }



    private void runPositionAndRotation(Pose pose) {
        mecanumDrive.setMode(MecanumDrive.Mode.AUTO_TRANSLATE_ROTATE);

        mecanumDrive.setTarget(pose.r);

        double[] test = new double[] {1, 2, 3};

        if (vuforiaWrangler.isTargetVisible() && !vuforiaWrangler.isTargetStone())
            mecanumDrive.setTarget(pose.x, pose.y);
        else
            mecanumDrive.setMode(MecanumDrive.Mode.E_STOP);
    }

    class Pose {
        public double x, y;
        public double r;
        public double time;
        public Motion motion;
        public boolean waitForStable = false;
        Pose(double x, double y) {
            this.x = x;
            this.y = y;
            motion = Motion.TRANSLATION;
        }
        Pose(double r) {
            this.r = r;
            motion = Motion.ROTATION;
        }
        Pose(double x, double y, double r) {
            this.x = x;
            this.y = y;
            this.r = r;
            motion = Motion.TRANSLATION_AND_ROTATION;
        }
        Pose(boolean waitForStable, double time) {
            this.time = time;
            this.waitForStable = waitForStable;
            motion = Motion.PAUSE;
        }
    }

    enum Motion {TRANSLATION, ROTATION, TRANSLATION_AND_ROTATION, PAUSE}
}
