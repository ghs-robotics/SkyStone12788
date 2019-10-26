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
        /*
        waypoints.add(new Pose(0));

        //waypoints.add(new Pose(-15, 0));
        //waypoints.add(new Pose(-15, 0));
        waypoints.add(new Pose(true, 4));
        waypoints.add(new Pose(90));
        waypoints.add(new Pose(true, 4));
        */

        waypoints.add(new Pose(0));
        waypoints.add(new Pose(true, 1));
        waypoints.add(new Pose(45));
        waypoints.add(new Pose(true, 1));
        waypoints.add(new Pose(90));
        waypoints.add(new Pose(true, 1));
        waypoints.add(new Pose(360));
        waypoints.add(new Pose(true, 1));
        /*waypoints.add(new Pose(180));
        waypoints.add(new Pose(true, 1));
        waypoints.add(new Pose(270));
        waypoints.add(new Pose(true, 1));
        waypoints.add(new Pose(360));
        waypoints.add(new Pose(true, 2));*/

    }

    void update() {
        Pose current = waypoints.get(index);

        if ((mecanumDrive.isDone() && !current.pause) || (current.pause && elapsedTime.seconds() > current.time)) {
            index++;
            if (index >= waypoints.size())
                index = 0;
            elapsedTime.reset();
        }

        current = waypoints.get(index);
        if (current.isTranslation)
            runPosition(current);
        else if (!current.pause)
            runRotation(current);
        //else
        //    mecanumDrive.setMode(MecanumDrive.Mode.E_STOP);


        //telemetry.addData("error for rotation", mecanumDrive.getError());
        //telemetry.addData("pause timer", elapsedTime.seconds());
        //telemetry.addData("action", index);

    }

    private void runPosition(Pose pose) {
        if (vuforiaWrangler.isTargetVisible() && vuforiaWrangler.isTargetStone()) {
            mecanumDrive.setMode(MecanumDrive.Mode.AUTO_TRANSLATE);
            // irrelevant now that MecanumDrive interfaces with Vuforia
            //mecanumDrive.resetPosition(vuforiaWrangler.getX(), vuforiaWrangler.getY());
            mecanumDrive.setTarget(waypoints.get(0).x, waypoints.get(0).y);
        } else {
            mecanumDrive.setMode(MecanumDrive.Mode.E_STOP);
        }
    }

    private void runRotation(Pose pose) {
        mecanumDrive.setMode(MecanumDrive.Mode.AUTO_ROTATE);
        mecanumDrive.setTarget(pose.r);
    }

    class Pose {
        public double x, y;
        public double r;
        public boolean isTranslation;
        public boolean pause;
        public double time;
        Pose(double x, double y) {
            this.x = x;
            this.y = y;
            isTranslation = true;
        }
        Pose(double r) {
            this.r = r;
            isTranslation = false;
        }
        Pose(boolean pause, double time) {
            this.pause = pause;
            this.time = time;
        }
    }
}
