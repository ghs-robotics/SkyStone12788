package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.List;

public class Navigator {
    private MecanumDrive mecanumDrive;
    private VuforiaWrangler vuforiaWrangler;
    private List<Position> waypoints;

    public Navigator(MecanumDrive mecanumDrive, VuforiaWrangler vuforiaWrangler) {
        this.mecanumDrive = mecanumDrive;
        this.vuforiaWrangler = vuforiaWrangler;
        waypoints = new ArrayList<>();
    }

    void init() {
        waypoints.add(new Position(-20, 0));
    }

    void update() {
        if (vuforiaWrangler.isTargetVisible() && vuforiaWrangler.isTargetStone()) {
            mecanumDrive.setMode(MecanumDrive.Mode.AUTO_TRANSLATE);
            mecanumDrive.resetPosition(vuforiaWrangler.getX(), vuforiaWrangler.getY());
            mecanumDrive.setTarget(waypoints.get(0).x, waypoints.get(0).y);
        } else {
            mecanumDrive.setMode(MecanumDrive.Mode.E_STOP);
        }
    }

    class Position {
        public double x, y;
        Position(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }
}
