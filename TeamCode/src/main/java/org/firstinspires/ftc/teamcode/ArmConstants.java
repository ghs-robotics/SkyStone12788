package org.firstinspires.ftc.teamcode;

public class ArmConstants {
    //arm positions for each arm state
    public static final double ARM_RESET_POSITION = 0.0,
                                ARM_DUCK_POSITION = 0.2,
                                ARM_INTAKE_POSITION = 0.1;
    //arm positions for each block placing stage
    public static final double PLACING_0 = 0.75,
                                PLACING_1 = 0.7,
                                PLACING_2 = 0.65,
                                PLACING_3 = 0.6;
    //power settings to use for each arm state
    public static final double POWER_TO_RESET = 0.1,
                                POWER_TO_DUCK = 1.0,
                                POWER_TO_INTAKE = 0.1,
                                POWER_TO_PLACING = 1,
                                POWER_IN_PLACING = 0.1;
    public static final int DONE_TOLERANCE_TICKS = 20;
}
