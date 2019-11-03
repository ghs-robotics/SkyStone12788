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
    public static final double POWER_TO_RESET = 0.2,
                                POWER_TO_DUCK = 1.0,
                                POWER_TO_INTAKE = 0.2,
                                POWER_TO_PLACING = 1,
                                POWER_IN_PLACING = 0.1;
    public static final int DONE_TOLERANCE_TICKS = 60;
    public static final double RESET_F = 0,
                                DUCK_F = -.15,
                                INTAKE_F = -.15,
                                PLACING_0_F = .15,
                                PLACING_1_F = .15,
                                PLACING_2_F = .15,
                                PLACING_3_F = .15;
//    public static final double SERVO_POS_RESET = ,
//                                SERVO_POS_DUCK = ,
//                                SERVO_POS_INTAKE = 0.5,
//                                SERVO_POS_PLACING0 = ,
//                                SERVO_POS_PLACING1 = ,
//                                SERVO_POS_PLACING2 = ,
//                                SERVO_POS_PLACING3 = ,

}
