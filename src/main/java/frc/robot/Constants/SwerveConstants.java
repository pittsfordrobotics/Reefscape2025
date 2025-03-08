package frc.robot.Constants;

import frc.robot.lib.SwerveModuleConstants;

public class SwerveConstants {
    private static SwerveModuleConstants FRONT_LEFT_CONSTANTS = new SwerveModuleConstants(0.157527, 2.0141, 0.14898, 0, 0, 0);
    private static SwerveModuleConstants FRONT_RIGHT_CONSTANTS = new SwerveModuleConstants(0.14548, 1.9997, 0.18201, 0, 0, 0);
    private static SwerveModuleConstants BACK_LEFT_CONSTANTS = new SwerveModuleConstants(0.15314, 1.9436, 0.1457, 0, 0, 0);
    private static SwerveModuleConstants BACK_RIGHT_CONSTANTS = new SwerveModuleConstants(0.14703, 2.0077, 0.18546, 0, 0, 0);
    public static final SwerveModuleConstants[] MODULE_CONSTANTS = {FRONT_LEFT_CONSTANTS, FRONT_RIGHT_CONSTANTS, BACK_LEFT_CONSTANTS, BACK_RIGHT_CONSTANTS};

    public static double SWERVE_MAXIMUM_VELOCITY = 5.5;
    public static double SWERVE_MAXIMUM_ANGULAR_VELOCITY = 13.6;

    public static final int FRONT_LEFT_MODULE_INDEX = 0;
    public static final int FRONT_RIGHT_MODULE_INDEX = 1;
    public static final int BACK_LEFT_MODULE_INDEX = 2;
    public static final int BACK_RIGHT_MODULE_INDEX = 3;

    // public class BACK_LEFT {
    //     public static final double drive_kS = 0.26965;
    //     public static final double drive_kV = 2.6011;
    //     public static final double drive_kA = 13.234;
    // }
    // public class BACK_RIGHT {
    //     public static final double drive_kS = 0.25825;
    //     public static final double drive_kV = 2.7213;
    //     public static final double drive_kA = 12.562;
    // }
    // public class FRONT_LEFT {
    //     public static final double drive_kS = 0.3199;
    //     public static final double drive_kV = 2.6512;
    //     public static final double drive_kA = 10.006;

    // }
    // public class FRONT_RIGHT {
    //     public static final double drive_kS = 0.23351;
    //     public static final double drive_kV = 2.6361;
    //     public static final double drive_kA = 13.654;
    // }
}
