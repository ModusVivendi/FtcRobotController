package org.firstinspires.ftc.teamcode.config;

public class HardwareConfig {
    public static class DrivetrainConfig {
        // Enable flags
        public static final boolean ENABLE_FL = true;
        public static final boolean ENABLE_FR = true;
        public static final boolean ENABLE_BL = true;
        public static final boolean ENABLE_BR = true;

        // Motor names
        public static final String FL_NAME = "frontLeft";
        public static final String FR_NAME = "frontRight";
        public static final String BL_NAME = "backLeft";
        public static final String BR_NAME = "backRight";

        // Motor directions
        public static final boolean FL_REVERSE = true;
        public static final boolean FR_REVERSE = false;
        public static final boolean BL_REVERSE = true;
        public static final boolean BR_REVERSE = false;

        // Motor specifications
        public static final double TICKS_PER_REV = 537.7;  // For GoBILDA 312 RPM Yellow Jacket
        public static final double WHEEL_DIAMETER_MM = 96.0;
        public static final double GEAR_RATIO = 1.0;
    }

    public static class VerticalSlideConfig {
        // Enable flags
        public static final boolean ENABLE_LEFT = true;
        public static final boolean ENABLE_RIGHT = true;

        // Motor names
        public static final String LEFT_NAME = "vertSlideLeft";
        public static final String RIGHT_NAME = "vertSlideRight";

        // Motor specifications
        public static final double TICKS_PER_REV = 537.7;  // GoBilda Yellow Jacket
        public static final double MAX_RPM = 312;
        public static final double MAX_HEIGHT_MM = 400.0;
        public static final double MIN_HEIGHT_MM = 0.0;

        public static final double SPOOL_DIAMETER_MM = 96;

        // PID Constants
        public static final double kP = 0.015;
        public static final double kI = 0.0;
        public static final double kD = 0.001;
    }

    public static class HorizontalSlideConfig {
        // Enable flags
        public static final boolean ENABLE_LEFT = true;
        public static final boolean ENABLE_RIGHT = true;

        // Servo names
        public static final String LEFT_NAME = "horizSlideLeft";
        public static final String RIGHT_NAME = "horizSlideRight";

        // Servo specifications
        public static final double MIN_POSITION = 0.0;
        public static final double MAX_POSITION = 1.0;

        // Servo preset positions
        public static final double RETRACTED_POSITION = 0.0;
        public static final double HALF_EXTENDED_POSITION = 0.5;
        public static final double FULLY_EXTENDED_POSITION = 1.0;

        // Motion constraints
        public static final double MAX_SERVO_SPEED = 0.8;  // Maximum speed for smooth motion
        public static final double MIN_SERVO_SPEED = 0.2;  // Minimum speed for consistent motion

        // Synchronization offset (if servos aren't perfectly aligned)
        public static final double LEFT_OFFSET = 0.0;
        public static final double RIGHT_OFFSET = 0.0;
    }

    public static class ClawConfig {
        // Vertical claw
        public static final boolean ENABLE_VERT_ROTATE_LEFT = true;
        public static final boolean ENABLE_VERT_ROTATE_RIGHT = true;
        public static final boolean ENABLE_VERT_GRIPPER = true;

        public static final String VERT_ROTATE_LEFT_NAME = "vertClawRotateLeft";
        public static final String VERT_ROTATE_RIGHT_NAME = "vertClawRotateRight";
        public static final String VERT_GRIPPER_NAME = "vertClawGripper";

        // Horizontal claw
        public static final boolean ENABLE_HORIZ_ROTATE_LEFT = true;
        public static final boolean ENABLE_HORIZ_ROTATE_RIGHT = true;
        public static final boolean ENABLE_HORIZ_GRIPPER = true;
        public static final boolean ENABLE_HORIZ_COLOR = true;

        public static final String HORIZ_ROTATE_LEFT_NAME = "horizClawRotateLeft";
        public static final String HORIZ_ROTATE_RIGHT_NAME = "horizClawRotateRight";
        public static final String HORIZ_GRIPPER_NAME = "horizClawGripper";
        public static final String HORIZ_COLOR_NAME = "horizClawColor";

        // Servo positions
        public static final double VERT_ROTATE_MIN = 0.0;
        public static final double VERT_ROTATE_MAX = 1.0;
        public static final double VERT_GRIPPER_OPEN = 0.7;
        public static final double VERT_GRIPPER_CLOSED = 0.2;

        public static final double HORIZ_ROTATE_MIN = 0.0;
        public static final double HORIZ_ROTATE_MAX = 1.0;
        public static final double HORIZ_GRIPPER_OPEN = 0.7;
        public static final double HORIZ_GRIPPER_CLOSED = 0.2;
    }

    public static class SensorConfig {
        // IMU
        public static final boolean ENABLE_IMU = true;
        public static final String IMU_NAME = "imu";

        // Distance sensors
        public static final boolean ENABLE_FRONT_DISTANCE = true;
        public static final String FRONT_DISTANCE_NAME = "frontDistance";

        // Color sensors
        public static final boolean ENABLE_BOTTOM_COLOR = true;
        public static final String BOTTOM_COLOR_NAME = "bottomColor";
    }
}
