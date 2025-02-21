package org.firstinspires.ftc.teamcode.config;

public class FieldConfig {
    // Field dimensions
    public static final double FIELD_SIZE_MM = 3658.0;  // 12 feet in mm
    public static final double TILE_SIZE_MM = 609.6;    // 24 inches in mm

    // Game elements
    public static final double JUNCTION_HEIGHT_HIGH_MM = 850.9;
    public static final double JUNCTION_HEIGHT_MEDIUM_MM = 596.9;
    public static final double JUNCTION_HEIGHT_LOW_MM = 342.9;
    public static final double JUNCTION_HEIGHT_GROUND_MM = 0.0;

    // Starting positions
    public static class StartingPositions {
        // Blue alliance
        public static final double BLUE_LEFT_X = 0.0;
        public static final double BLUE_LEFT_Y = 0.0;
        public static final double BLUE_RIGHT_X = TILE_SIZE_MM;
        public static final double BLUE_RIGHT_Y = 0.0;

        // Red alliance
        public static final double RED_LEFT_X = FIELD_SIZE_MM - TILE_SIZE_MM;
        public static final double RED_LEFT_Y = 0.0;
        public static final double RED_RIGHT_X = FIELD_SIZE_MM;
        public static final double RED_RIGHT_Y = 0.0;
    }

    // Vision targets
    public static class VisionTargets {
        public static final int BLUE_PROP_ID = 1;
        public static final int RED_PROP_ID = 2;
        public static final int BACKDROP_TAG_ID = 3;

        public static final double BACKDROP_HEIGHT_MM = 850.9;
        public static final double PROP_HEIGHT_MM = 100.0;
    }

    // Scoring locations
    public static class ScoringLocations {
        public static final double BACKDROP_X = FIELD_SIZE_MM - 100.0;
        public static final double BACKDROP_MIN_Y = TILE_SIZE_MM;
        public static final double BACKDROP_MAX_Y = FIELD_SIZE_MM - TILE_SIZE_MM;

        public static final double PIXEL_STACK_X = FIELD_SIZE_MM / 2;
        public static final double PIXEL_STACK_Y = TILE_SIZE_MM * 2;
    }
}
