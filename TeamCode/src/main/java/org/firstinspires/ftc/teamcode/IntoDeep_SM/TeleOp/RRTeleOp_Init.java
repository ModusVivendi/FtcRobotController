package org.firstinspires.ftc.teamcode.IntoDeep_SM.TeleOp;
import org.firstinspires.ftc.teamcode.config.HardwareConfig;
import org.firstinspires.ftc.teamcode.config.RobotConfig;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorEx.CurrentUnit;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Functions.ArmEncoder;  //?
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.Functions.GamepadCalc;
import org.firstinspires.ftc.teamcode.Functions.ColorSensorV3;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.advanced.PoseStorage;

@TeleOp(name="RRTeleOpIntoDeep_Init", group = "IntoDeep_SM")
@Disabled
public class RRTeleOp_Init extends LinearOpMode {
    // Load motor config
    private RobotConfig config;

    //Declare motors

    // Drive motors (GoBilda Yellow Jacket)
    private DcMotor leftMotor, rightMotor, leftMotorBack, rightMotorBack;

    // Vertical Slider Motors
    private DcMotorEx vertSlideLeft, vertSlideRight;

    // Horizontal Slider Servos
    private Servo horizSlideLeft, horizSlideRight;

    // Vertical Claw Servos
    private Servo vertClawRotateLeft, vertClawRotateRight, vertClawGripper;

    // Horizontal Claw Servos
    private Servo horizClawRotateLeft, horizClawRotateRight, horizClawGripper;

    // Color Sensor
    private ColorSensorV3 horizClawColor;

    // GoBilda motor constants
    private static final double GOBILDA_TICKS_PER_REV = 537.7; // For 19.2:1 Yellow Jacket
    private static final double GOBILDA_MAX_RPM = 312; // Max RPM for Yellow Jacket
    private static final double WHEEL_DIAMETER_MM = 96.0;  // 96mm GoBilda Mecanum wheel
    private static final double DRIVE_GEAR_RATIO = 1.0;    // Direct drive


    // Vertical slider positions
    private static final int VERT_SLIDE_MIN = 0;
    private static final int VERT_SLIDE_MAX = 2000;
    private static final int VERT_SLIDE_LOW = 100;
    private static final int VERT_SLIDE_MID = 1000;
    private static final int VERT_SLIDE_HIGH = 1800;
    // Add these constants at the top of your class
    private static final double MAX_SLIDE_POWER = 1.0; // Increase from current value
    private static final double POWER_SMOOTHING = 0.3; // Reduce for faster acceleration (was likely higher)


    // Horizontal slider positions
    private static final double HORIZ_SLIDE_MIN = 0.0;
    private static final double HORIZ_SLIDE_MAX = 0.20;
    private static final double HORIZ_SLIDE_INCREMENT = 0.02;

    // Constants for continuous servo control
    private static final double HORIZ_SERVO_STOP = 0.5;
    private static final double HORIZ_SERVO_SPEED = 0.5;

    // Claw rotation positions
    private static final double VERT_CLAW_PARALLEL = 0.5;
    private static final double VERT_CLAW_ROTATED = 0.8;
    private static final double HORIZ_CLAW_PARALLEL = 0.3;
    private static final double HORIZ_CLAW_ROTATED = 0.7;

    private ElapsedTime rotationTimer = new ElapsedTime();
    private static final double PARTIAL_ROTATION_TIME = 0.4; // Adjust this value for desired rotation amount
    private boolean isRotating = false;

    // Claw gripper positions
    private static final double CLAW_OPEN = 0.2;
    private static final double CLAW_CLOSED = 0.9;

    private static final double HCLAW_OPEN = 0.0;
    private static final double HCLAW_CLOSED = 1.0;

    // PID Constants for vertical slides
    private static final double SLIDES_P = 0.005;
    private static final double SLIDES_I = 0.0;
    private static final double SLIDES_D = 0.0;

    // State tracking
    private VertSlideState vertSlideState = VertSlideState.IDLE;
    private HorizSlideState horizSlideState = HorizSlideState.IDLE;
    private VertClawState vertClawState = VertClawState.INIT;
    private static final double SERVO_STOP = 0.5;
    private int targetVertPosition = 0;
    private double currentHorizPosition = HORIZ_SLIDE_MIN;
    private static final double VERTICAL_SLIDE_TARGET_THRESHOLD = 10; // for isAtVerticalTarget()

    // Viper slide extension motors
    private DcMotorEx armMotorLeft, armMotorRight; //?

    // Viper slide rotation motors
    private DcMotorEx rotateMotorLeft, rotateMotorRight; //?

    // Intake servos
    private Servo leftAxleServo, rightAxleServo;      // ?Servos for rotating the intake axle
    private Servo leftGeckoServo, rightGeckoServo;    // ?Servos for Gecko wheels
    private Servo horizClawServo, vertClawServo;    // ?Servos for Gecko wheels
    private Servo leftSliderServo, rightSliderServo;    // ?Servos for Gecko wheels

    // Speed profiles
    private static final double PRECISION_POWER_SMOOTHING = 0.3;
    private static final double PRECISION_MAX_POWER = 0.8;
    private static final double FAST_POWER_SMOOTHING = 0.2;
    private static final double FAST_MAX_POWER = 0.6;

    private double currentPowerSmoothing = PRECISION_POWER_SMOOTHING;
    private double currentSlideMaxPower = PRECISION_MAX_POWER;

    private boolean isClawAtLeftLimit = false;
    private boolean isClawAtRightLimit = false;

    // PID and timing variables (consolidated)
    private ElapsedTime runtime = new ElapsedTime();
    private double movement;

    // Initialize controller classes
    private ArmEncoder controller;
    private SampleMecanumDrive drive;
    private GamepadCalc gamepadCalc;


    // Constants for viper slide positions
    private static final int SLIDES_LOW_POSITION = 0;
    private static final int SLIDES_MEDIUM_POSITION = 2250;  // Adjust based on your needs
    private static final int SLIDES_HIGH_POSITION = 4500;    // Adjust based on your needs

    // Constants for rotation positions (in ticks)
    private static final int ROTATION_HORIZONTAL = 0;
    private static final int ROTATION_45_DEGREES = 384;    // Adjust based on your gear ratio
    private static final int ROTATION_VERTICAL = 768;      // Adjust based on your gear ratio

    // Constants for servo positions
    private static final double AXLE_INTAKE_POSITION = 0.0;     // Position for intaking
    private static final double AXLE_DEPOSIT_POSITION = 1.0;    // Position for depositing
    private static final double GECKO_WHEEL_STOP = 0.5;         // Neutral position
    private static final double GECKO_WHEEL_INTAKE = 1.0;       // Intake direction
    private static final double GECKO_WHEEL_OUTTAKE = 0.0;      // Outtake direction

//    // PID Constants for viper slides
//    private static final double SLIDES_P = 0.005;
//    private static final double SLIDES_I = 0.0;
//    private static final double SLIDES_D = 0.0;

    // PID Constants for rotation
    private static final double ROTATION_P = 0.005;
    private static final double ROTATION_I = 0.0;
    private static final double ROTATION_D = 0.0;

    // Initialize timing variables
    private ElapsedTime vertSlidePIDTimer = new ElapsedTime();

    // PID variables
    private ElapsedTime slidesTimer = new ElapsedTime();
    private ElapsedTime clawRotationTimer = new ElapsedTime();
    private double lastSlidesError = 0;
    private double lastRotationError = 0;
    private double slidesIntegralSum = 0;
    private double rotationIntegralSum = 0;

    // Safety constants
    private static final double VERT_SLIDE_MAX_POWER = 0.5;  // Adjust this value (0.0 to 1.0)
//    private static final double POWER_SMOOTHING = 0.2;
    private static final int SLIDES_MAX_POSITION = 4700;  // Absolute maximum extension
    private static final int SLIDES_MIN_POSITION = -10;   // Allow slight negative for zero calibration
    private static final double SLIDES_MAX_POWER = 1.0;   // Maximum allowed power
    private static final double ROTATION_MAX_POWER = 0.7; // Maximum rotation power

    // Motor current monitoring thresholds (in amps)
    private static final double CURRENT_LIMIT_SLIDES = 2.5;
    private static final double CURRENT_LIMIT_ROTATION = 2.0;

    // Coordination constants
    private static final int SAFE_ROTATION_EXTENSION = 1000; // Minimum extension for rotation
    private static final int MAX_EXTENSION_AT_ANGLE = 3000;  // Maximum extension when rotated

    // State tracking
    private boolean isOverCurrentProtected = false;
    private double lastSlidePower = 0;  // For smoothing
    private double lastRotationPower = 0;

    private double vertIntegralSum = 0;  // For accumulating error over time
    private double lastVertError = 0;    // For calculating the derivative term.

    // Deadzone Constants for Joysticks:
    private static final double JOYSTICK_DEADZONE = 0.1;
    private static final double TRIGGER_THRESHOLD = 0.5;

    //Position Safety Limits:
    // Add safety boundaries
    private static final int VERT_SLIDE_SAFETY_MARGIN = 50;  // ticks from limits
    private static final double HORIZ_SLIDE_SAFETY_MARGIN = 0.05;  // from limits

    // Movement Speed Profiles
    private static final double VERT_SLIDE_SLOW_POWER = 0.3;
    private static final double VERT_SLIDE_FAST_POWER = 0.5;
    private static final double HORIZ_SLIDE_SLOW_INCREMENT = 0.01;
    private static final double HORIZ_SLIDE_FAST_INCREMENT = 0.02;

    // Position tracking variables
    private int lastVerticalPosition = 0;
    private int currentVerticalPosition = 0;
    private double verticalVelocity = 0;
    private static final double VELOCITY_DANGER_THRESHOLD = 1000; // Adjust this value

    // State machine enums
    private enum RobotSequence {
        NONE,
        PICKUP_SEQUENCE,
        PLACE_SEQUENCE,
        HOME_SEQUENCE
    }

    private enum VertSlideState {
        IDLE,
        MANUAL_CONTROL,
        MOVING_TO_POSITION,
        ERROR
    }

    private enum HorizSlideState {
        IDLE,
        MOVING_IN,
        MOVING_OUT,
        ERROR
    }

    private enum RotationState {
        IDLE,
        MANUAL_CONTROL,
        MOVING_TO_POSITION,
        ERROR
    }
    private enum VertClawState {
        INIT,
        IDLE,
        OPEN,
        CLOSE,
        HOME,
        TRANSFER,
        ERROR
    }
    private enum HorizClawState {
        IDLE,
        OPEN,
        CLOSE,
        ERROR
    }
    private enum IntakeState {
        IDLE,
        INTAKING,
        OUTTAKING,
        ERROR
    }

    private enum RotationPosition {
        LEFT_LIMIT,
        CENTER,
        RIGHT_LIMIT,
        IDLE
    }

    private RotationPosition rotationPosition = RotationPosition.CENTER;
    private double currentVertClawPosition = SERVO_STOP;

    // State tracking
    private VertSlideState slideState = VertSlideState.IDLE;   // ??????????????
    private RotationState rotationState = RotationState.IDLE; // ??????????????
    private IntakeState intakeState = IntakeState.IDLE;
    private int targetSlidePosition = 0;
    private int targetRotationPosition = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        int tickAdjustment = 100;

        // Initialize hardware
        initializeHardware();

        // Configure motor behaviors
        setupMotors();

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();
        runtime.reset();

        if (isStopRequested()) return;

        while(opModeIsActive() && !isStopRequested()) {
            gamepadCalc.calculate();

            // Add Emergency Stop check at the start
            if (gamepadCalc.getGamepad1().back && gamepadCalc.getGamepad2().back) {
                emergencyStop();
                continue;  // Skip the rest of the loop iteration
            }

//            // Add System Health Check
//            if (!checkSystemHealth()) {
//                telemetry.addLine("⚠️ System Health Warning ⚠️");
//                telemetry.update();
//            }

            // Sequence triggers
            if (gamepadCalc.getGamepad2().share) {  // Adjust button as needed
                executeHomeSequence();
            }
            if (gamepadCalc.getGamepad2().options) {  // Adjust button as needed
                executeCycle();
            }

            movement = gamepadCalc.getGamepad1().left_trigger - gamepadCalc.getGamepad1().right_trigger;

            handleDrive(drive);
            //handleViperSlides();
            //handleRotation();
            //handleIntake();
            handleVerticalSlides();
            handleHorizontalSlides();
            handleVerticalClawContinuous();
//            handleVerticalClaw();
            handleHorizontalClaw();
            updateTelemetry();
            drive.update();

            if(gamepadCalc.getGamepad2().right_bumper) {
                drive.setPoseEstimate(PoseStorage.currentPose);
                telemetry.addData("Heading reseted to: ", PoseStorage.currentPose);
                telemetry.update();
            }
        }
    }

    private void executePickupSequence() {
        // Coordinated movement of slides and claws
    }

    private void initializeHardware() {
        config = new RobotConfig(hardwareMap);

        // Initialize drive motors
        leftMotor = config.getMotorIfEnabled("FL", HardwareConfig.DrivetrainConfig.ENABLE_FL);
        rightMotor = config.getMotorIfEnabled("FR", HardwareConfig.DrivetrainConfig.ENABLE_FR);
        leftMotorBack = config.getMotorIfEnabled("BL", HardwareConfig.DrivetrainConfig.ENABLE_BL);
        rightMotorBack = config.getMotorIfEnabled("BR", HardwareConfig.DrivetrainConfig.ENABLE_BR);

        // Initialize vertical slide motors
        vertSlideLeft = config.getMotorExIfEnabled("VSL", HardwareConfig.VerticalSlideConfig.ENABLE_LEFT);
        vertSlideRight = config.getMotorExIfEnabled("VSR", HardwareConfig.VerticalSlideConfig.ENABLE_RIGHT);

        // Initialize horizontal slide servos
        horizSlideLeft = config.getServoIfEnabled("HSL", HardwareConfig.HorizontalSlideConfig.ENABLE_LEFT);
        horizSlideRight = config.getServoIfEnabled("HSR", HardwareConfig.HorizontalSlideConfig.ENABLE_RIGHT);

        // Initialize vertical claw servos
        vertClawRotateLeft = config.getServoIfEnabled("VCRL", HardwareConfig.ClawConfig.ENABLE_VERT_ROTATE_LEFT);
        vertClawRotateRight = config.getServoIfEnabled("VCRR", HardwareConfig.ClawConfig.ENABLE_VERT_ROTATE_RIGHT);
        vertClawGripper = config.getServoIfEnabled("VCG", HardwareConfig.ClawConfig.ENABLE_VERT_GRIPPER);

        // Initialize horizontal claw servos
        horizClawRotateLeft = config.getServoIfEnabled("HCRL", HardwareConfig.ClawConfig.ENABLE_HORIZ_ROTATE_LEFT);
        horizClawRotateRight = config.getServoIfEnabled("HCRR", HardwareConfig.ClawConfig.ENABLE_HORIZ_ROTATE_RIGHT);
        horizClawGripper = config.getServoIfEnabled("HCG", HardwareConfig.ClawConfig.ENABLE_HORIZ_GRIPPER);

        // Color sensor
        horizClawColor = config.getColorSensorIfEnabled("HCC", HardwareConfig.SensorConfig.ENABLE_BOTTOM_COLOR);

        // Reset claw rotation limits
        isClawAtLeftLimit = false;
        isClawAtRightLimit = false;

        // Initialize controller classes
        if (armMotorLeft != null && armMotorRight != null) {
            controller = new ArmEncoder(armMotorLeft, armMotorRight);
        }
        drive = new SampleMecanumDrive(hardwareMap);
        gamepadCalc = new GamepadCalc(gamepad1, gamepad2, this);

        // Add gamepad status to telemetry
//        telemetry.addLine("=== Gamepad Setup Instructions ===");
//        telemetry.addLine("Gamepad 1 (Driver): Press OPTIONS + SQUARE (□)");
//        telemetry.addLine("Gamepad 2 (Operator): Press OPTIONS + CIRCLE (○)");
        telemetry.addLine("=== Gamepad Status ===");
        telemetry.addData("Status", gamepadCalc.getStatus());
        telemetry.addData("System Health", gamepadCalc.isGamepadSystemHealthy() ? "OK" : "ERROR");
        telemetry.update();
    }

    private void setupMotors() {
        // Drive motors setup
        if (leftMotor != null && rightMotor != null && leftMotorBack != null && rightMotorBack != null) {

            // First reset encoders
            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Set to RUN_WITHOUT_ENCODER since we're using external odometry
            leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftMotorBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightMotorBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // Set motor directions for mecanum drive
            leftMotor.setDirection(DcMotor.Direction.REVERSE);
            rightMotor.setDirection(DcMotor.Direction.FORWARD);
            leftMotorBack.setDirection(DcMotor.Direction.REVERSE);
            rightMotorBack.setDirection(DcMotor.Direction.FORWARD);

            // Set zero power behavior
            leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Vertical slide motors setup
        if (vertSlideLeft != null && vertSlideRight != null) {
            vertSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            vertSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            vertSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            vertSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            vertSlideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            vertSlideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Set opposite directions for parallel-mounted motors
            vertSlideLeft.setDirection(DcMotor.Direction.REVERSE);
            vertSlideRight.setDirection(DcMotor.Direction.FORWARD);
        }
    }
    private void handleDrive(SampleMecanumDrive drive) {
//        Pose2d poseEstimate = drive.getPoseEstimate();

        // Get gamepad inputs
        double y = -gamepadCalc.getGamepad1().left_stick_y;  // Forward/back
        double x = -gamepadCalc.getGamepad1().left_stick_x;   // Strafe
        double rx = -gamepadCalc.getGamepad1().right_stick_x; // Turn

        // Add telemetry for motor states
//        telemetry.addData("FL Mode", leftMotor.getMode());
//        telemetry.addData("FR Mode", rightMotor.getMode());
//        telemetry.addData("BL Mode", leftMotorBack.getMode());
//        telemetry.addData("BR Mode", rightMotorBack.getMode());

        // Test mode using D-pad
//        if (gamepadCalc.getGamepad1().dpad_up) {
//            // Test Front Left motor only
//            leftMotor.setPower(0.3);
//            rightMotor.setPower(0);
//            leftMotorBack.setPower(0);
//            rightMotorBack.setPower(0);
//            telemetry.addData("Testing", "Front Left Motor (FL)");
//        }
//        else if (gamepadCalc.getGamepad1().dpad_right) {
//            // Test Front Right motor only
//            leftMotor.setPower(0);
//            rightMotor.setPower(0.3);
//            leftMotorBack.setPower(0);
//            rightMotorBack.setPower(0);
//            telemetry.addData("Testing", "Front Right Motor (FR)");
//        }
//        else if (gamepadCalc.getGamepad1().dpad_down) {
//            // Test Back Right motor only
//            leftMotor.setPower(0);
//            rightMotor.setPower(0);
//            leftMotorBack.setPower(0);
//            rightMotorBack.setPower(0.3);
//            telemetry.addData("Testing", "Back Right Motor (BR)");
//        }
//        else if (gamepadCalc.getGamepad1().dpad_left) {
//            // Test Back Left motor only
//            leftMotor.setPower(0);
//            rightMotor.setPower(0);
//            leftMotorBack.setPower(0.3);
//            rightMotorBack.setPower(0);
//            telemetry.addData("Testing", "Back Left Motor (BL)");
//        }
//        else {
//            // Stop all motors if no D-pad pressed
//            leftMotor.setPower(0);
//            rightMotor.setPower(0);
//            leftMotorBack.setPower(0);
//            rightMotorBack.setPower(0);
//            telemetry.addData("Testing", "No motor - Press D-pad to test");
//        }

//        // Add motor power telemetry
//        telemetry.addData("Front Left Power", leftMotor.getPower());
//        telemetry.addData("Front Right Power", rightMotor.getPower());
//        telemetry.addData("Back Left Power", leftMotorBack.getPower());
//        telemetry.addData("Back Right Power", rightMotorBack.getPower());
//        telemetry.update();
//
//        telemetry.addData("FL Power", leftMotor.getPower());
//        telemetry.addData("FR Power", rightMotor.getPower());
//        telemetry.addData("BL Power", leftMotorBack.getPower());
//        telemetry.addData("BR Power", rightMotorBack.getPower());
//
//        telemetry.addData("Stick Y", y);
//        telemetry.addData("Stick X", x);
//        telemetry.addData("Rotation", rx);
//        telemetry.update();

        // Define deadzones
        double STRAFE_DEADZONE = 0.3;  // Horizontal deadzone
        double VERTICAL_DEADZONE = 0.15;  // Vertical deadzone (smaller since forward/back is primary movement)
        double ROTATION_DEADZONE = 0.2;  // Rotation deadzone

        // If vertical movement is more significant than horizontal, ignore small horizontal movements
        if (Math.abs(y) > Math.abs(x) && Math.abs(x) < STRAFE_DEADZONE) {
            x = 0; // Ignore small horizontal input when moving primarily forward/backward
        }

        // If horizontal movement is more significant than vertical, ignore small vertical movements
        if (Math.abs(x) > Math.abs(y) && Math.abs(y) < VERTICAL_DEADZONE) {
            y = 0; // Ignore small horizontal input when moving primarily forward/backward
        }

        if (Math.abs(rx) < ROTATION_DEADZONE) {
            rx = 0;
        }

        // Create input vector from gamepad values
        // Note the different input mapping:
        // left_stick_y controls forward/backward (negative because pushing forward gives negative values)
        // left_stick_x controls strafing (negative because right should be positive)
        Vector2d input = new Vector2d(
                y,
                x
        );

        // Rotate the input vector by the negative of the robot's heading
        // This makes the controls field-centric
//        input = input.rotated(-poseEstimate.getHeading());

        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),   // Forward/back
                        input.getY(),   // Left/right strafe
                        rx              // Rotation (negative for correct direction)
                )
        );

//        double y = -gamepadCalc.getGamepad1().left_stick_y;  // Forward/back
//        double x = gamepadCalc.getGamepad1().left_stick_x;   // Strafe
//        double rx = -gamepadCalc.getGamepad1().right_stick_x; // Turn
//
//        // Calculate powers
//        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//        double frontLeftPower = (y + x + rx) / denominator;
//        double backLeftPower = (y - x + rx) / denominator;
//        double frontRightPower = (y - x - rx) / denominator;
//        double backRightPower = (y + x - rx) / denominator;

//        // Add detailed telemetry for debugging
//        telemetry.addData("Front Left Power", "%.2f", frontLeftPower);
//        telemetry.addData("Front Right Power", "%.2f", frontRightPower);
//        telemetry.addData("Back Left Power", "%.2f", backLeftPower);
//        telemetry.addData("Back Right Power", "%.2f", backRightPower);
//        telemetry.addData("Stick Y", "%.2f", y);
//        telemetry.addData("Stick X", "%.2f", x);
//        telemetry.addData("Rotation", "%.2f", rx);
//        telemetry.update();

//        drive.setWeightedDrivePower(new Pose2d(
//                y,
//                x,
//                rx
//        ));
    }

    private void handleVerticalSlides() {
        if (vertSlideLeft == null || vertSlideRight == null || !gamepadCalc.isGamepadSystemHealthy()) {
            telemetry.addData("Vertical Slides Error", "Required components not initialized");
            return;
        }

        // Add position tracking at the start
        updatePositionTracking();
        telemetry.addData("getCurrentPosition Left: ", vertSlideLeft.getCurrentPosition());
        telemetry.addData("getCurrentPosition Right: ", vertSlideRight.getCurrentPosition());
        telemetry.addData("Warning", "Already at maximum height! ", vertSlideLeft.getCurrentPosition());


        // Add velocity safety check
        if (Math.abs(verticalVelocity) > VELOCITY_DANGER_THRESHOLD) {
            telemetry.addData("⚠️ WARNING", "Moving too fast: %.2f", verticalVelocity);
            stopVerticalSlides();
            return;
        }

        Gamepad gamepad2 = gamepadCalc.getGamepad2();
        if (gamepad2 == null) {
            telemetry.addData("Gamepad Error", "Gamepad 2 not available");
            return;
        }

        double slidePower = -gamepad2.left_stick_y;
        telemetry.addData("Vertical Slide", "Gamepad Triangle power: ", slidePower);
        boolean isManualControl = Math.abs(slidePower) > 0.1;

//        // Add detailed telemetry for debugging
//        telemetry.addData("Vertical Slide Power", "Raw: %.2f, Last: %.2f", slidePower, lastSlidePower);
//        telemetry.addData("Vertical Slide State", vertSlideState);
//        telemetry.addData("Manual Control", isManualControl);

        // State machine for vertical slides
        switch (vertSlideState) {
            case IDLE:
                if (isManualControl) {
                    vertSlideState = VertSlideState.MANUAL_CONTROL;
                } else if (gamepad2.dpad_up && vertSlideLeft.getCurrentPosition() < VERT_SLIDE_HIGH) {
                    targetVertPosition = VERT_SLIDE_HIGH;
                    vertSlideState = VertSlideState.MOVING_TO_POSITION;
                } else if (gamepad2.dpad_right) {
                    targetVertPosition = VERT_SLIDE_MID;
                    vertSlideState = VertSlideState.MOVING_TO_POSITION;
                } else if (gamepad2.dpad_down && vertSlideLeft.getCurrentPosition() > VERT_SLIDE_LOW) {
                    targetVertPosition = VERT_SLIDE_LOW;
                    vertSlideState = VertSlideState.MOVING_TO_POSITION;
                }

                // Add telemetry for position limits
                if (gamepad2.dpad_up && vertSlideLeft.getCurrentPosition() >= VERT_SLIDE_HIGH) {
                    telemetry.addData("Warning", "Already at maximum height! ", VERT_SLIDE_HIGH);
                }
                if (gamepad2.dpad_down && vertSlideLeft.getCurrentPosition() <= VERT_SLIDE_LOW) {
                    telemetry.addData("Warning", "Already at minimum height! ", VERT_SLIDE_LOW);
                }

                break;

//            case MANUAL_CONTROL:
//                if (!isManualControl) {
//                    vertSlideState = VertSlideState.IDLE;
//                    stopVerticalSlides();
//                } else {
//                    // Smooth the power change
//                    double smoothedPower = lastSlidePower + (slidePower - lastSlidePower) * currentPowerSmoothing;
//                    // Apply safety limits
//                    double safePower = getVerticalSlidePower(smoothedPower);
//                    // Limit the maximum power
//                    safePower = Range.clip(smoothedPower, -currentSlideMaxPower, currentSlideMaxPower);
//                    telemetry.addData("smoothedPower: ", smoothedPower);
//                    telemetry.addData("safePower: ", safePower);
//                    telemetry.addData("slidedPower: ", slidePower);
//
//                    vertSlideLeft.setPower(safePower);
//                    vertSlideRight.setPower(safePower);
//
//                    lastSlidePower = safePower;
//
//                    // Add power telemetry
//                    telemetry.addData("Applied Power", "Smoothed: %.2f, Safe: %.2f", smoothedPower, safePower);
//                }
//                break;

            // Update the MANUAL_CONTROL case in your handleVerticalSlides method
            case MANUAL_CONTROL:
                if (!isManualControl) {
                    vertSlideState = VertSlideState.IDLE;
                    stopVerticalSlides();
                } else {
                    // Reduce smoothing for more responsive control
                    double smoothedPower = lastSlidePower + (slidePower - lastSlidePower) * POWER_SMOOTHING;

                    // Apply safety limits
                    double safePower = getVerticalSlidePower(smoothedPower);

                    // Use full power range
                    safePower = Range.clip(smoothedPower, -MAX_SLIDE_POWER, MAX_SLIDE_POWER);

                    vertSlideLeft.setPower(safePower);
                    vertSlideRight.setPower(safePower);

                    lastSlidePower = safePower;

                    telemetry.addData("smoothedPower", smoothedPower);
                    telemetry.addData("safePower", safePower);
                    telemetry.addData("slidePower", slidePower);
                }
                break;

//            case MOVING_TO_POSITION:
//                if (isManualControl) {
//                    vertSlideState = VertSlideState.MANUAL_CONTROL;
//                } else {
//                    moveVerticalSlidesToPosition(targetVertPosition);
//                    if (isAtVerticalTarget()) {
//                        vertSlideState = VertSlideState.IDLE;
//                        stopVerticalSlides();
//                    }
//                }
//                break;

            // Update the MOVING_TO_POSITION case to use velocity-based movement
            case MOVING_TO_POSITION:
                if (isManualControl) {
                    vertSlideState = VertSlideState.MANUAL_CONTROL;
                } else {
                    // Use velocity-based movement instead of position-based
                    // Adjust the max velocity value based on your specific needs (ticks/second)
                    runToPositionWithVelocity(targetVertPosition, 1000);

                    if (isAtVerticalTarget()) {
                        vertSlideState = VertSlideState.IDLE;
                        stopVerticalSlides();
                        // Reset motor mode after reaching position
                        vertSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        vertSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }
                }
                break;

            case ERROR:
                // Handle error state if needed
                telemetry.addData("Vertical Slides", "ERROR STATE");
                break;
        }

        // Add position telemetry
        telemetry.addData("Left Slide Position", vertSlideLeft.getCurrentPosition());
        telemetry.addData("Right Slide Position", vertSlideRight.getCurrentPosition());
        telemetry.addData("Left gamepad2 x", gamepad2.x);
//        telemetry.addData("Left Trigger Value", gamepad2.left_trigger);

        telemetry.addData("Left Position", vertSlideLeft.getCurrentPosition());
        telemetry.addData("Right Position", vertSlideRight.getCurrentPosition());
    }

    // Run to position method for velocity-based movement (add this method)
    private void runToPositionWithVelocity(int targetPosition, double maxVelocity) {
        // Make sure you're using the correct motor mode
        vertSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        vertSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set target position
        vertSlideLeft.setTargetPosition(targetPosition);
        vertSlideRight.setTargetPosition(targetPosition);

        // Switch to RUN_TO_POSITION mode
        vertSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vertSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set velocity in ticks per second (adjust based on your motor specs)
        // For Yellow Jacket motors with 19.2:1 gear ratio, max velocity is around 1150 ticks/second
        vertSlideLeft.setVelocity(maxVelocity);
        vertSlideRight.setVelocity(maxVelocity);

        // Set power to ensure motor runs at requested velocity
        vertSlideLeft.setPower(1.0);
        vertSlideRight.setPower(1.0);
    }

    private void handleHorizontalSlides() {
        if (horizSlideLeft == null || horizSlideRight == null) return;

        Gamepad gamepad2 = gamepadCalc.getGamepad2();
        if (gamepad2 == null) {
            telemetry.addData("Gamepad Error", "Gamepad 2 not available");
            return;
        }

        // Add deadzone for right stick X
        double HORIZ_DEADZONE = 0.1;
        double stickX = gamepad2.right_stick_x;

        // Use right stick X for horizontal movement
        if (Math.abs(stickX) > HORIZ_DEADZONE) {
            // Convert stick input to servo power
            // Map from [-1,1] to [0,1] for servo power
            double servoPower = 0.5 + (stickX * 0.5); // This maps -1 to 0, 0 to 0.5, and 1 to 1

//            // Calculate new position with increment (for min max servo)
//            currentHorizPosition += stickX * HORIZ_SLIDE_INCREMENT;
//
//            // Ensure position stays within bounds
//            currentHorizPosition = Range.clip(currentHorizPosition, HORIZ_SLIDE_MIN, HORIZ_SLIDE_MAX);
//
//            // Set servo positions
//            horizSlideLeft.setPosition(currentHorizPosition);
//            horizSlideRight.setPosition(1 - currentHorizPosition); // Reverse for opposite side

            // Set servo powers (one needs to be reversed)
            servoPower = Range.clip(servoPower,0,1);
            horizSlideLeft.setPosition(servoPower);
            horizSlideRight.setPosition(1.0 - servoPower); // Reverse direction for opposite side

            telemetry.addData("Servo Powers", "Left: %.2f, Right: %.2f", servoPower, 1.0 - servoPower);


//
//            // Add detailed telemetry
//            telemetry.addData("Stick X Input", String.format("%.2f", stickX));
//            telemetry.addData("Current Position", String.format("%.2f", currentorizPosition));
//            telemetry.addData("Left Servo", String.format("%.2f", horizSlideLHeft.getPosition()));
//            telemetry.addData("Right Servo", String.format("%.2f", horizSlideRight.getPosition()));
        }
//        else {
//            // Stop the servos
//            horizSlideLeft.setPosition(0.5);
//            horizSlideRight.setPosition(0.5);
//        }
    }

    private void handleVerticalClaw() {
        if (vertClawRotateLeft == null || vertClawRotateRight == null || vertClawGripper == null || !gamepadCalc.isGamepadSystemHealthy()) {
            telemetry.addData("Vertical Claw Error", "Required components not initialized");
            return;
        }

        Gamepad gamepad2 = gamepadCalc.getGamepad2();
        if (gamepad2 == null) {
            telemetry.addData("Gamepad Error", "Gamepad 2 not available");
            return;
        }

        // Add telemetry for current state and positions
        telemetry.addData("Vertical Claw State", vertClawState);
        telemetry.addData("Rotate Left Position", vertClawRotateLeft.getPosition());
        telemetry.addData("Rotate Right Position", vertClawRotateRight.getPosition());
        telemetry.addData("Gripper Position", vertClawGripper.getPosition());

        switch (vertClawState) {
            case INIT:
                // Set initial positions
                vertClawRotateLeft.setPosition(VERT_CLAW_PARALLEL);
                vertClawGripper.setPosition(CLAW_CLOSED);
                telemetry.addData("Vertical Claw", "Initializing...");
                // Move to IDLE after initialization
                vertClawState = VertClawState.IDLE;
                break;

            case IDLE:
                // Set default rotation position
//                vertClawRotateLeft.setPosition(VERT_CLAW_PARALLEL);
//                vertClawRotateRight.setPosition(1 - VERT_CLAW_PARALLEL);

                // Handle rotation controls
                if (gamepad2.left_bumper) {
                    vertClawRotateLeft.setPosition(VERT_CLAW_PARALLEL);
                    // vertClawRotateRight.setPosition(1 - VERT_CLAW_PARALLEL);
                    telemetry.addData("Rotate Action: ", "Parallel");
                } else if (gamepad2.left_trigger > 0.5) {
                    vertClawRotateLeft.setPosition(VERT_CLAW_ROTATED);
                    // vertClawRotateRight.setPosition(1 - VERT_CLAW_ROTATED);
                    telemetry.addData("Rotate Action: ", "Rotated");
                }

                // Handle gripper state changes
                if (gamepad2.x) {
                    vertClawState = VertClawState.CLOSE;
                } else if (gamepad2.b) {
                    vertClawState = VertClawState.OPEN;
                }
                break;

            case OPEN:
                vertClawGripper.setPosition(CLAW_OPEN);
                telemetry.addData("Gripper Action", "Opening");
                vertClawState = VertClawState.IDLE;  // Return to IDLE after setting position
                break;

            case CLOSE:
                vertClawGripper.setPosition(CLAW_CLOSED);
                telemetry.addData("Gripper Action", "Closing");
                vertClawState = VertClawState.IDLE;  // Return to IDLE after setting position
                break;

            case TRANSFER:
                vertClawRotateLeft.setPosition(VERT_CLAW_PARALLEL);
                vertClawGripper.setPosition(CLAW_OPEN);
                telemetry.addData("Vertical Claw", "Transfer Position");
                if (Math.abs(vertClawRotateLeft.getPosition() - VERT_CLAW_PARALLEL) < 0.05) {
                    vertClawState = VertClawState.IDLE;
                }
                break;

            case HOME:
                // Check if slides are in safe position first
                if (vertSlideLeft.getCurrentPosition() > VERT_SLIDE_LOW + 50) {
                    telemetry.addData("Home Sequence", "Waiting for slides");
                    break;
                }

                vertClawRotateLeft.setPosition(VERT_CLAW_PARALLEL);
                vertClawGripper.setPosition(CLAW_CLOSED);
                telemetry.addData("Home Sequence", "Setting home position");

                // Check if everything is in position
                if (Math.abs(vertClawRotateLeft.getPosition() - VERT_CLAW_PARALLEL) < 0.05 &&
                        Math.abs(vertClawGripper.getPosition() - CLAW_CLOSED) < 0.05) {
                    telemetry.addData("Home Sequence", "Complete");
                    vertClawState = VertClawState.IDLE;
                }
                break;

            case ERROR:
                telemetry.addData("Vertical Claw", "ERROR STATE");
                // Add error recovery logic if needed
                break;
        }
    }

    private void handleVerticalClawContinuous() {
        if (vertClawRotateLeft == null || vertClawRotateRight == null ||
                vertClawGripper == null || !gamepadCalc.isGamepadSystemHealthy()) {
            telemetry.addData("Vertical Claw Error", "Required components not initialized");
            return;
        }

        Gamepad gamepad2 = gamepadCalc.getGamepad2();
        if (gamepad2 == null) {
            telemetry.addData("Gamepad Error", "Gamepad 2 not available");
            return;
        }

        // Constants for continuous servo control
        final double SERVO_STOP = 0.5;
        final double HOLDING_POWER = 0.05;
        final double ROTATE_SPEED_LEFT = 0.2;   // Speed for left rotation
        final double ROTATE_SPEED_RIGHT = 0.2;  // Reduced speed for right rotation

        // Rotation control with limits
        if (gamepad2.left_bumper && !isClawAtLeftLimit) {
            // Left rotation only if not at left limit
            vertClawRotateLeft.setPosition(SERVO_STOP - ROTATE_SPEED_LEFT);
            vertClawRotateRight.setPosition(SERVO_STOP + ROTATE_SPEED_LEFT);
            isClawAtRightLimit = false;  // Reset right limit when moving left
            telemetry.addData("Rotate Action", "Left");
        }
        else if (gamepad2.left_trigger > 0.2 && !isClawAtRightLimit) {
            // Right rotation only if not at right limit
            vertClawRotateLeft.setPosition(SERVO_STOP + ROTATE_SPEED_RIGHT);
            vertClawRotateRight.setPosition(SERVO_STOP - ROTATE_SPEED_RIGHT);
            isClawAtLeftLimit = false;  // Reset left limit when moving right
            telemetry.addData("Rotate Action", "Right");
        }
        else {
            vertClawRotateLeft.setPosition(SERVO_STOP);
            vertClawRotateRight.setPosition(SERVO_STOP);
            telemetry.addData("Rotate Action", "Stopped");
        }

//        // Rotation control using continuous servo mode
//        switch (rotationPosition) {
//            case LEFT_LIMIT:
//                if (gamepad2.left_trigger > 0.3) {
//                    vertClawRotateLeft.setPosition(SERVO_STOP + ROTATE_SPEED_RIGHT);
//                    vertClawRotateRight.setPosition(SERVO_STOP - ROTATE_SPEED_RIGHT);
////                    rotationPosition = RotationPosition.CENTER;
//                } else {
//                    // Apply holding power to maintain position
//                    vertClawRotateLeft.setPosition(SERVO_STOP + HOLDING_POWER);
//                    vertClawRotateRight.setPosition(SERVO_STOP - HOLDING_POWER);
//                }
//                break;
//
//            case RIGHT_LIMIT:
//                if (gamepad2.left_bumper) {
//                    vertClawRotateLeft.setPosition(SERVO_STOP - ROTATE_SPEED_LEFT);
//                    vertClawRotateRight.setPosition(SERVO_STOP + ROTATE_SPEED_LEFT);
////                    rotationPosition = RotationPosition.CENTER;
//                } else {
//                    // Apply holding power to maintain position
//                    vertClawRotateLeft.setPosition(SERVO_STOP - HOLDING_POWER);
//                    vertClawRotateRight.setPosition(SERVO_STOP + HOLDING_POWER);
//                }
//                break;
//
//            case CENTER:
//                if (gamepad2.left_bumper) {
//                    vertClawRotateLeft.setPosition(SERVO_STOP - ROTATE_SPEED_LEFT);
//                    vertClawRotateRight.setPosition(SERVO_STOP + ROTATE_SPEED_LEFT);
//                    rotationPosition = RotationPosition.LEFT_LIMIT;
//                } else if (gamepad2.left_trigger > 0.3) {
//                    vertClawRotateLeft.setPosition(SERVO_STOP + ROTATE_SPEED_RIGHT);
//                    vertClawRotateRight.setPosition(SERVO_STOP - ROTATE_SPEED_RIGHT);
//                    rotationPosition = RotationPosition.RIGHT_LIMIT;
//                } else {
//                    // At center, no holding power needed
//                    vertClawRotateLeft.setPosition(SERVO_STOP);
//                    vertClawRotateRight.setPosition(SERVO_STOP);
//                }
//                break;
//        }
//
//        if (gamepad2.dpad_left) {
//            isClawAtLeftLimit = false;
//            isClawAtRightLimit = false;
//            telemetry.addData("Limits", "Reset");
//        }

//        // Add test trigger - press Y to start test
//        if (gamepad2.y) {
//            runServoTest();
//            return;  // Skip normal operation during test
//        }

        // Gripper control remains position-based (non-continuous)
        if (gamepad2.x) {
            vertClawGripper.setPosition(CLAW_CLOSED);
            telemetry.addData("Gripper", "Closed");
        } else if (gamepad2.b) {
            vertClawGripper.setPosition(CLAW_OPEN);
            telemetry.addData("Gripper", "Open");
        }

//        // Add telemetry
//        telemetry.addData("Rotate Left Position", vertClawRotateLeft.getPosition());
//        // telemetry.addData("Rotate Right Position", vertClawRotateRight.getPosition());
//        telemetry.addData("Gripper Position", vertClawGripper.getPosition());

        // Add these inside handleVerticalClawContinuous()
//        telemetry.addData("Left Servo Position", vertClawRotateLeft.getPosition());
//        telemetry.addData("Right Servo Position", vertClawRotateRight.getPosition());
//        telemetry.addData("Left Bumper Pressed", gamepad2.left_bumper);
//        telemetry.addData("Left Trigger Value", gamepad2.left_trigger);
    }


    private void runServoTest() {
        ElapsedTime timer = new ElapsedTime();
        double testStartTime = timer.seconds();

        while (timer.seconds() - testStartTime < 15) {
            double currentTime = timer.seconds() - testStartTime;

            if (currentTime < 3) {  // Stop position
                vertClawRotateLeft.setPosition(0.5);
                vertClawRotateRight.setPosition(0.5);
                telemetry.addData("Step", "Stop position");
            }
            else if (currentTime < 6) {  // Direction 1 - REVERSED RIGHT SERVO
                vertClawRotateLeft.setPosition(0.0);
                vertClawRotateRight.setPosition(1.0);  // Changed from 1.0 to 0.0
                telemetry.addData("Step", "Modified direction 1");
            }
            else if (currentTime < 9) {  // Stop
                vertClawRotateLeft.setPosition(0.5);
                vertClawRotateRight.setPosition(0.5);
                telemetry.addData("Step", "Stop after direction 1");
            }
            else if (currentTime < 12) {  // Direction 2 (already working)
                vertClawRotateLeft.setPosition(1.0);
                vertClawRotateRight.setPosition(0.0);
                telemetry.addData("Step", "Direction 2");
            }
            else {  // Stop
                vertClawRotateLeft.setPosition(0.5);
                vertClawRotateRight.setPosition(0.5);
                telemetry.addData("Step", "Final stop");
            }

            telemetry.addData("Left Servo", vertClawRotateLeft.getPosition());
            telemetry.addData("Right Servo", vertClawRotateRight.getPosition());
            telemetry.addData("Time", String.format("%.1f", currentTime));
            telemetry.update();
        }
    }
    private void testServoSync() {
        // Test 1: Check same distance from center
        vertClawRotateLeft.setPosition(0.7);   // 0.2 above center
        vertClawRotateRight.setPosition(0.3);  // 0.2 below center
        telemetry.addData("Test 1", "Servos should move equally in opposite directions");
        telemetry.addData("Left Pos", vertClawRotateLeft.getPosition());
        telemetry.addData("Right Pos", vertClawRotateRight.getPosition());

        // Wait a moment
        sleep(2000);

        // Test 2: Return to center
        vertClawRotateLeft.setPosition(0.5);
        vertClawRotateRight.setPosition(0.5);
        telemetry.addData("Test 2", "Servos should return to center");
        telemetry.update();

        sleep(2000);
    }

    private void handleHorizontalClaw() {
        if (horizClawRotateLeft == null || horizClawRotateRight == null || horizClawGripper == null) return;

        // Rotation control (right bumper/trigger)
        if (gamepad2.right_bumper) {
            horizClawRotateLeft.setPosition(HORIZ_CLAW_PARALLEL);
            horizClawRotateRight.setPosition(1 - HORIZ_CLAW_PARALLEL);
        } else if (gamepad2.right_trigger > 0.5) {
            horizClawRotateLeft.setPosition(HORIZ_CLAW_ROTATED);
            horizClawRotateRight.setPosition(1 - HORIZ_CLAW_ROTATED);
        }

        // Gripper control (B button)
        if (gamepad2.y) {
            horizClawGripper.setPosition(HCLAW_CLOSED);
        } else if (gamepad2.a) {
            horizClawGripper.setPosition(HCLAW_OPEN);
        }
    }

//    private void moveVerticalSlidesToPosition(int targetPosition) {
//        // Safety bounds check
//        targetPosition = Range.clip(targetPosition, VERT_SLIDE_MIN, VERT_SLIDE_MAX);
//
//        int currentPositionLeft = vertSlideLeft.getCurrentPosition();
//        int currentPositionRight = vertSlideRight.getCurrentPosition();
//        int currentPosition = (vertSlideLeft.getCurrentPosition() + vertSlideRight.getCurrentPosition()) / 2;
//        double error = targetPosition - currentPosition;
//        double deltaTime = vertSlidePIDTimer.seconds();
//
//        // PID calculation
//        vertIntegralSum += error * deltaTime;
//        double derivative = (error - lastVertError) / deltaTime;
//
//        double power = (error * SLIDES_P) + (vertIntegralSum * SLIDES_I) + (derivative * SLIDES_D);
//        power = Range.clip(power, -1.0, 1.0);
//
//        // Apply power to motors
//        vertSlideLeft.setPower(power);
////        vertSlideRight.setPower(power);
//
//        // Update PID variables
//        lastVertError = error;
//        vertSlidePIDTimer.reset();
//    }

    // Modify the moveVerticalSlidesToPosition method to use higher velocity
    private void moveVerticalSlidesToPosition(int targetPosition) {
        // Calculate the difference between current and target positions
        int currentPosition = vertSlideLeft.getCurrentPosition();
        int positionError = targetPosition - currentPosition;

        // Use proportional control with a higher gain
        double kP = 0.015; // Increase this value for faster movement
        double power = kP * positionError;

        // Ensure minimum power to overcome static friction
        if (Math.abs(power) < 0.15 && Math.abs(positionError) > 10) {
            power = Math.signum(power) * 0.15;
        }

        // Apply maximum power limit but allow full speed
        power = Range.clip(power, -MAX_SLIDE_POWER, MAX_SLIDE_POWER);

        vertSlideLeft.setPower(power);
        vertSlideRight.setPower(power);

        // Add telemetry for debugging
        telemetry.addData("Target Position", targetPosition);
        telemetry.addData("Current Position", currentPosition);
        telemetry.addData("Error", positionError);
        telemetry.addData("Power", power);
    }

    private boolean isAtVerticalTarget() {
        int currentPosition = (vertSlideLeft.getCurrentPosition() +
                vertSlideRight.getCurrentPosition()) / 2;
        return Math.abs(currentPosition - targetVertPosition) < 10;
    }

    private void stopVerticalSlides() {
        if (vertSlideLeft != null && vertSlideRight != null) {
            vertSlideLeft.setPower(0);
            vertSlideRight.setPower(0);
        }
    }

    private double getVerticalSlidePower(double rawPower) {
        // Add position-based safety limits
        int currentPos = (vertSlideLeft.getCurrentPosition() + vertSlideRight.getCurrentPosition()) / 2;
        telemetry.addData("CurrentPos: ", currentPos);
        telemetry.addData("rawPower: ", rawPower);

        // Near bottom limit
        if (currentPos < VERT_SLIDE_LOW + VERT_SLIDE_SAFETY_MARGIN && rawPower < 0) {
            rawPower *= 0.5; // Half power when near bottom
            telemetry.addData("Power Limit", "Near bottom");
        }

        // Near top limit
        if (currentPos > VERT_SLIDE_HIGH - VERT_SLIDE_SAFETY_MARGIN && rawPower > 0) {
            rawPower *= 0.5; // Half power when near top
            telemetry.addData("Power Limit", "Near top");
        }

        return rawPower;
    }

    private void coordinatedMove(int slideTarget, double rotateTarget, double gripperTarget) {
        if (!checkSystemHealth()) {
            telemetry.addData("Coordinated Move", "Failed - System health check failed");
            return;
        }

        // Start all movements
        setSpeedProfile(slideTarget);  // Set appropriate speed profile based on target height
        vertSlideState = VertSlideState.MOVING_TO_POSITION;
        targetVertPosition = slideTarget;
        vertClawRotateLeft.setPosition(rotateTarget);
        vertClawRotateRight.setPosition(rotateTarget);
        vertClawGripper.setPosition(gripperTarget);

        ElapsedTime moveTimeout = new ElapsedTime();
        moveTimeout.reset();

        // Wait for completion with timeout
        while (opModeIsActive() && !isAtVerticalTarget() && moveTimeout.seconds() < 2.0) {
            // Keep updating position tracking
            updatePositionTracking();

            // Update telemetry
            telemetry.addLine("=== Coordinated Move ===");
            telemetry.addData("Target Height", slideTarget);
            telemetry.addData("Current Height", currentVerticalPosition);
            telemetry.addData("Time Elapsed", "%.1f seconds", moveTimeout.seconds());
            telemetry.update();

            // Safety checks
            if (Math.abs(verticalVelocity) > VELOCITY_DANGER_THRESHOLD) {
                telemetry.addData("⚠️ WARNING", "Moving too fast: %.2f", verticalVelocity);
                stopVerticalSlides();
                return;
            }

            // Check for emergency stop
            if (gamepadCalc.getGamepad1().back && gamepadCalc.getGamepad2().back) {
                emergencyStop();
                return;
            }

            // Keep system running
            gamepadCalc.calculate();
            drive.update();
        }

        // Check if move completed successfully
        if (isAtVerticalTarget()) {
            telemetry.addData("Coordinated Move", "Complete");
        } else {
            telemetry.addData("Coordinated Move", "Timeout - Movement incomplete");
        }
    }

    private void executeCycleTransfer() {
        // Safety check
        if (!checkSystemHealth()) return;

        // Ground pickup configuration
        coordinatedMove(
                VERT_SLIDE_LOW,      // Move slides down
                VERT_CLAW_PARALLEL,  // Rotate claw parallel
                CLAW_OPEN           // Open claw for pickup
        );
        sleep(500);  // Short delay for stability

        // Grasp object
        coordinatedMove(
                VERT_SLIDE_LOW,      // Keep slides down
                VERT_CLAW_PARALLEL,  // Keep claw parallel
                CLAW_CLOSED         // Close claw on object
        );
        sleep(300);  // Short delay for grip

        // Move to high position and place
        coordinatedMove(
                VERT_SLIDE_HIGH,     // Raise slides
                VERT_CLAW_ROTATED,   // Rotate claw
                CLAW_CLOSED         // Keep grip
        );
        sleep(500);  // Short delay for stability

        // Release object
        coordinatedMove(
                VERT_SLIDE_HIGH,     // Keep slides up
                VERT_CLAW_ROTATED,   // Keep claw rotated
                CLAW_OPEN           // Release object
        );
    }
    private double calculatePID(double reference, double state, double kP, double kI, double kD,
                                ElapsedTime timer, double lastError, double integralSum) {
        double error = reference - state;
        double deltaTime = timer.seconds();
        integralSum += error * deltaTime;
        double derivative = (error - lastError) / deltaTime;

        timer.reset();

        double output = (error * kP) + (derivative * kD) + (integralSum * kI);
        return Range.clip(output, -1, 1);  // Clamp output between -1 and 1
    }

    private void applySlidePower(double power) {
        // Single place where power is applied to slides
        armMotorLeft.setPower(power);
        armMotorRight.setPower(power);
    }

    private boolean isAtTargetPosition() {
        int currentPosition = (armMotorLeft.getCurrentPosition() +
                armMotorRight.getCurrentPosition()) / 2;
        return Math.abs(currentPosition - targetSlidePosition) < 10;
    }

    private void moveToTargetPosition() {
        int currentPosition = (armMotorLeft.getCurrentPosition() +
                armMotorRight.getCurrentPosition()) / 2;
        double power = calculatePID(targetSlidePosition, currentPosition,
                SLIDES_P, SLIDES_I, SLIDES_D,
                slidesTimer, lastSlidesError, slidesIntegralSum);
        applySlidePower(Range.clip(power, -SLIDES_MAX_POWER, SLIDES_MAX_POWER));
    }

    private void stopRotation() {
        rotateMotorLeft.setPower(0);
        rotateMotorRight.setPower(0);
    }

    private void applyRotationPower(double power) {
        rotateMotorLeft.setPower(power);
        rotateMotorRight.setPower(power);
    }

    private double getSmoothedRotationPower(double rawPower) {
        double safePower = Range.clip(rawPower, -ROTATION_MAX_POWER, ROTATION_MAX_POWER);
        if (Math.abs(safePower - lastRotationPower) > 0.3) {
            safePower = (safePower + lastRotationPower) / 2;
        }
        lastRotationPower = safePower;
        return safePower;
    }

    private boolean isAtRotationTarget() {
        int currentPosition = (rotateMotorLeft.getCurrentPosition() +
                rotateMotorRight.getCurrentPosition()) / 2;
        return Math.abs(currentPosition - targetRotationPosition) < 10;
    }

    private void updatePositionTracking() {
        lastVerticalPosition = currentVerticalPosition;
        currentVerticalPosition = (vertSlideLeft.getCurrentPosition() +
                vertSlideRight.getCurrentPosition()) / 2;

        double deltaTime = runtime.seconds();
        if (deltaTime > 0) {  // Prevent division by zero
            verticalVelocity = (currentVerticalPosition - lastVerticalPosition) / deltaTime;
        }
        runtime.reset();  // Reset timer for next calculation

//        // Add tracking telemetry
//        telemetry.addData("Vertical Position", currentVerticalPosition);
//        telemetry.addData("Vertical Velocity", "%.2f ticks/sec", verticalVelocity);
    }

    private boolean checkSystemHealth() {
        boolean isHealthy = true;

        // Check motor encoders
        if (Math.abs(vertSlideLeft.getCurrentPosition() -
                vertSlideRight.getCurrentPosition()) > 100) {
            telemetry.addData("⚠️ Warning", "Vertical slides desynchronized");
            isHealthy = false;
        }

        // Check servos
        if (Math.abs(horizSlideLeft.getPosition() -
                (1 - horizSlideRight.getPosition())) > 0.1) {
            telemetry.addData("⚠️ Warning", "Horizontal slides desynchronized");
            isHealthy = false;
        }

        return isHealthy;
    }

    private void executeHomeSequence() {
        // Safety check
        if (!checkSystemHealth()) return;

        // Move slides to safe position first
        vertSlideState = VertSlideState.MOVING_TO_POSITION;
        targetVertPosition = VERT_SLIDE_LOW;

        // Set claws to safe position
        vertClawRotateLeft.setPosition(VERT_CLAW_PARALLEL);
        vertClawGripper.setPosition(CLAW_CLOSED);

        // Return horizontal slides
        horizSlideLeft.setPosition(HORIZ_SLIDE_MIN);
        horizSlideRight.setPosition(1 - HORIZ_SLIDE_MIN);

        // Reset claw states
        isClawAtLeftLimit = false;
        isClawAtRightLimit = false;

        telemetry.addData("Sequence", "Homing...");
    }

    private void executeCycle() {
        // Safety check
        if (!checkSystemHealth()) return;

        // Ground pickup
        setSpeedProfile(VERT_SLIDE_LOW);
        vertSlideState = VertSlideState.MOVING_TO_POSITION;
        targetVertPosition = VERT_SLIDE_LOW;
        vertClawState = VertClawState.CLOSE;

        // Wait for pickup completion
        while (opModeIsActive() && !isAtVerticalTarget()) {
            updatePositionTracking();
            telemetry.addData("Cycle", "Moving to pickup");
            telemetry.update();
        }

        // Move to high position
        setSpeedProfile(VERT_SLIDE_HIGH);
        vertSlideState = VertSlideState.MOVING_TO_POSITION;
        targetVertPosition = VERT_SLIDE_HIGH;

        // Wait for movement completion
        while (opModeIsActive() && !isAtVerticalTarget()) {
            updatePositionTracking();
            telemetry.addData("Cycle", "Moving to place");
            telemetry.update();
        }

        // Place
        vertClawState = VertClawState.OPEN;
    }

    private void setSpeedProfile(int targetHeight) {
        if (targetHeight < 1.5*VERT_SLIDE_MID) {
            currentPowerSmoothing = PRECISION_POWER_SMOOTHING;
            currentSlideMaxPower = PRECISION_MAX_POWER;
        } else {
            currentPowerSmoothing = FAST_POWER_SMOOTHING;
            currentSlideMaxPower = FAST_MAX_POWER;
        }
    }

    private void emergencyStop() {
        // Stop all motors
        stopVerticalSlides();

        // Stop drive motors
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));

        // Return claws to safe position
        vertClawRotateLeft.setPosition(VERT_CLAW_PARALLEL);
        vertClawGripper.setPosition(CLAW_CLOSED);

        // Return slides to safe position
        horizSlideLeft.setPosition(HORIZ_SLIDE_MIN);
        horizSlideRight.setPosition(1 - HORIZ_SLIDE_MIN);

        // Reset states
        vertSlideState = VertSlideState.IDLE;
        vertClawState = VertClawState.IDLE;
        horizSlideState = HorizSlideState.IDLE;

        // Clear accumulated values
        lastSlidePower = 0;
        vertIntegralSum = 0;

        // Reset claw states
        isClawAtLeftLimit = false;
        isClawAtRightLimit = false;

        // Show emergency stop message
        telemetry.clearAll();
        telemetry.addLine("⚠️ EMERGENCY STOP ACTIVATED ⚠️");
        telemetry.addLine("Press both START buttons to resume");
        telemetry.update();

        // Optional: Wait for confirmation to resume
        while (opModeIsActive() &&
                !(gamepadCalc.getGamepad1().start && gamepadCalc.getGamepad2().start)) {
            // Keep updating gamepads while waiting
            gamepadCalc.calculate();
        }
    }

    private void updateTelemetry() {

//        // Color sensor telemetry
//        telemetry.addLine("=== Color Sensor Status ===");
//        if (horizClawColor != null) {
//            telemetry.addData("Color Sensor", "ENABLED");
//            telemetry.addData("Is Red", horizClawColor.isRed());
//            telemetry.addData("Is Blue", horizClawColor.isBlue());
//            telemetry.addData("HSV Values", horizClawColor.getColorInfo());
//        } else {
//            telemetry.addData("Color Sensor", "DISABLED in hardware config");
//        }

        if (HardwareConfig.HorizontalSlideConfig.ENABLE_LEFT) {
            telemetry.addData("Left Slide Position", armMotorLeft.getCurrentPosition());
        }
        if (HardwareConfig.HorizontalSlideConfig.ENABLE_RIGHT) {
            telemetry.addData("Right Slide Position", armMotorRight.getCurrentPosition());
        }
        if (HardwareConfig.VerticalSlideConfig.ENABLE_LEFT) {
            telemetry.addData("Left Rotation Position", rotateMotorLeft.getCurrentPosition());
        }
        if (HardwareConfig.VerticalSlideConfig.ENABLE_RIGHT) {
            telemetry.addData("Right Rotation Position", rotateMotorRight.getCurrentPosition());
        }
//        // Only show servo positions if enabled
//        if (HardwareConfig.ENABLE_LEFT_AXLE) {
//            telemetry.addData("Left Axle Position", leftAxleServo.getPosition());
//        }
//        if (HardwareConfig.ENABLE_RIGHT_AXLE) {
//            telemetry.addData("Right Axle Position", rightAxleServo.getPosition());
//        }
//        if (HardwareConfig.ENABLE_LEFT_GECKO) {
//            telemetry.addData("Left Gecko Position", leftGeckoServo.getPosition());
//        }
//        if (HardwareConfig.ENABLE_RIGHT_GECKO) {
//            telemetry.addData("Right Gecko Position", rightGeckoServo.getPosition());
//        }

        if (isOverCurrentProtected) {
            telemetry.addLine("⚠️ OVERCURRENT PROTECTION ACTIVE ⚠️");
        }

        // Drive System
        telemetry.addLine("=== Drive System ===");
        if (drive.getPoseEstimate() != null) {
            telemetry.addData("Pose Estimate", drive.getPoseEstimate());
        }

//        // Vertical Slides
//        telemetry.addLine("=== Vertical Slides ===");
//        if (vertSlideLeft != null && vertSlideRight != null) {
//            telemetry.addData("Left Position", vertSlideLeft.getCurrentPosition());
//            telemetry.addData("Right Position", vertSlideRight.getCurrentPosition());
//            telemetry.addData("State", vertSlideState);
//            if (vertSlideState == VertSlideState.MOVING_TO_POSITION) {
//                telemetry.addData("Target Position", targetVertPosition);
//            }
//        }

        // Horizontal Slides
        telemetry.addLine("=== Horizontal Slides ===");
        if (horizSlideLeft != null && horizSlideRight != null) {
            telemetry.addData("Left Position", horizSlideLeft.getPosition());
            telemetry.addData("Right Position", horizSlideRight.getPosition());
            telemetry.addData("Current Position", currentHorizPosition);
        }

//        // Vertical Claw
//        telemetry.addLine("=== Vertical Claw ===");
//        if (vertClawRotateLeft != null && vertClawRotateRight != null && vertClawGripper != null) {
//            telemetry.addData("Rotate Left", vertClawRotateLeft.getPosition());
//            telemetry.addData("Rotate Right", vertClawRotateRight.getPosition());
//            telemetry.addData("Gripper", vertClawGripper.getPosition());
//        }

//        // Horizontal Claw
//        telemetry.addLine("=== Horizontal Claw ===");
//        if (horizClawRotateLeft != null && horizClawRotateRight != null && horizClawGripper != null) {
//            telemetry.addData("Rotate Left", horizClawRotateLeft.getPosition());
//            telemetry.addData("Rotate Right", horizClawRotateRight.getPosition());
//            telemetry.addData("Gripper", horizClawGripper.getPosition());
//        }

        telemetry.update();
    }


}
