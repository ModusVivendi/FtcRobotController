package org.firstinspires.ftc.teamcode.opmode.teleop;

import org.firstinspires.ftc.teamcode.IntoDeep_SM.TeleOp.RRTeleOp_Init;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.config.HardwareConfig;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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

import static org.firstinspires.ftc.teamcode.states.VertMechanicsStates.*;
import static org.firstinspires.ftc.teamcode.states.HorizMechanicsStates.*;
import static org.firstinspires.ftc.teamcode.states.RobotStates.*;

// Add PID controller imports
import org.firstinspires.ftc.teamcode.controllers.PIDCoefficients;
import org.firstinspires.ftc.teamcode.controllers.PIDController;
import org.firstinspires.ftc.teamcode.controllers.PIDFController;

@TeleOp(name="RRTeleOpIntoDeep", group = "IntoDeep")
public class RRTeleOp extends LinearOpMode {
    // Load motor config
    private RobotConfig config;

    // Control mode flag
    private boolean usePIDControlVert = false;  // False for direct control, True for PID

    // PID Controller instances
    private PIDFController leftSlideController;
    private PIDFController rightSlideController;

    // PID coefficients for vertical slides
    private static final PIDCoefficients VERT_SLIDE_PID = new PIDCoefficients(
            0.05,  // kP
            0.005,    // kI
            0.0001  // kD - small value for velocity damping
    );

    // Feedforward constants
    private static final double kV = 0.01;  // Velocity feedforward
    private static final double kA = 0.0;   // Acceleration feedforward
    private static final double kStatic = 0.05;  // Static friction compensation

//    // Feedforward constants
//    private static final double kV = 0.01;  // Velocity feedforward
//    private static final double kA = 0.0;   // Acceleration feedforward
//    private static final double kStatic = 0.05;  // Static friction compensation

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
    private Servo horizClawRotateUpDown, horizClawRotateLeftRight, horizClawGripper;

    // Color Sensor
    private ColorSensorV3 horizClawColor;

    // GoBilda motor constants
    private static final double GOBILDA_TICKS_PER_REV = 537.7; // For 19.2:1 Yellow Jacket
    private static final double GOBILDA_MAX_RPM = 312; // Max RPM for Yellow Jacket
    private static final double WHEEL_DIAMETER_MM = 96.0;  // 96mm GoBilda Mecanum wheel
    private static final double DRIVE_GEAR_RATIO = 1.0;    // Direct drive


    // Vertical slider positions
    private static final int VERT_SLIDE_MIN = 100;
    private static final int VERT_SLIDE_MAX = 3000;
    private static final int VERT_SLIDE_LOW = 100;
    private static final int VERT_SLIDE_MID = 1000;
    private static final int VERT_SLIDE_HIGH = 1500;

    // Horizontal slider positions
    private static final double HORIZ_SLIDE_MIN = 0.03;
    private static final double HORIZ_SLIDE_MAX = 0.97;
    private static final double HORIZ_SLIDE_INCREMENT = 0.02;
    private double currentHorizPosition = HORIZ_SLIDE_MIN;

    // Constants for continuous servo control
    private static final double HORIZ_SERVO_STOP = 0.5;
    private static final double HORIZ_SERVO_SPEED = 0.5;
    private static final double HORIZ_CLAW_ROTATED = 0.2;

    // Claw rotation positions
    private boolean safetyVerticalClaw = false;
    private static final double VERT_CLAW_ARM_STOPPED = 0.5;
    private static final double VERT_CLAW_ROTATED = 0.8;
    private static final double VERT_CLAW_LEFT_ROTATION = 0.3;
    private static final double VERT_CLAW_RIGHT_ROTATION = 0.7;
    private static final double VERT_CLAW_GRIP_OPEN = 0.0;
    private static final double VERT_CLAW_GRIP_CLOSE = 1.0;
    private static final double HORIZ_CLAW_DOWN = 1.0;
    private static double currentVertPosition = 0.1;
    private boolean isClawClosed = false;
    private double currentGripPosition = VERT_CLAW_GRIP_CLOSE;
    final double VERT_GRIP_INCREMENT = 0.25;
    private static final double CLAW_STOP = 0.5;     // Servo stops at 0.5
    private static final double CLAW_CLOSE_SPEED = 0.0;  // Full speed one direction
    private static final double CLAW_OPEN_SPEED = 1.0;   // Full speed other direction
    private static final long GRIP_DURATION_MS = 500;    // Time to open/close in milliseconds
    private long lastGripChangeTime = 0;

    private VertGripperState vertGripperState = VertGripperState.INIT;
    private VertRotationState vertRotationState = VertRotationState.INIT;
    // Vertical claw gripper positions
    private static final double VERT_CLAW_GRIP_INIT = 0.5;
    private static final double CLAW_OPEN = -1;
    private static final double CLAW_CLOSED = 1;

    // Add these with your other class variables
    private boolean isHorizClawAtLeftLimit = false;
    private boolean isHorizClawAtRightLimit = false;

    private ElapsedTime rotationTimer = new ElapsedTime();
    private static final double PARTIAL_ROTATION_TIME = 0.4; // Adjust this value for desired rotation amount
    private boolean isRotating = false;

    // PID Constants for vertical slides
    private static final double SLIDES_P = 0.005;
    private static final double SLIDES_I = 0.0;
    private static final double SLIDES_D = 0.0;

    // State tracking
    // State declarations
    private VertSlideState vertSlideState = VertSlideState.IDLE;
    private HorizSlideState horizSlideState = HorizSlideState.INIT;
    private VertClawState vertClawState = VertClawState.INIT;
    private RotationPosition rotationPosition = RotationPosition.CENTER;
    private VertSlideState slideState = VertSlideState.IDLE;
    private RotationState rotationState = RotationState.IDLE;
    private IntakeState intakeState = IntakeState.IDLE;

    private static final double SERVO_STOP = 0.5;
    private int targetVertPosition = 0;
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
    private static final double PRECISION_POWER_SMOOTHING = 0.1;
    private static final double PRECISION_MAX_POWER = 0.9;
    private static final double FAST_POWER_SMOOTHING = 0.2;
    private static final double FAST_MAX_POWER = 0.6;

    private double currentPowerSmoothing = PRECISION_POWER_SMOOTHING;
    private double currentSlideMaxPower = PRECISION_MAX_POWER;

    private boolean slideHorizontalInitialized = false;

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
    private static final double POWER_SMOOTHING = 0.2;
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
    private static final double VELOCITY_DANGER_THRESHOLD = 2000; // Adjust this value

    // State machine enums
    private enum RobotSequence {
        NONE,
        PICKUP_SEQUENCE,
        PLACE_SEQUENCE,
        HOME_SEQUENCE
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

    private double currentVertClawPosition = SERVO_STOP;

    // State tracking
    private int targetSlidePosition = 0;
    private int targetRotationPosition = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        int tickAdjustment = 100;

        // Initialize hardware
        initializeHardware();

        // Configure motor behaviors
        setupMotors();

        // Initialize PID controllers
        initializeSlideControllers();

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();
        runtime.reset();

        if (isStopRequested()) return;

        while(opModeIsActive() && !isStopRequested()) {
            gamepadCalc.calculate();

//            // Toggle PID control with left stick button (button 9)
//            if (gamepadCalc.getGamepad2().back && gamepadCalc.getGamepad2().y) {
//                usePIDControlVert = !usePIDControlVert;
//                // Reset controllers when switching modes
//                if (usePIDControlVert) {
//                    leftSlideController.reset();
//                    rightSlideController.reset();
//                }
//                telemetry.update();
//                sleep(100); // Debounce delay
//            }

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
//            handleVerticalClawContinuous();
//            handleVerticalClaw();
            handleVerticalClawRotation();
            handleVerticalClawGripper();
            handleHorizontalClaw();
//            handleHorizontalClawContinuous();
            updateTelemetry();
            drive.update();

//            if(gamepadCalc.getGamepad2().right_bumper) {
//                drive.setPoseEstimate(PoseStorage.currentPose);
//                telemetry.addData("Heading reseted to: ", PoseStorage.currentPose);
//                telemetry.update();
//            }
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
        horizClawRotateUpDown = config.getServoIfEnabled("HCRL", HardwareConfig.ClawConfig.ENABLE_HORIZ_ROTATE_LEFT);
        horizClawRotateLeftRight = config.getServoIfEnabled("HCRR", HardwareConfig.ClawConfig.ENABLE_HORIZ_ROTATE_RIGHT);
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
            vertSlideLeft.setDirection(DcMotor.Direction.FORWARD);
            vertSlideRight.setDirection(DcMotor.Direction.REVERSE);
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

    private void initializeSlideControllers() {
        // Create PIDF controllers for both slides
        leftSlideController = new PIDFController(VERT_SLIDE_PID, kV, kA, kStatic);
        rightSlideController = new PIDFController(VERT_SLIDE_PID, kV, kA, kStatic);

        // Set integral bounds to prevent windup
        leftSlideController.setIntegralBounds(-0.2, 0.2);
        rightSlideController.setIntegralBounds(-0.2, 0.2);

        // Set output bounds (motor power limits)
        leftSlideController.setOutputBounds(-currentSlideMaxPower, currentSlideMaxPower);
        rightSlideController.setOutputBounds(-currentSlideMaxPower, currentSlideMaxPower);
        telemetry.addData("currentSlideMaxPower : ", currentSlideMaxPower);
    }

    private void handleVerticalSlides() {
        if (vertSlideLeft == null || vertSlideRight == null || !gamepadCalc.isGamepadSystemHealthy()) {
            telemetry.addData("Vertical Slides Error", "Required components not initialized");
            return;
        }

        // Add position tracking at the start
        updatePositionTracking();

        // Add velocity safety check
        if (Math.abs(verticalVelocity) > VELOCITY_DANGER_THRESHOLD) {
            telemetry.addData("⚠️ WARNING", "Moving too fast: %.2f", verticalVelocity, " tics/sec");
            stopVerticalSlides();
            return;
        }

        Gamepad gamepad2 = gamepadCalc.getGamepad2();
        if (gamepad2 == null) {
            telemetry.addData("Gamepad Error", "Gamepad 2 not available");
            return;
        }

        double slidePower = -gamepad2.left_stick_y;
//        telemetry.addData("Vertical Slide", "Gamepad Triangle power: ", slidePower);
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
                } else if (gamepad2.dpad_right && vertSlideLeft.getCurrentPosition() < VERT_SLIDE_HIGH &&
                        vertSlideLeft.getCurrentPosition() > VERT_SLIDE_LOW) {
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

            case MANUAL_CONTROL:
                if (!isManualControl) {
                    vertSlideState = VertSlideState.IDLE;
                    stopVerticalSlides();
                } else {
                    // Direct power control with safety limits
                    double safePower = getVerticalSlidePower(slidePower);
                    applyPowerToSlides(safePower);
                }
                break;
//                if (!isManualControl) {
//                    vertSlideState = RRTeleOp.VertSlideState.IDLE;
//                    stopVerticalSlides();
//                } else {
//                    // Improved power smoothing with acceleration limiting
//                    double targetPower = slidePower;
//                    double powerDiff = targetPower - lastSlidePower;
//
//                    // Limit acceleration (prevent sudden power changes)
//                    double MAX_POWER_CHANGE = 0.1; // Maximum power change per cycle
//                    powerDiff = Range.clip(powerDiff, -MAX_POWER_CHANGE, MAX_POWER_CHANGE);
//
//                    // Calculate new power with limited acceleration
//                    double smoothedPower = lastSlidePower + powerDiff;
//
//                    // Apply safety limits based on position
//                    double safePower = getVerticalSlidePower(smoothedPower);
//
//                    // Apply final power limits
//                    safePower = Range.clip(safePower, -currentSlideMaxPower, currentSlideMaxPower);
//
//                    // Additional power scaling near limits
//                    int currentPos = vertSlideLeft.getCurrentPosition();
//                    if (currentPos < VERT_SLIDE_MIN + 200 || currentPos > VERT_SLIDE_MAX - 200) {
//                        safePower *= 0.7; // Reduce power near limits
//                    }
////                    // Smooth the power change
////                    double smoothedPower = slidePower; //lastSlidePower + (slidePower - lastSlidePower) * currentPowerSmoothing;
////                    // Apply safety limits
////                    double safePower = getVerticalSlidePower(smoothedPower);
////                    // Limit the maximum power
////                    safePower = Range.clip(smoothedPower, -currentSlideMaxPower, currentSlideMaxPower);
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

            case MOVING_TO_POSITION:
                if (isManualControl) {
                    vertSlideState = VertSlideState.MANUAL_CONTROL;
                } else {
                    if (usePIDControlVert) {
//                    moveVerticalSlidesToPosition(targetVertPosition);
                        updateSlidePIDControl();
                        if (isSlidesAtTarget(10)) {
                            vertSlideState = VertSlideState.IDLE;
                            stopVerticalSlides();
                        }
                    } else {
                        // Direct movement to position without PID
                        updateSlideDirectControl();
                        if (isAtVerticalTarget()) {
                            vertSlideState = VertSlideState.IDLE;
                            stopVerticalSlides();
                        }
                    }
                }
                break;

            case ERROR:
                // Handle error state if needed
                telemetry.addData("Vertical Slides", "ERROR STATE");
                break;
        }

//        // Add position telemetry
//        telemetry.addData("Slide State", vertSlideState);
//        telemetry.addData("Left Position", vertSlideLeft.getCurrentPosition());
//        telemetry.addData("Right Position", vertSlideRight.getCurrentPosition());
//        telemetry.addData("Velocity", vertSlideRight.getVelocity());
//        telemetry.addData("Control Mode", usePIDControlVert ? "PID" : "Direct");

    }

    private void setSlideTarget(int target) {
        target = Range.clip(target, VERT_SLIDE_MIN, VERT_SLIDE_MAX);
        leftSlideController.setTargetPosition(target);
        rightSlideController.setTargetPosition(target);
        targetVertPosition = target;
    }

    private void updateSlideDirectControl() {
        int currentPosition = vertSlideLeft.getCurrentPosition();
        int error = targetVertPosition - currentPosition;

        // Simple proportional control
        double power = error * 0.005; // Simple P factor
        power = Range.clip(power, -currentSlideMaxPower, currentSlideMaxPower);
        telemetry.addData("Power vertical: ", power);

        // Apply safety limits
        power = getVerticalSlidePower(power);

        // Apply power
        applyPowerToSlides(power);
    }

    private void updateSlidePIDControl() {
        long timestamp = System.nanoTime();

        // Get current positions and velocities
        double leftPos = vertSlideLeft.getCurrentPosition();
        double rightPos = vertSlideRight.getCurrentPosition();
        double leftVel = vertSlideLeft.getVelocity();
        double rightVel = vertSlideRight.getVelocity();

        // Calculate PID outputs
        double leftPower = leftSlideController.update(timestamp, leftPos, leftVel);
        double rightPower = rightSlideController.update(timestamp, rightPos, rightVel);

        // Apply powers with position-based safety limits
        leftPower = getVerticalSlidePower(leftPower);
        rightPower = getVerticalSlidePower(rightPower);

        vertSlideLeft.setPower(leftPower);
        vertSlideRight.setPower(rightPower);

        telemetry.addData("Left Power", leftPower);
        telemetry.addData("Right Power", rightPower);
    }

    private boolean isSlidesAtTarget(double tolerance) {
        return leftSlideController.atTargetPosition(tolerance) &&
                rightSlideController.atTargetPosition(tolerance);
    }

    private void applyPowerToSlides(double power) {
        // Progressive power scaling near limits
        int currentPos = (vertSlideLeft.getCurrentPosition() + vertSlideRight.getCurrentPosition())/2;
        if (currentPos < VERT_SLIDE_MIN + 200 || currentPos > VERT_SLIDE_MAX - 200) {
            power *= 0.7;  // Reduce power near limits
        }

        vertSlideLeft.setPower(power);
        vertSlideRight.setPower(power);
    }

    private void handleHorizontalSlides() {
        if (horizSlideLeft == null || horizSlideRight == null) return;

        Gamepad gamepad2 = gamepadCalc.getGamepad2();
        if (gamepad2 == null) {
            telemetry.addData("Gamepad Error", "Gamepad 2 not available");
            return;
        }

        // Constants for smoother control
        final double HORIZ_DEADZONE = 0.1;
        final double SERVO_STOP = 0.5;
        final double SAFETY_MARGIN = 0.02;  // Margin before max/min positions
        final double SLOW_SPEED = 0.02;     // Slower speed near limits
        final double NORMAL_SPEED = 0.5;    // Normal movement speed
        final double RETRACT_SPEED = 0.4;   // Speed for retracting during TRANSFER

        double stickX = gamepad2.right_stick_x;

        // State machine for horizontal slides
        switch (horizSlideState) {
            case INIT:
                // Initialize the sliders to a known starting position
                horizSlideLeft.setPosition(SERVO_STOP);
                horizSlideRight.setPosition(SERVO_STOP);

                // Set the initial position value
                currentHorizPosition = HORIZ_SLIDE_MIN;  // Or any other initial position you prefer

                // Reset any flags or counters if needed
                slideHorizontalInitialized = true;

                // Transition to IDLE state after initialization
                horizSlideState = HorizSlideState.IDLE;

                telemetry.addLine("Horizontal Slides Initialized");
                break;

            case IDLE:
                // Check for manual input
                if (Math.abs(stickX) > HORIZ_DEADZONE) {
                    if (stickX > 0) {
                        horizSlideState = HorizSlideState.MOVING_OUT;
                    } else {
                        horizSlideState = HorizSlideState.MOVING_IN;
                    }
                } else if (gamepad2.dpad_left) {
                    // Add TRANSFER state transition on dpad_left press
                    horizSlideState = HorizSlideState.TRANSFER;
                    telemetry.addLine("Entering Transfer Mode - Retracting Slides");
                }
                break;

            case MOVING_OUT:
                telemetry.addLine("Moving out");
                // Check if we should stop
                if (Math.abs(stickX) <= HORIZ_DEADZONE ||
                        currentHorizPosition >= HORIZ_SLIDE_MAX) {
                    horizSlideState = HorizSlideState.IDLE;
                    break;
                }

                // Calculate target and speed
                double targetOut = currentHorizPosition + (stickX * HORIZ_SLIDE_INCREMENT);
                double moveSpeedOut = (targetOut > HORIZ_SLIDE_MAX - SAFETY_MARGIN) ?
                        SLOW_SPEED : NORMAL_SPEED;

                // Apply movement
                if (currentHorizPosition < HORIZ_SLIDE_MAX) {
                    horizSlideLeft.setPosition(SERVO_STOP + moveSpeedOut);
                    horizSlideRight.setPosition(SERVO_STOP - moveSpeedOut);
                    currentHorizPosition = Range.clip(targetOut, HORIZ_SLIDE_MIN, HORIZ_SLIDE_MAX);
                }

                // Check for TRANSFER state override
                if (gamepad2.dpad_left) {
                    horizSlideState = HorizSlideState.TRANSFER;
                    telemetry.addLine("Transfer Mode Activated");
                }
                break;

            case MOVING_IN:
                telemetry.addLine("Moving in");
                // Check if we should stop
                if (Math.abs(stickX) <= HORIZ_DEADZONE ||
                        currentHorizPosition <= HORIZ_SLIDE_MIN) {
                    horizSlideState = HorizSlideState.IDLE;
                    break;
                }

                // Calculate target and speed
                double targetIn = currentHorizPosition + (stickX * HORIZ_SLIDE_INCREMENT);
                double moveSpeedIn = (targetIn < HORIZ_SLIDE_MIN + SAFETY_MARGIN) ?
                        SLOW_SPEED : NORMAL_SPEED;

                // Apply movement
                if (currentHorizPosition > HORIZ_SLIDE_MIN) {
                    horizSlideLeft.setPosition(SERVO_STOP - moveSpeedIn);
                    horizSlideRight.setPosition(SERVO_STOP + moveSpeedIn);
                    currentHorizPosition = Range.clip(targetIn, HORIZ_SLIDE_MIN, HORIZ_SLIDE_MAX);
                }

                // Check for TRANSFER state override
                if (gamepad2.dpad_left) {
                    horizSlideState = HorizSlideState.TRANSFER;
                    telemetry.addLine("Transfer Mode Activated");
                }
                break;

            case TRANSFER:
                telemetry.addLine("TRANSFER: Retracting horizontal slides");

                // Check if we've fully retracted
                if (currentHorizPosition <= HORIZ_SLIDE_MIN + SAFETY_MARGIN) {
                    // We've reached the minimum position (fully retracted)
                    horizSlideLeft.setPosition(SERVO_STOP);
                    horizSlideRight.setPosition(SERVO_STOP);
                    currentHorizPosition = HORIZ_SLIDE_MIN;

                    // Stay in TRANSFER state while button is held
                    if (!gamepad2.dpad_left) {
                        horizSlideState = HorizSlideState.IDLE;
                    }
                } else {
                    // Not fully retracted yet, move slides in
                    horizSlideLeft.setPosition(SERVO_STOP - RETRACT_SPEED);
                    horizSlideRight.setPosition(SERVO_STOP + RETRACT_SPEED);

                    // Update position tracking (faster retraction than normal)
                    currentHorizPosition -= 0.03;  // Faster decrement for quicker retraction
                    currentHorizPosition = Math.max(currentHorizPosition, HORIZ_SLIDE_MIN);
                }
                break;

            case ERROR:
                // Error recovery
                horizSlideLeft.setPosition(SERVO_STOP);
                horizSlideRight.setPosition(SERVO_STOP);
                if (checkHorizSlideHealth()) {
                    horizSlideState = HorizSlideState.IDLE;
                }
                break;
        }

//        // Add state telemetry
//        telemetry.addData("Horiz State", horizSlideState);
//        telemetry.addData("Current Pos", String.format("%.3f", currentHorizPosition));
//        telemetry.addData("Stick Input", String.format("%.2f", stickX));
    }

    // Horizontal Slide Check helper method
    private boolean checkHorizSlideHealth() {
        // Check if servos are responding
        boolean servosResponding = horizSlideLeft.getPosition() != -1 &&
                horizSlideRight.getPosition() != -1;

        // Check if position tracking is within bounds
        boolean positionValid = currentHorizPosition >= HORIZ_SLIDE_MIN &&
                currentHorizPosition <= HORIZ_SLIDE_MAX;

        return servosResponding && positionValid;
    }


    private void handleVerticalClawRotation() {
        if (vertClawRotateLeft == null || vertClawRotateRight == null || !gamepadCalc.isGamepadSystemHealthy()) {
            telemetry.addData("Rotation Error", "Components not initialized");
            return;
        }

        Gamepad gamepad2 = gamepadCalc.getGamepad2();
        if (gamepad2 == null) return;

        // Define angle limits (adjust these values according to your needs)
        final double MIN_ANGLE = 0.0;   // Minimum servo position (0.0 to 1.0)
        final double MAX_ANGLE = 1.0;   // Maximum servo position (0.0 to 1.0)
        final double TRANSFER_POSITION = 0.5; // Transfer position (between 0.0 and 1.0)

        switch (vertRotationState) {
            case INIT:
                vertClawRotateLeft.setPosition(VERT_CLAW_ARM_STOPPED);
                vertClawRotateRight.setPosition(VERT_CLAW_ARM_STOPPED);
                if (safetyVerticalClaw) {
                    // Reset to middle position during init
                    currentVertPosition = 0.1;
                }
                vertRotationState = VertRotationState.IDLE;
                break;

            case IDLE:
                if (safetyVerticalClaw) {
                    if (gamepad2.left_bumper && currentVertPosition > MIN_ANGLE) {
                        vertRotationState = VertRotationState.ROTATING_LEFT;
                    } else if (gamepad2.left_trigger > 0.5 && currentVertPosition < MAX_ANGLE) {
                        vertRotationState = VertRotationState.ROTATING_RIGHT;
                    } else if (gamepad2.dpad_left) {
                        // Added the transfer state trigger with dpad_left
                        vertRotationState = VertRotationState.TRANSFER;
                        telemetry.addData("Safety Vertical Rotation", "Entering Transfer Mode");
                    }
                } else {
                    if (gamepad2.left_bumper) {
                        vertRotationState = VertRotationState.ROTATING_LEFT;
                    } else if (gamepad2.left_trigger > 0.5) {
                        vertRotationState = VertRotationState.ROTATING_RIGHT;
                    } else if (gamepad2.dpad_left) {
                        // Added the transfer state trigger with dpad_left
                        vertRotationState = VertRotationState.TRANSFER;
                        telemetry.addData("Vertical Rotation", "Entering Transfer Mode");
                    }
                }
                break;

            case ROTATING_LEFT:
                if (safetyVerticalClaw) {
                    if (!gamepad2.left_bumper || currentVertPosition <= MIN_ANGLE) {
                        vertRotationState = VertRotationState.IDLE;
                        vertClawRotateLeft.setPosition(VERT_CLAW_ARM_STOPPED);
                        vertClawRotateRight.setPosition(VERT_CLAW_ARM_STOPPED);
                    } else {
                        // Update current position (adjust the increment as needed for your servo speed)
                        currentVertPosition -= 0.01;
                        // Make sure we don't go below the minimum
                        currentVertPosition = Math.max(currentVertPosition, MIN_ANGLE);
                        vertClawRotateLeft.setPosition(1-VERT_CLAW_LEFT_ROTATION);
                        vertClawRotateRight.setPosition(VERT_CLAW_LEFT_ROTATION);
                    }
                } else {
                    if (!gamepad2.left_bumper){
                        vertRotationState = VertRotationState.IDLE;
                        vertClawRotateLeft.setPosition(VERT_CLAW_ARM_STOPPED);
                        vertClawRotateRight.setPosition(VERT_CLAW_ARM_STOPPED);
                    } else {
                        vertClawRotateLeft.setPosition(1-VERT_CLAW_LEFT_ROTATION);
                        vertClawRotateRight.setPosition(VERT_CLAW_LEFT_ROTATION);
                    }
                }
                break;

            case ROTATING_RIGHT:
                if (safetyVerticalClaw) {
                    if (gamepad2.left_trigger <= 0.5 || currentVertPosition >= MAX_ANGLE) {
                        vertRotationState = VertRotationState.IDLE;
                        vertClawRotateLeft.setPosition(VERT_CLAW_ARM_STOPPED);
                        vertClawRotateRight.setPosition(VERT_CLAW_ARM_STOPPED);
                    } else {
                        // Update current position (adjust the increment as needed for your servo speed)
                        currentVertPosition += 0.01;
                        // Make sure we don't go above the maximum
                        currentVertPosition = Math.min(currentVertPosition, MAX_ANGLE);
                        vertClawRotateLeft.setPosition(1-VERT_CLAW_RIGHT_ROTATION);
                        vertClawRotateRight.setPosition(VERT_CLAW_RIGHT_ROTATION);
                    }
                } else {
                    if (gamepad2.left_trigger <= 0.5){
                        vertRotationState = VertRotationState.IDLE;
                        vertClawRotateLeft.setPosition(VERT_CLAW_ARM_STOPPED);
                        vertClawRotateRight.setPosition(VERT_CLAW_ARM_STOPPED);
                    } else {
                        vertClawRotateLeft.setPosition(1-VERT_CLAW_RIGHT_ROTATION);
                        vertClawRotateRight.setPosition(VERT_CLAW_RIGHT_ROTATION);
                    }
                }
                break;

            case TRANSFER:
                // Set servos to transfer position
                vertClawRotateLeft.setPosition(VERT_CLAW_ARM_STOPPED);
                vertClawRotateRight.setPosition(VERT_CLAW_ARM_STOPPED);

                // Update the current position tracking
                currentVertPosition = TRANSFER_POSITION;

                // Add telemetry
                telemetry.addData("Rotation", "Transfer Position");

                // If user releases dpad_left, return to IDLE
                if (!gamepad2.dpad_left) {
                    vertRotationState = VertRotationState.IDLE;
                } else {
                    vertRotationState = VertRotationState.INIT;
                }
                break;
        }

        telemetry.addData("Rotation State", vertRotationState);
        telemetry.addData("Left Rot Pos", vertClawRotateLeft.getPosition());
        telemetry.addData("Right Rot Pos", vertClawRotateRight.getPosition());
    }


    private void handleVerticalClawGripper() {
        // Constants for smoother control
        final double SERVO_STOP = 0.5;
        final double GRIP_SAFETY_MARGIN = 0.02;  // Margin before max/min positions
        final double GRIP_SLOW_SPEED = 0.1;      // Slower speed near limits
        final double GRIP_NORMAL_SPEED = 0.4;    // Normal movement speed
        final double POSITION_THRESHOLD = 0.1;   // Distance from target to start slowing

        double moveGripSpeed = 0.0;
        double moveSpeedOutGrip =  GRIP_NORMAL_SPEED;

        if (vertClawGripper == null || !gamepadCalc.isGamepadSystemHealthy()) {
            telemetry.addData("Gripper Error", "Components not initialized");
            return;
        }

        Gamepad gamepad2 = gamepadCalc.getGamepad2();
        if (gamepad2 == null) return;

        switch (vertGripperState) {
            case INIT:
                // First close the gripper
                vertClawGripper.setPosition(VERT_CLAW_GRIP_CLOSE);
                sleep(300);  // Wait for gripper to close

                // Then move to init position to maintain tension
                vertClawGripper.setPosition(VERT_CLAW_GRIP_INIT);
                sleep(200);  // Short delay for stability

                // Update tracking
                currentGripPosition = VERT_CLAW_GRIP_INIT;
                vertGripperState = VertGripperState.IDLE;
                break;

            case IDLE:
                if (gamepad2.x) {
                    vertGripperState = VertGripperState.CLOSING;
                    currentGripPosition = VERT_CLAW_GRIP_CLOSE;
                } else if (gamepad2.b) {
                    vertGripperState = VertGripperState.OPENING;
                    currentGripPosition = VERT_CLAW_GRIP_OPEN;
                }
                break;

            case OPENING:
                if (currentGripPosition <= VERT_CLAW_GRIP_OPEN + GRIP_SAFETY_MARGIN) {
                    // We've reached the open position
                    vertClawGripper.setPosition(VERT_CLAW_GRIP_OPEN);
                    currentGripPosition = VERT_CLAW_GRIP_OPEN;
                    vertGripperState = VertGripperState.IDLE;
                } else {
                    // Calculate distance to target
                    double distanceToTarget = currentGripPosition - VERT_CLAW_GRIP_OPEN;
                    if (distanceToTarget < POSITION_THRESHOLD) {
                        // Linear interpolation between slow and normal speed
                        double speedFactor = distanceToTarget / POSITION_THRESHOLD;
                        moveGripSpeed = GRIP_SLOW_SPEED + (GRIP_NORMAL_SPEED - GRIP_SLOW_SPEED) * speedFactor;
                    } else {
                        moveGripSpeed = GRIP_NORMAL_SPEED;
                    }

                    double newGripPosition = currentGripPosition - moveGripSpeed;
                    newGripPosition = Range.clip(newGripPosition, VERT_CLAW_GRIP_OPEN, VERT_CLAW_GRIP_CLOSE);

                    vertClawGripper.setPosition(newGripPosition);
                    currentGripPosition = newGripPosition;

                    // Debug telemetry
                    telemetry.addData("Distance to Target", distanceToTarget);
                    telemetry.addData("Move Speed", moveGripSpeed);
                    telemetry.addData("Current Position", currentGripPosition);

//                    telemetry.addData("newgridPosition:", newGripPosition);
//                    moveSpeedOutGrip = (newGripPosition > VERT_CLAW_GRIP_OPEN + GRIP_SAFETY_MARGIN) ?
//                            GRIP_SLOW_SPEED : GRIP_NORMAL_SPEED;
//                    currentGripPosition = Range.clip(newGripPosition,
//                            VERT_CLAW_GRIP_OPEN,
//                            VERT_CLAW_GRIP_CLOSE);
////                    vertClawGripper.setPosition(newGripPosition);
//                    vertClawGripper.setPosition(VERT_CLAW_GRIP_INIT - moveSpeedOutGrip);
//                    currentGripPosition = moveSpeedOutGrip;
                }
                break;

            case CLOSING:
                if (currentGripPosition >= VERT_CLAW_GRIP_CLOSE) {
                    vertClawGripper.setPosition(VERT_CLAW_GRIP_CLOSE);
                    vertGripperState = VertGripperState.IDLE;
                } else {
                    double newGripPosition = currentGripPosition + VERT_GRIP_INCREMENT;
                    newGripPosition = Range.clip(newGripPosition,
                            VERT_CLAW_GRIP_OPEN,
                            VERT_CLAW_GRIP_CLOSE);
                    vertClawGripper.setPosition(newGripPosition);
                    currentGripPosition = newGripPosition;
                }
                break;
        }

        telemetry.addData("Gripper State", vertGripperState);
        telemetry.addData("Grip Position", String.format("%.2f", currentGripPosition));
    }





    private void handleVerticalClawRotation2() {
        // Constants for smoother control
        final double SERVO_STOP = 0.5;
        final double GRIP_SAFETY_MARGIN = 0.02;  // Margin before max/min positions
        final double GRIP_SLOW_SPEED = 0.1;      // Slower speed near limits
        final double GRIP_NORMAL_SPEED = 0.4;    // Normal movement speed
        final double POSITION_THRESHOLD = 0.1;   // Distance from target to start slowing

        double moveGripSpeed = 0.0;
        double moveSpeedOutGrip =  GRIP_NORMAL_SPEED;

        if (vertClawGripper == null || !gamepadCalc.isGamepadSystemHealthy()) {
            telemetry.addData("Gripper Error", "Components not initialized");
            return;
        }

        Gamepad gamepad2 = gamepadCalc.getGamepad2();
        if (gamepad2 == null) return;

        switch (vertGripperState) {
            case INIT:
                // First close the gripper
                vertClawGripper.setPosition(VERT_CLAW_GRIP_CLOSE);
                sleep(300);  // Wait for gripper to close

                // Then move to init position to maintain tension
                vertClawGripper.setPosition(VERT_CLAW_GRIP_INIT);
                sleep(200);  // Short delay for stability

                // Update tracking
                currentGripPosition = VERT_CLAW_GRIP_INIT;
                vertGripperState = VertGripperState.IDLE;
                break;

            case IDLE:
                if (gamepad2.x) {
                    vertGripperState = VertGripperState.CLOSING;
                    currentGripPosition = VERT_CLAW_GRIP_CLOSE;
                } else if (gamepad2.b) {
                    vertGripperState = VertGripperState.OPENING;
                    currentGripPosition = VERT_CLAW_GRIP_OPEN;
                }
                break;

            case OPENING:
                if (currentGripPosition <= VERT_CLAW_GRIP_OPEN + GRIP_SAFETY_MARGIN) {
                    // We've reached the open position
                    vertClawGripper.setPosition(VERT_CLAW_GRIP_OPEN);
                    currentGripPosition = VERT_CLAW_GRIP_OPEN;
                    vertGripperState = VertGripperState.IDLE;
                } else {
                    // Calculate distance to target
                    double distanceToTarget = currentGripPosition - VERT_CLAW_GRIP_OPEN;
                    if (distanceToTarget < POSITION_THRESHOLD) {
                        // Linear interpolation between slow and normal speed
                        double speedFactor = distanceToTarget / POSITION_THRESHOLD;
                        moveGripSpeed = GRIP_SLOW_SPEED + (GRIP_NORMAL_SPEED - GRIP_SLOW_SPEED) * speedFactor;
                    } else {
                        moveGripSpeed = GRIP_NORMAL_SPEED;
                    }

                    double newGripPosition = currentGripPosition - moveGripSpeed;
                    newGripPosition = Range.clip(newGripPosition, VERT_CLAW_GRIP_OPEN, VERT_CLAW_GRIP_CLOSE);

                    vertClawGripper.setPosition(newGripPosition);
                    currentGripPosition = newGripPosition;

                    // Debug telemetry
                    telemetry.addData("Distance to Target", distanceToTarget);
                    telemetry.addData("Move Speed", moveGripSpeed);
                    telemetry.addData("Current Position", currentGripPosition);

//                    telemetry.addData("newgridPosition:", newGripPosition);
//                    moveSpeedOutGrip = (newGripPosition > VERT_CLAW_GRIP_OPEN + GRIP_SAFETY_MARGIN) ?
//                            GRIP_SLOW_SPEED : GRIP_NORMAL_SPEED;
//                    currentGripPosition = Range.clip(newGripPosition,
//                            VERT_CLAW_GRIP_OPEN,
//                            VERT_CLAW_GRIP_CLOSE);
////                    vertClawGripper.setPosition(newGripPosition);
//                    vertClawGripper.setPosition(VERT_CLAW_GRIP_INIT - moveSpeedOutGrip);
//                    currentGripPosition = moveSpeedOutGrip;
                }
                break;

            case CLOSING:
                if (currentGripPosition >= VERT_CLAW_GRIP_CLOSE) {
                    vertClawGripper.setPosition(VERT_CLAW_GRIP_CLOSE);
                    vertGripperState = VertGripperState.IDLE;
                } else {
                    double newGripPosition = currentGripPosition + VERT_GRIP_INCREMENT;
                    newGripPosition = Range.clip(newGripPosition,
                            VERT_CLAW_GRIP_OPEN,
                            VERT_CLAW_GRIP_CLOSE);
                    vertClawGripper.setPosition(newGripPosition);
                    currentGripPosition = newGripPosition;
                }
                break;
        }

        telemetry.addData("Gripper State", vertGripperState);
        telemetry.addData("Grip Position", String.format("%.2f", currentGripPosition));
    }







//
//    private void handleVerticalClawRotation() {
//        if (vertClawRotateLeft == null || vertClawRotateRight == null || !gamepadCalc.isGamepadSystemHealthy()) {
//            telemetry.addData("Rotation Error", "Components not initialized");
//            return;
//        }
//
//        Gamepad gamepad2 = gamepadCalc.getGamepad2();
//        if (gamepad2 == null) return;
//
//        switch (vertRotationState) {
//            case INIT:
//                vertClawRotateLeft.setPosition(VERT_CLAW_ARM_STOPPED);
//                vertClawRotateRight.setPosition(VERT_CLAW_ARM_STOPPED);
//                vertRotationState = VertRotationState.IDLE;
//                break;
//
//            case IDLE:
//                if (gamepad2.left_bumper) {
//                    vertRotationState = VertRotationState.ROTATING_LEFT;
//                } else if (gamepad2.left_trigger > 0.5) {
//                    vertRotationState = VertRotationState.ROTATING_RIGHT;
//                }
//                break;
//
//            case ROTATING_LEFT:
//                if (!gamepad2.left_bumper) {
//                    vertRotationState = VertRotationState.IDLE;
//                    vertClawRotateLeft.setPosition(VERT_CLAW_ARM_STOPPED);
//                    vertClawRotateRight.setPosition(VERT_CLAW_ARM_STOPPED);
//                } else {
//                    vertClawRotateLeft.setPosition(VERT_CLAW_LEFT_ROTATION);
//                    vertClawRotateRight.setPosition(1 - VERT_CLAW_LEFT_ROTATION);
//                }
//                break;
//
//            case ROTATING_RIGHT:
//                if (gamepad2.left_trigger <= 0.5) {
//                    vertRotationState = VertRotationState.IDLE;
//                    vertClawRotateLeft.setPosition(VERT_CLAW_ARM_STOPPED);
//                    vertClawRotateRight.setPosition(VERT_CLAW_ARM_STOPPED);
//                } else {
//                    vertClawRotateLeft.setPosition(VERT_CLAW_RIGHT_ROTATION);
//                    vertClawRotateRight.setPosition(1 - VERT_CLAW_RIGHT_ROTATION);
//                }
//                break;
//        }
//
//        telemetry.addData("Rotation State", vertRotationState);
//        telemetry.addData("Left Rot Pos", vertClawRotateLeft.getPosition());
//        telemetry.addData("Right Rot Pos", vertClawRotateRight.getPosition());
//    }
//






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

        final double VERT_GRIP_DEADZONE = 0.1;
        final double VERT_GRIP_SAFETY_MARGIN = 0.02;  // Margin before max/min positions
        final double VERT_GRIP_SLOW_SPEED = 0.02;      // Slower speed near limits
        final double VERT_GRIP_NORMAL_SPEED = 0.5;    // Normal movement speed

        double clawAction = (gamepad2.x ? -1.0 : (gamepad2.b ? 1.0 : 0.0));

        // Add telemetry for current state and positions
        telemetry.addData("Vertical Claw State", vertClawState);
        telemetry.addData("Rotate Left Position", vertClawRotateLeft.getPosition());
        telemetry.addData("Rotate Right Position", vertClawRotateRight.getPosition());
        telemetry.addData("Gripper Position", vertClawGripper.getPosition());

        switch (vertClawState) {
            case INIT:
                // Set initial positions
                vertClawRotateLeft.setPosition(VERT_CLAW_ARM_STOPPED);
                vertClawRotateRight.setPosition(VERT_CLAW_ARM_STOPPED);
                vertClawGripper.setPosition(VERT_CLAW_GRIP_INIT);
                telemetry.addData("Vertical Claw", "Initializing...");
                // Move to IDLE after initialization
                vertClawState = VertClawState.IDLE;
                break;

            case IDLE:
                // Set default rotation position
//                vertClawRotateLeft.setPosition(VERT_CLAW_ARM_STOPPED);
//                vertClawRotateRight.setPosition(1 - VERT_CLAW_ARM_STOPPED);

                // Handle rotation controls
                if (gamepad2.left_bumper) {
                    vertClawState = VertClawState.CLOSE;
//                    vertClawRotateLeft.setPosition(VERT_CLAW_LEFT_ROTATION);
//                    vertClawRotateRight.setPosition(1 - VERT_CLAW_LEFT_ROTATION);
                    telemetry.addData("Rotate Action: ", "Parallel");
                } else if (gamepad2.left_trigger > 0.5) {
                    vertClawState = VertClawState.OPEN;
//                    vertClawRotateLeft.setPosition(VERT_CLAW_ROTATED);
//                    vertClawRotateRight.setPosition(1 - VERT_CLAW_ROTATED);
                    telemetry.addData("Rotate Action: ", "Rotated");
                }

                // Handle gripper state changes
                if (gamepad2.x) {
                    vertClawState = VertClawState.OPEN;
                } else if (gamepad2.b) {
                    vertClawState = VertClawState.CLOSE;
                }
                break;

            case OPEN:
                // Calculate target position for opening
                double newGripPosition = currentGripPosition - VERT_GRIP_INCREMENT;
                telemetry.addData("New grip possition: ", newGripPosition);

                // Check if we've reached the maximum open position
                if (currentGripPosition >= VERT_CLAW_RIGHT_ROTATION) {
                    // At maximum open position, stop
                    vertClawGripper.setPosition(VERT_CLAW_ARM_STOPPED);
                    vertClawState = VertClawState.IDLE;
                    telemetry.addData("Gripper", "At Max Open");
                } else {
                    // Continue opening
                    newGripPosition = Range.clip(newGripPosition,
                            VERT_CLAW_LEFT_ROTATION,
                            VERT_CLAW_RIGHT_ROTATION);
                    vertClawGripper.setPosition(newGripPosition);
                    currentGripPosition = newGripPosition;

                    telemetry.addData("Gripper", "Opening");
                    telemetry.addData("Position", String.format("%.2f", currentGripPosition));
                }


//                // Apply speed control near limits
//                double moveSpeed = (targetPosition > VERT_CLAW_RIGHT_ROTATION - VERT_GRIP_SAFETY_MARGIN) ?
//                        VERT_GRIP_SLOW_SPEED : VERT_GRIP_NORMAL_SPEED;
//
//                // Check if at limit
//                if (currentGripPosition >= VERT_CLAW_RIGHT_ROTATION) {
//                    vertClawGripper.setPosition(VERT_CLAW_RIGHT_ROTATION);
//                    vertClawState = VertClawState.IDLE;
//                } else {
//                    // Move toward open position
//                    double newPosition = currentGripPosition + moveSpeed;
//                    newPosition = Range.clip(newPosition, VERT_CLAW_LEFT_ROTATION, VERT_CLAW_RIGHT_ROTATION);
//                    vertClawGripper.setPosition(newPosition);
//                    currentGripPosition = newPosition;
//                }
//
//                telemetry.addData("Gripper Action", "Opening");
//                telemetry.addData("Target Pos", targetPosition);
//                telemetry.addData("Current Pos", currentGripPosition);
//                break;
//                // Check if we should stop
//                if (gamepad2.b  &&
//                        currentGripPosition >= VERT_CLAW_RIGHT_ROTATION) {
//                    vertClawState = VertClawState.IDLE;
//                    break;
//                }
//
//                // Calculate terget claw position and speed
//                double clawTargetOut = currentGripPosition + (clawAction*VERT_GRIP_NORMAL_SPEED);
//                double moveClawSpeedClose = (clawTargetOut < VERT_CLAW_RIGHT_ROTATION + VERT_GRIP_SAFETY_MARGIN) ?
//                        VERT_GRIP_SLOW_SPEED : VERT_GRIP_NORMAL_SPEED;
//
//                // Apply movement
//                if (currentVertClawPosition > VERT_CLAW_RIGHT_ROTATION) {
//                    vertClawGripper.setPosition(VERT_CLAW_GRIP_INIT+moveClawSpeedClose);
//                    currentVertClawPosition = Range.clip(VERT_CLAW_LEFT_ROTATION, VERT_CLAW_RIGHT_ROTATION);
//                }
////                vertClawGripper.setPosition(CLAW_OPEN);
//                telemetry.addData("Gripper Action", "Opening");
//                vertClawState = VertClawState.IDLE;  // Return to IDLE after setting position
//                break;

            case CLOSE:
                if (currentGripPosition >= VERT_CLAW_RIGHT_ROTATION) {
                    // At maximum closed position, stop
                    vertClawGripper.setPosition(VERT_CLAW_ARM_STOPPED);
                    vertClawState = VertClawState.IDLE;
                    telemetry.addData("currentGripPosition: ", currentGripPosition);
                    telemetry.addData("Gripper", "At Max Close");
                } else {
                    // Continue closing
                    double newClosePosition = currentGripPosition + VERT_GRIP_INCREMENT;
                    newClosePosition = Range.clip(newClosePosition,
                            VERT_CLAW_LEFT_ROTATION,
                            VERT_CLAW_RIGHT_ROTATION);
                    vertClawGripper.setPosition(newClosePosition);
                    currentGripPosition = newClosePosition;

                    telemetry.addData("Gripper", "Closing");
                    telemetry.addData("Position", String.format("%.2f", currentGripPosition));
                }

//                vertClawGripper.setPosition(CLAW_CLOSED);
//                telemetry.addData("Gripper Action", "Closing");
//                vertClawState = RRTeleOp.VertClawState.IDLE;  // Return to IDLE after setting position
//                break;

            case TRANSFER:

                vertClawRotateLeft.setPosition(VERT_CLAW_ARM_STOPPED);
                vertClawGripper.setPosition(CLAW_OPEN);
                telemetry.addData("Vertical Claw", "Transfer Position");
                if (Math.abs(vertClawRotateLeft.getPosition() - VERT_CLAW_ARM_STOPPED) < 0.05) {
                    vertClawState = RRTeleOp.VertClawState.IDLE;
                }
                break;

            case HOME:
                // Check if slides are in safe position first
                if (vertSlideLeft.getCurrentPosition() > VERT_SLIDE_LOW + 50) {
                    telemetry.addData("Home Sequence", "Waiting for slides");
                    break;
                }

                vertClawRotateLeft.setPosition(VERT_CLAW_ARM_STOPPED);
                vertClawGripper.setPosition(CLAW_CLOSED);
                telemetry.addData("Home Sequence", "Setting home position");

                // Check if everything is in position
                if (Math.abs(vertClawRotateLeft.getPosition() - VERT_CLAW_ARM_STOPPED) < 0.05 &&
                        Math.abs(vertClawGripper.getPosition() - CLAW_CLOSED) < 0.05) {
                    telemetry.addData("Home Sequence", "Complete");
                    vertClawState = RRTeleOp.VertClawState.IDLE;
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
        telemetry.addData("Left Servo Position", vertClawRotateLeft.getPosition());
        telemetry.addData("Right Servo Position", vertClawRotateRight.getPosition());
        telemetry.addData("Left Bumper Pressed", gamepad2.left_bumper);
        telemetry.addData("Left Trigger Value", gamepad2.left_trigger);
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
        if (horizClawRotateUpDown == null || horizClawRotateLeftRight == null || horizClawGripper == null) return;

        // Define rotation limits for horizontal claw
        final double MIN_HORIZ_ANGLE = 0.0;  // Minimum position limit
        final double MAX_HORIZ_ANGLE = 1.0;  // Maximum position limit

        // Rotation control (right bumper/trigger)
        if (gamepad2.right_bumper && currentHorizPosition > MIN_HORIZ_ANGLE) {
            // Move backward (decrease position)
            currentHorizPosition -= 0.01;  // Adjust increment as needed
            currentHorizPosition = Math.max(currentHorizPosition, MIN_HORIZ_ANGLE);

            horizClawRotateUpDown.setPosition(1-HORIZ_CLAW_DOWN);
            horizClawRotateLeftRight.setPosition(HORIZ_CLAW_DOWN);
            telemetry.addData("Rotate back Action", "Up");
            telemetry.addData("HC value:", horizClawRotateUpDown.getPosition());
            telemetry.addData("HC Position:", currentHorizPosition);
        } else if (gamepad2.right_trigger > 0.2 && currentHorizPosition < MAX_HORIZ_ANGLE) {
            // Move forward (increase position)
            currentHorizPosition += 0.01;  // Adjust increment as needed
            currentHorizPosition = Math.min(currentHorizPosition, MAX_HORIZ_ANGLE);

            horizClawRotateUpDown.setPosition(HORIZ_CLAW_DOWN);
            horizClawRotateLeftRight.setPosition(1-HORIZ_CLAW_DOWN);
            telemetry.addData("rotate back Action", "Down");
            telemetry.addData("HCC value", horizClawRotateUpDown.getPosition());
            telemetry.addData("HC Position:", currentHorizPosition);
        } //else {
//            // No input or at limit - stop rotation
//            horizClawRotateUpDown.setPosition(0.5);  // Neutral position to stop continuous rotation
//            horizClawRotateLeftRight.setPosition(0.5); // Neutral position to stop continuous rotation
//            telemetry.addData("Rotate Status", "Stopped");
//        }

//        // Gripper control (B button)
//        if (gamepad2.y) {
//            horizClawGripper.setPosition(CLAW_OPEN);
//            telemetry.addData("Claw Action", "Open");
//
//        } else if (gamepad2.a) {
//            horizClawGripper.setPosition(CLAW_CLOSED);
//            telemetry.addData("Claw Action", "Close");
//
//        }

        // Gripper control with continuous rotation servo
        long currentTime = System.currentTimeMillis();

        if (gamepad2.y && isClawClosed) {
            // Open claw
            telemetry.addLine("Claw open");
            horizClawGripper.setPosition(CLAW_OPEN_SPEED);
            lastGripChangeTime = currentTime;
            isClawClosed = false;
        } else if (gamepad2.a && !isClawClosed) {
            // Close claw
            telemetry.addLine("Claw closes");
            horizClawGripper.setPosition(CLAW_CLOSE_SPEED);
            lastGripChangeTime = currentTime;
            isClawClosed = true;
        }

//        // Stop the servo after duration
//        if (currentTime - lastGripChangeTime > GRIP_DURATION_MS) {
//            horizClawGripper.setPosition(CLAW_STOP);
//        }
    }

    private void handleHorizontalClawContinuous() {
        if (horizClawRotateUpDown == null || horizClawRotateLeftRight == null ||
                horizClawGripper == null || !gamepadCalc.isGamepadSystemHealthy()) {
            telemetry.addData("Horizontal Claw Error", "Required components not initialized");
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
        final double ROTATE_SPEED_LEFT = 0.4;   // Speed for left rotation
        final double ROTATE_SPEED_RIGHT = 0.46;  // Reduced speed for right rotation

        // Rotation control with limits
        if (gamepad2.right_bumper && !isHorizClawAtLeftLimit) {
            // Left rotation only if not at left limit
            horizClawRotateUpDown.setPosition(SERVO_STOP - ROTATE_SPEED_LEFT);
            horizClawRotateLeftRight.setPosition(SERVO_STOP - ROTATE_SPEED_LEFT);
            isHorizClawAtRightLimit = false;  // Reset right limit when moving left
            telemetry.addData("Horiz Rotate", "Left");
        }
        else if (gamepad2.right_trigger > 0.1 && !isHorizClawAtRightLimit) {
            // Right rotation only if not at right limit
            horizClawRotateUpDown.setPosition(SERVO_STOP + ROTATE_SPEED_RIGHT);
            horizClawRotateLeftRight.setPosition(SERVO_STOP + ROTATE_SPEED_RIGHT);
            isHorizClawAtLeftLimit = false;  // Reset left limit when moving right
            telemetry.addData("Horiz Rotate", "Right");
        }
        else {
            horizClawRotateUpDown.setPosition(SERVO_STOP);
            horizClawRotateLeftRight.setPosition(SERVO_STOP);
            telemetry.addData("Horiz Rotate", "Stopped");
        }

        // Gripper control remains position-based (non-continuous)
        if (gamepad2.y) {
            horizClawGripper.setPosition(CLAW_CLOSED);
            telemetry.addData("Horiz Gripper", "Closed");
        } else if (gamepad2.a) {
            horizClawGripper.setPosition(CLAW_OPEN);
            telemetry.addData("Horiz Gripper", "Open");
        }

        // Add telemetry for debugging
        telemetry.addData("Horiz claw Position", horizClawRotateUpDown.getPosition());
        telemetry.addData("Right Servo Position", horizClawRotateLeftRight.getPosition());
        telemetry.addData("Right Bumper Pressed", gamepad2.right_bumper);
        telemetry.addData("Right Trigger Value", gamepad2.right_trigger);
    }

    private void moveVerticalSlidesToPosition(int targetPosition) {
        // Safety bounds check
        targetPosition = Range.clip(targetPosition, VERT_SLIDE_MIN, VERT_SLIDE_MAX);

        int currentPosition = (vertSlideLeft.getCurrentPosition() + vertSlideRight.getCurrentPosition()) / 2;
        double error = targetPosition - currentPosition;
        double deltaTime = vertSlidePIDTimer.seconds();

        // PID calculation
        vertIntegralSum += error * deltaTime;
        double derivative = (error - lastVertError) / deltaTime;

        double power = (error * SLIDES_P) + (vertIntegralSum * SLIDES_I) + (derivative * SLIDES_D);
        power = Range.clip(power, -1.0, 1.0);

        // Apply power to motors
        vertSlideLeft.setPower(power);
        vertSlideRight.setPower(power);

        // Update PID variables
        lastVertError = error;
        vertSlidePIDTimer.reset();
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
                VERT_CLAW_ARM_STOPPED,  // Rotate claw parallel
                CLAW_OPEN           // Open claw for pickup
        );
        sleep(500);  // Short delay for stability

        // Grasp object
        coordinatedMove(
                VERT_SLIDE_LOW,      // Keep slides down
                VERT_CLAW_ARM_STOPPED,  // Keep claw parallel
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
        vertClawRotateLeft.setPosition(VERT_CLAW_ARM_STOPPED);
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
        vertClawState = RRTeleOp.VertClawState.CLOSE;

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
        vertClawState = RRTeleOp.VertClawState.OPEN;
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
        vertClawRotateLeft.setPosition(VERT_CLAW_ARM_STOPPED);
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

//        if (HardwareConfig.HorizontalSlideConfig.ENABLE_LEFT) {
//            telemetry.addData("Left Slide Position", armMotorLeft.getCurrentPosition());
//        }
//        if (HardwareConfig.HorizontalSlideConfig.ENABLE_RIGHT) {
//            telemetry.addData("Right Slide Position", armMotorRight.getCurrentPosition());
//        }
//        if (HardwareConfig.VerticalSlideConfig.ENABLE_LEFT) {
//            telemetry.addData("Left Rotation Position", rotateMotorLeft.getCurrentPosition());
//        }
//        if (HardwareConfig.VerticalSlideConfig.ENABLE_RIGHT) {
//            telemetry.addData("Right Rotation Position", rotateMotorRight.getCurrentPosition());
//        }
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

//        // Drive System
//        telemetry.addLine("=== Drive System ===");
//        if (drive.getPoseEstimate() != null) {
//            telemetry.addData("Pose Estimate", drive.getPoseEstimate());
//        }

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

//        // Horizontal Slides
//        telemetry.addLine("=== Horizontal Slides ===");
//        if (horizSlideLeft != null && horizSlideRight != null) {
//            telemetry.addData("Left Position", horizSlideLeft.getPosition());
//            telemetry.addData("Right Position", horizSlideRight.getPosition());
//            telemetry.addData("Current Position", currentHorizPosition);
//        }

//        // Vertical Claw
//        telemetry.addLine("=== Vertical Claw ===");
//        if (vertClawRotateLeft != null && vertClawRotateRight != null && vertClawGripper != null) {
//            telemetry.addData("Rotate Left", vertClawRotateLeft.getPosition());
//            telemetry.addData("Rotate Right", vertClawRotateRight.getPosition());
//            telemetry.addData("Gripper", vertClawGripper.getPosition());
//        }

//        // Horizontal Claw
//        telemetry.addLine("=== Horizontal Claw ===");
//        if (horizClawRotateUpDown != null && horizClawRotateLeftRight != null && horizClawGripper != null) {
//            telemetry.addData("Rotate Left", horizClawRotateUpDown.getPosition());
//            telemetry.addData("Rotate Right", horizClawRotateLeftRight.getPosition());
//            telemetry.addData("Gripper", horizClawGripper.getPosition());
//        }

        telemetry.update();
    }


}

