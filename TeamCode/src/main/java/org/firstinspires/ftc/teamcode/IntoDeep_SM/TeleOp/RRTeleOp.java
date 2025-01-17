package org.firstinspires.ftc.teamcode.IntoDeep_SM.TeleOp;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import static org.firstinspires.ftc.teamcode.config.RobotConfig.HardwareConfig;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorEx.CurrentUnit;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Functions.ArmEncoder;  //?
import org.firstinspires.ftc.teamcode.Functions.GamepadCalc;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.advanced.PoseStorage;

@TeleOp(name="RRTeleOpIntoDeep", group = "IntoDeep_SM")
public class RRTeleOp extends LinearOpMode {
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

    // GoBilda motor constants
    private static final double GOBILDA_TICKS_PER_REV = 537.7; // For 19.2:1 Yellow Jacket
    private static final double GOBILDA_MAX_RPM = 312; // Max RPM for Yellow Jacket
    private static final double WHEEL_DIAMETER_MM = 96.0;  // 96mm GoBilda Mecanum wheel
    private static final double DRIVE_GEAR_RATIO = 1.0;    // Direct drive


    // Vertical slider positions
    private static final int VERT_SLIDE_MIN = 0;
    private static final int VERT_SLIDE_MAX = 4500;
    private static final int VERT_SLIDE_LOW = 500;
    private static final int VERT_SLIDE_MID = 2250;
    private static final int VERT_SLIDE_HIGH = 4000;

    // Horizontal slider positions
    private static final double HORIZ_SLIDE_MIN = 0.0;
    private static final double HORIZ_SLIDE_MAX = 1.0;
    private static final double HORIZ_SLIDE_INCREMENT = 0.02;

    // Claw rotation positions
    private static final double VERT_CLAW_PARALLEL = 0.0;
    private static final double VERT_CLAW_ROTATED = 1.0;
    private static final double HORIZ_CLAW_PARALLEL = 0.0;
    private static final double HORIZ_CLAW_ROTATED = 1.0;

    // Claw gripper positions
    private static final double CLAW_OPEN = 0.7;
    private static final double CLAW_CLOSED = 0.2;

    // PID Constants for vertical slides
    private static final double SLIDES_P = 0.005;
    private static final double SLIDES_I = 0.0;
    private static final double SLIDES_D = 0.0;

    // State tracking
    private VertSlideState vertSlideState = VertSlideState.IDLE;
    private HorizSlideState horizSlideState = HorizSlideState.IDLE;
    private int targetVertPosition = 0;
    private double currentHorizPosition = HORIZ_SLIDE_MIN;

    // Viper slide extension motors
    private DcMotorEx armMotorLeft, armMotorRight; //?

    // Viper slide rotation motors
    private DcMotorEx rotateMotorLeft, rotateMotorRight; //?

    // Intake servos
    private Servo leftAxleServo, rightAxleServo;      // ?Servos for rotating the intake axle
    private Servo leftGeckoServo, rightGeckoServo;    // ?Servos for Gecko wheels
    private Servo horizClawServo, vertClawServo;    // ?Servos for Gecko wheels
    private Servo leftSliderServo, rightSliderServo;    // ?Servos for Gecko wheels


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
    private ElapsedTime rotationTimer = new ElapsedTime();
    private double lastSlidesError = 0;
    private double lastRotationError = 0;
    private double slidesIntegralSum = 0;
    private double rotationIntegralSum = 0;

    // Safety constants
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
    private double lastSlidePower = 0;
    private double lastRotationPower = 0;

    private double vertIntegralSum = 0;  // For accumulating error over time
    private double lastVertError = 0;    // For calculating the derivative term


    // State machine enums
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
        IDLE,
        OPEN,
        CLOSE,
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
            movement = gamepadCalc.getGamepad1().left_trigger - gamepadCalc.getGamepad1().right_trigger;

            handleDrive(drive);
            //handleViperSlides();
            //handleRotation();
            //handleIntake();
            handleVerticalSlides();
            handleHorizontalSlides();
            handleVerticalClaw();
            handleHorizontalClaw();
            updateTelemetry();
            drive.update();

            if(gamepad2.right_bumper) {
                drive.setPoseEstimate(PoseStorage.currentPose);
                telemetry.addData("Heading reseted to: ", PoseStorage.currentPose);
                telemetry.update();
            }
        }
    }

    private void initializeHardware() {
        config = new RobotConfig(hardwareMap);

        // Initialize drive motors
        leftMotor = config.getMotorIfEnabled("FL", HardwareConfig.ENABLE_FL);
        rightMotor = config.getMotorIfEnabled("FR", HardwareConfig.ENABLE_FR);
        leftMotorBack = config.getMotorIfEnabled("BL", HardwareConfig.ENABLE_BL);
        rightMotorBack = config.getMotorIfEnabled("BR", HardwareConfig.ENABLE_BR);

//        // Initialize viper slide motors
//        armMotorLeft = config.getMotorExIfEnabled("SL", HardwareConfig.ENABLE_SLIDE_LEFT);
//        armMotorRight = config.getMotorExIfEnabled("SR", HardwareConfig.ENABLE_SLIDE_RIGHT);
//
//        // Initialize rotation motors
//        rotateMotorLeft = config.getMotorExIfEnabled("RL", HardwareConfig.ENABLE_ROTATE_LEFT);
//        rotateMotorRight = config.getMotorExIfEnabled("RR", HardwareConfig.ENABLE_ROTATE_RIGHT);
//
//
//        // Initialize intake servos
//        leftAxleServo = config.getServoIfEnabled("LA", HardwareConfig.ENABLE_LEFT_AXLE);
//        rightAxleServo = config.getServoIfEnabled("RA", HardwareConfig.ENABLE_RIGHT_AXLE);
//        leftGeckoServo = config.getServoIfEnabled("LG", HardwareConfig.ENABLE_LEFT_GECKO);
//        rightGeckoServo = config.getServoIfEnabled("RG", HardwareConfig.ENABLE_RIGHT_GECKO);

// Initialize vertical slide motors
        vertSlideLeft = config.getMotorExIfEnabled("VSL", HardwareConfig.ENABLE_VERT_SLIDE_LEFT);
        vertSlideRight = config.getMotorExIfEnabled("VSR", HardwareConfig.ENABLE_VERT_SLIDE_RIGHT);

        // Initialize horizontal slide servos
        horizSlideLeft = config.getServoIfEnabled("HSL", HardwareConfig.ENABLE_HORIZ_SLIDE_LEFT);
        horizSlideRight = config.getServoIfEnabled("HSR", HardwareConfig.ENABLE_HORIZ_SLIDE_RIGHT);

        // Initialize vertical claw servos
        vertClawRotateLeft = config.getServoIfEnabled("VCRL", HardwareConfig.ENABLE_VERT_CLAW_ROTATE_LEFT);
        vertClawRotateRight = config.getServoIfEnabled("VCRR", HardwareConfig.ENABLE_VERT_CLAW_ROTATE_RIGHT);
        vertClawGripper = config.getServoIfEnabled("VCG", HardwareConfig.ENABLE_VERT_CLAW_GRIPPER);

        // Initialize horizontal claw servos
        horizClawRotateLeft = config.getServoIfEnabled("HCRL", HardwareConfig.ENABLE_HORIZ_CLAW_ROTATE_LEFT);
        horizClawRotateRight = config.getServoIfEnabled("HCRR", HardwareConfig.ENABLE_HORIZ_CLAW_ROTATE_RIGHT);
        horizClawGripper = config.getServoIfEnabled("HCG", HardwareConfig.ENABLE_HORIZ_CLAW_GRIPPER);

        // Initialize controller classes
        if (armMotorLeft != null && armMotorRight != null) {
            controller = new ArmEncoder(armMotorLeft, armMotorRight);
        }
        drive = new SampleMecanumDrive(hardwareMap);
        gamepadCalc = new GamepadCalc(this);
    }

    private void setupMotors() {
        // Drive motors setup
        if (leftMotor != null && rightMotor != null && leftMotorBack != null && rightMotorBack != null) {
            leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            leftMotor.setDirection(DcMotor.Direction.REVERSE);
            leftMotorBack.setDirection(DcMotor.Direction.REVERSE);
            rightMotor.setDirection(DcMotor.Direction.FORWARD);
            rightMotorBack.setDirection(DcMotor.Direction.FORWARD);
        }

        // Vertical slide motors setup
        if (vertSlideLeft != null && vertSlideRight != null) {
            vertSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            vertSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            vertSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            vertSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            vertSlideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            vertSlideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            vertSlideRight.setDirection(DcMotor.Direction.REVERSE);
        }
//        // Viper slides
//        if (armMotorLeft != null && armMotorRight != null) {
//            armMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            armMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//            armMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            armMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//            armMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            armMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            armMotorRight.setDirection(DcMotor.Direction.REVERSE);
//        }
//
//        // Rotation motors setup
//        if (rotateMotorLeft != null && rotateMotorRight != null) {
//            rotateMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            rotateMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            rotateMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            rotateMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            rotateMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            rotateMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            rotateMotorRight.setDirection(DcMotor.Direction.REVERSE);
//        }
    }

    private void handleDrive(SampleMecanumDrive drive) {
        Pose2d poseEstimate = drive.getPoseEstimate();
        Vector2d input = new Vector2d(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x
        ).rotated(-poseEstimate.getHeading());

        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -gamepad1.right_stick_x
                )
        );
    }

    private void handleVerticalSlides() {
        if (vertSlideLeft == null || vertSlideRight == null) return;

        double slidePower = -gamepad2.left_stick_y;
        boolean isManualControl = Math.abs(slidePower) > 0.1;

        // State machine for vertical slides
        switch (vertSlideState) {
            case IDLE:
                if (isManualControl) {
                    vertSlideState = VertSlideState.MANUAL_CONTROL;
                } else if (gamepad2.dpad_up) {
                    targetVertPosition = VERT_SLIDE_HIGH;
                    vertSlideState = VertSlideState.MOVING_TO_POSITION;
                } else if (gamepad2.dpad_right) {
                    targetVertPosition = VERT_SLIDE_MID;
                    vertSlideState = VertSlideState.MOVING_TO_POSITION;
                } else if (gamepad2.dpad_down) {
                    targetVertPosition = VERT_SLIDE_LOW;
                    vertSlideState = VertSlideState.MOVING_TO_POSITION;
                }
                break;

            case MANUAL_CONTROL:
                if (!isManualControl) {
                    vertSlideState = VertSlideState.IDLE;
                    stopVerticalSlides();
                } else {
                    double safePower = Range.clip(slidePower, -1.0, 1.0);
                    vertSlideLeft.setPower(safePower);
                    vertSlideRight.setPower(safePower);
                }
                break;

            case MOVING_TO_POSITION:
                if (isManualControl) {
                    vertSlideState = VertSlideState.MANUAL_CONTROL;
                } else {
                    moveVerticalSlidesToPosition(targetVertPosition);
                    if (isAtVerticalTarget()) {
                        vertSlideState = VertSlideState.IDLE;
                        stopVerticalSlides();
                    }
                }
                break;

            case ERROR:
                // Handle error state if needed
                break;
        }
    }

    private void handleHorizontalSlides() {
        if (horizSlideLeft == null || horizSlideRight == null) return;

        // Use right stick X for horizontal movement
        if (Math.abs(gamepad2.right_stick_x) > 0.1) {
            currentHorizPosition += gamepad2.right_stick_x * HORIZ_SLIDE_INCREMENT;
            currentHorizPosition = Range.clip(currentHorizPosition, HORIZ_SLIDE_MIN, HORIZ_SLIDE_MAX);

            horizSlideLeft.setPosition(currentHorizPosition);
            horizSlideRight.setPosition(1 - currentHorizPosition); // Reverse for opposite side
        }
    }

    private void handleVerticalClaw() {
        if (vertClawRotateLeft == null || vertClawRotateRight == null || vertClawGripper == null) return;

        // Rotation control (left bumper/trigger)
        if (gamepad2.left_bumper) {
            vertClawRotateLeft.setPosition(VERT_CLAW_PARALLEL);
            vertClawRotateRight.setPosition(1 - VERT_CLAW_PARALLEL);
        } else if (gamepad2.left_trigger > 0.5) {
            vertClawRotateLeft.setPosition(VERT_CLAW_ROTATED);
            vertClawRotateRight.setPosition(1 - VERT_CLAW_ROTATED);
        }

        // Gripper control (Y button)
        if (gamepad2.y) {
            vertClawGripper.setPosition(CLAW_CLOSED);
        } else if (gamepad2.x) {
            vertClawGripper.setPosition(CLAW_OPEN);
        }
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
        if (gamepad2.b) {
            horizClawGripper.setPosition(CLAW_CLOSED);
        } else if (gamepad2.a) {
            horizClawGripper.setPosition(CLAW_OPEN);
        }
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




//    private void handleViperSlides() {
//        if (armMotorLeft == null || armMotorRight == null) return;
//
//        double slidePower = -gamepad2.left_stick_y;
//        boolean isManualControl = Math.abs(slidePower) > 0.1;
//
//        // Handle manual control
//        if (isManualControl) {
//            armMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            armMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            double smoothedPower = getSmoothedSlidePower(slidePower);
//            applySlidePower(smoothedPower);
//        }
//        // Handle presets
//        else if (gamepad2.dpad_up) {
//            controller.goTo(SLIDES_HIGH_POSITION, SLIDES_HIGH_POSITION);
//        } else if (gamepad2.dpad_right) {
//            controller.goTo(SLIDES_MEDIUM_POSITION, SLIDES_MEDIUM_POSITION);
//        } else if (gamepad2.dpad_down) {
//            controller.goTo(SLIDES_LOW_POSITION, SLIDES_LOW_POSITION);
//        }
//
//        // Check current limits first
////        if (checkCurrentLimits()) {
////            VertslideState = VertSlideState.ERROR;
////            stopSlides();
////            return;
////        }
//
//        // Check for overcurrent protection
//        if (isOverCurrentProtected) {
//            stopSlides();
//            telemetry.addData("WARNING", "Slide motors current limit exceeded!");
//            return;
//        }
//
////        // Position limits
////        if (currentPosition > SLIDES_MAX_POSITION && slidePower > 0) {
////            slidePower = 0;
////            telemetry.addData("WARNING", "Maximum extension reached!");
////        }
////        if (currentPosition < SLIDES_MIN_POSITION && slidePower < 0) {
////            slidePower = 0;
////            telemetry.addData("WARNING", "Minimum position reached!");
////        }
//
//
//        // State machine for slides
//        switch (slideState) {
//            case IDLE:
//                if (isManualControl) {
//                    slideState = SlideState.MANUAL_CONTROL;
//                }
////                else if (isPresetRequested) {
////                    slideState = SlideState.MOVING_TO_POSITION;
////                    if (gamepad2.dpad_up) targetSlidePosition = SLIDES_HIGH_POSITION;
////                    if (gamepad2.dpad_right) targetSlidePosition = SLIDES_MEDIUM_POSITION;
////                    if (gamepad2.dpad_down) targetSlidePosition = SLIDES_LOW_POSITION;
////                }
//                break;
//
//            case MANUAL_CONTROL:
//                if (!isManualControl) {
//                    slideState = SlideState.IDLE;
//                    stopSlides();
//                } else {
//                    double smoothedPower = getSmoothedSlidePower(slidePower);
//                    applySlidePower(smoothedPower);
//                }
//                break;
//
//            case MOVING_TO_POSITION:
//                if (isManualControl) {
//                    slideState = SlideState.MANUAL_CONTROL;
//                } else if (isAtTargetPosition()) {
//                    slideState = SlideState.IDLE;
//                    stopSlides();
//                } else {
//                    moveToTargetPosition();
//                }
//                break;
//
//            case ERROR:
////                if (!checkCurrentLimits()) {
////                    slideState = SlideState.IDLE;
////                }
//                break;
//        }
//
//        // Position telemetry can stay if needed
//        if(armMotorLeft.getCurrentPosition()==5000 && armMotorRight.getCurrentPosition()==5000) {
//            telemetry.addData("Slider Pos: ", "ok");
//        }
//    }
//
//    private void handleRotation() {
//        double rotatePower = -gamepad2.right_stick_y * ROTATION_MAX_POWER;
//        boolean isManualControl = Math.abs(rotatePower) > 0.1;
//        boolean isPresetRequested = gamepad2.left_bumper || gamepad2.right_bumper || gamepad2.left_trigger > 0.5;
//        int currentRotation = (rotateMotorLeft.getCurrentPosition() + rotateMotorRight.getCurrentPosition()) / 2;
//        int slidePosition = (armMotorLeft.getCurrentPosition() + armMotorRight.getCurrentPosition()) / 2;
//
//        // Check current limits first
////        if (checkRotationCurrentLimits()) {
////            rotationState = RotationState.ERROR;
////            stopRotation();
////            telemetry.addData("WARNING", "Rotation motors current limit exceeded!");
////            return;
////        }
//
//        // Check for overcurrent protection
//        if (isOverCurrentProtected) {
//            stopRotation();
//            telemetry.addData("WARNING", "Rotation motors current limit exceeded!");
//            return;
//        }
//
//        // Safety check for rotation when slides are retracted
//        if (slidePosition < SAFE_ROTATION_EXTENSION && currentRotation < ROTATION_HORIZONTAL
//                && Math.abs(rotatePower) > 0.1) {
//            telemetry.addData("WARNING", "Extend slides before rotating!");
//            rotatePower = 0;
//        }
//
//        // Prevent rotation past limits
//        if ((currentRotation >= ROTATION_VERTICAL && rotatePower > 0) ||
//                (currentRotation <= ROTATION_HORIZONTAL && rotatePower < 0)) {
//            rotatePower = 0;
//            telemetry.addData("WARNING", "Rotation limit reached!");
//        }
//
//        // State machine for rotation
//        switch (rotationState) {
//            case IDLE:
//                stopRotation();
//                if (isManualControl) {
//                    rotationState = RotationState.MANUAL_CONTROL;
//                } else if (isPresetRequested) {
//                    rotationState = RotationState.MOVING_TO_POSITION;
//                    if (gamepad2.left_bumper) targetRotationPosition = ROTATION_HORIZONTAL;
//                    if (gamepad2.right_bumper) targetRotationPosition = ROTATION_VERTICAL;
//                    if (gamepad2.left_trigger > 0.5) targetRotationPosition = ROTATION_45_DEGREES;
//                }
//                break;
//
//            case MANUAL_CONTROL:
//                if (!isManualControl) {
//                    rotationState = RotationState.IDLE;
//                    stopRotation();
//                } else {
//                    double smoothedPower = getSmoothedRotationPower(rotatePower);
//                    applyRotationPower(smoothedPower);
//                }
//                break;
//
//            case MOVING_TO_POSITION:
//                if (isManualControl) {
//                    rotationState = RotationState.MANUAL_CONTROL;
//                } else if (isAtRotationTarget()) {
//                    rotationState = RotationState.IDLE;
//                    stopRotation();
//                } else {
//                    rotateViperSlidesTo(targetRotationPosition);
//                }
//                break;
//
//            case ERROR:
////                if (!checkRotationCurrentLimits()) {
////                    rotationState = RotationState.IDLE;
////                }
//                break;
//        }
//    }

//    private void handleIntake() {
//        // Early returns if servos disabled
//        if (!(HardwareConfig.ENABLE_LEFT_AXLE && HardwareConfig.ENABLE_RIGHT_AXLE)) {
//            return;
//        }
//
//        // Axle rotation control using gamepad2 buttons
//        if (gamepad2.x) {  // Intake position
//            leftAxleServo.setPosition(AXLE_INTAKE_POSITION);
//            rightAxleServo.setPosition(1 - AXLE_INTAKE_POSITION);  // Reverse for opposite side
//        } else if (gamepad2.y) {  // Deposit position
//            leftAxleServo.setPosition(AXLE_DEPOSIT_POSITION);
//            rightAxleServo.setPosition(1 - AXLE_DEPOSIT_POSITION);  // Reverse for opposite side
//        }
//
//        // Early return for gecko servos
//        if (!(HardwareConfig.ENABLE_LEFT_GECKO && HardwareConfig.ENABLE_RIGHT_GECKO)) {
//            return;
//        }
//
//        // Gecko wheel control using right trigger for intake, left trigger for outtake
//        if (gamepad2.right_trigger > 0.1) {  // Intake
//            leftGeckoServo.setPosition(GECKO_WHEEL_INTAKE);
//            rightGeckoServo.setPosition(1 - GECKO_WHEEL_INTAKE);  // Reverse for opposite direction
//        } else if (gamepad2.left_trigger > 0.1) {  // Outtake
//            leftGeckoServo.setPosition(GECKO_WHEEL_OUTTAKE);
//            rightGeckoServo.setPosition(1 - GECKO_WHEEL_OUTTAKE);  // Reverse for opposite direction
//        } else {  // Stop
//            leftGeckoServo.setPosition(GECKO_WHEEL_STOP);
//            rightGeckoServo.setPosition(GECKO_WHEEL_STOP);
//        }
//    }

//    private void moveViperSlidesTo(int targetPosition) {
//        // Safety bounds check
//        targetPosition = Range.clip(targetPosition, SLIDES_MIN_POSITION, SLIDES_MAX_POSITION);
//
//        // Check if rotation angle allows this extension
//        int rotationPosition = (rotateMotorLeft.getCurrentPosition() + rotateMotorRight.getCurrentPosition()) / 2;
//        if (rotationPosition > ROTATION_45_DEGREES && targetPosition > MAX_EXTENSION_AT_ANGLE) {
//            targetPosition = MAX_EXTENSION_AT_ANGLE;
//            telemetry.addData("WARNING", "Extension limited due to rotation angle!");
//        }
//
//        int currentPosition = (armMotorLeft.getCurrentPosition() + armMotorRight.getCurrentPosition()) / 2;
//        double power = calculatePID(targetPosition, currentPosition, SLIDES_P, SLIDES_I, SLIDES_D,
//                slidesTimer, lastSlidesError, slidesIntegralSum);
//
//        // Apply power with safety limit
//        power = Range.clip(power, -SLIDES_MAX_POWER, SLIDES_MAX_POWER);
//        if (!isOverCurrentProtected) {
//            armMotorLeft.setPower(power);
//            armMotorRight.setPower(power);
//        }
//    }

//    private void rotateViperSlidesTo(int targetPosition) {
//        // Safety bounds check
//        targetPosition = Range.clip(targetPosition, ROTATION_HORIZONTAL, ROTATION_VERTICAL);
//
//        int currentPosition = (rotateMotorLeft.getCurrentPosition() + rotateMotorRight.getCurrentPosition()) / 2;
//        double power = calculatePID(targetPosition, currentPosition, ROTATION_P, ROTATION_I, ROTATION_D,
//                rotationTimer, lastRotationError, rotationIntegralSum);
//
//        // Update PID variables
//        lastRotationError = currentPosition - targetPosition;
//
//        // Apply power with safety limit
//        power = Range.clip(power, -ROTATION_MAX_POWER, ROTATION_MAX_POWER);
//        if (!isOverCurrentProtected) {
//            applyRotationPower(power);
//        }
//    }

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

//    private boolean checkCurrentLimits() {
//        double leftCurrent = armMotorLeft.getCurrent();  // Just use getCurrent() directly
//        double rightCurrent = armMotorRight.getCurrent();
//        return leftCurrent > CURRENT_LIMIT_SLIDES || rightCurrent > CURRENT_LIMIT_SLIDES;
//    }
//
//    private boolean checkRotationCurrentLimits() {
//        double leftCurrent = rotateMotorLeft.getCurrent();
//        double rightCurrent = rotateMotorRight.getCurrent();
//        return leftCurrent > CURRENT_LIMIT_ROTATION || rightCurrent > CURRENT_LIMIT_ROTATION;
//    }

//    private void stopSlides() {
//        armMotorLeft.setPower(0);
//        armMotorRight.setPower(0);
//    }

//    private double getSmoothedSlidePower(double rawPower) {
//        double safePower = Range.clip(rawPower, -SLIDES_MAX_POWER, SLIDES_MAX_POWER);
//        if (Math.abs(safePower - lastSlidePower) > 0.5) {
//            safePower = (safePower + lastSlidePower) / 2;
//        }
//        lastSlidePower = safePower;
//        return safePower;
//    }

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

    private void updateTelemetry() {
        telemetry.addData("Left Slide Position", armMotorLeft.getCurrentPosition());
        telemetry.addData("Right Slide Position", armMotorRight.getCurrentPosition());
        telemetry.addData("Left Rotation Position", rotateMotorLeft.getCurrentPosition());
        telemetry.addData("Right Rotation Position", rotateMotorRight.getCurrentPosition());

        // Only show servo positions if enabled
        if (HardwareConfig.ENABLE_LEFT_AXLE) {
            telemetry.addData("Left Axle Position", leftAxleServo.getPosition());
        }
        if (HardwareConfig.ENABLE_RIGHT_AXLE) {
            telemetry.addData("Right Axle Position", rightAxleServo.getPosition());
        }
        if (HardwareConfig.ENABLE_LEFT_GECKO) {
            telemetry.addData("Left Gecko Position", leftGeckoServo.getPosition());
        }
        if (HardwareConfig.ENABLE_RIGHT_GECKO) {
            telemetry.addData("Right Gecko Position", rightGeckoServo.getPosition());
        }

        if (isOverCurrentProtected) {
            telemetry.addLine("⚠️ OVERCURRENT PROTECTION ACTIVE ⚠️");
        }

        // Drive System
        telemetry.addLine("=== Drive System ===");
        telemetry.addData("Pose Estimate", drive.getPoseEstimate());

        // Vertical Slides
        telemetry.addLine("=== Vertical Slides ===");
        if (vertSlideLeft != null && vertSlideRight != null) {
            telemetry.addData("Left Position", vertSlideLeft.getCurrentPosition());
            telemetry.addData("Right Position", vertSlideRight.getCurrentPosition());
            telemetry.addData("State", vertSlideState);
            if (vertSlideState == VertSlideState.MOVING_TO_POSITION) {
                telemetry.addData("Target Position", targetVertPosition);
            }
        }

        // Horizontal Slides
        telemetry.addLine("=== Horizontal Slides ===");
        if (horizSlideLeft != null && horizSlideRight != null) {
            telemetry.addData("Left Position", horizSlideLeft.getPosition());
            telemetry.addData("Right Position", horizSlideRight.getPosition());
            telemetry.addData("Current Position", currentHorizPosition);
        }

        // Vertical Claw
        telemetry.addLine("=== Vertical Claw ===");
        if (vertClawRotateLeft != null && vertClawRotateRight != null && vertClawGripper != null) {
            telemetry.addData("Rotate Left", vertClawRotateLeft.getPosition());
            telemetry.addData("Rotate Right", vertClawRotateRight.getPosition());
            telemetry.addData("Gripper", vertClawGripper.getPosition());
        }

        // Horizontal Claw
        telemetry.addLine("=== Horizontal Claw ===");
        if (horizClawRotateLeft != null && horizClawRotateRight != null && horizClawGripper != null) {
            telemetry.addData("Rotate Left", horizClawRotateLeft.getPosition());
            telemetry.addData("Rotate Right", horizClawRotateRight.getPosition());
            telemetry.addData("Gripper", horizClawGripper.getPosition());
        }

        telemetry.update();
    }


}
