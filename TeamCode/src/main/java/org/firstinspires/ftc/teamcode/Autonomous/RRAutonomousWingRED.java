package org.firstinspires.ftc.teamcode.Autonomous;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Functions.ArmEncoder;
import org.firstinspires.ftc.teamcode.Functions.ClawServos;
import org.firstinspires.ftc.teamcode.Functions.GamepadCalc;
import org.firstinspires.ftc.teamcode.Functions.Move;
import org.firstinspires.ftc.teamcode.Functions.TopServos;
import org.firstinspires.ftc.teamcode.Functions.Vacuum;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;
@Disabled
@Autonomous
public class RRAutonomousWingRED extends LinearOpMode {

    private String webcamName = "Webcam";
    private DcMotor leftMotor, rightMotor, leftMotorBack, rightMotorBack, vacumMotor;
    private DcMotor armMotorLeft, armMotorRight;
    private Servo clawServo, rightServo, leftServo;
    private ArmEncoder controller;
    private Move move;
    private Vacuum vacum;
    private ClawServos clawServos;
    private TopServos topServos;
    private boolean status1;
    int armLevel = 0;
    double integralSum = 0;

    String armCurrentDirection = "up";

    String detection = "";
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/prop.tflite";
    private static final String[] LABELS = {
            "Left", "Center", "Right"
    };
    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    GamepadCalc gamepadCalc;
    @Override
    public void runOpMode() throws InterruptedException {
        leftMotor = hardwareMap.dcMotor.get("FL");
        rightMotor = hardwareMap.dcMotor.get("FR");
        leftMotorBack = hardwareMap.dcMotor.get("BL");
        rightMotorBack = hardwareMap.dcMotor.get("BR");
        vacumMotor = hardwareMap.dcMotor.get("AS");
        armMotorLeft = hardwareMap.dcMotor.get("SL");
        armMotorRight = hardwareMap.dcMotor.get("SR");
        leftServo = hardwareMap.servo.get("ASL");
        rightServo = hardwareMap.servo.get("ASR");
        clawServo = hardwareMap.servo.get("CS");
        move = new Move(leftMotor, rightMotor, leftMotorBack, rightMotorBack);
        vacum = new Vacuum(vacumMotor);
//        rotate = new Rotate(leftMotor, rightMotor, leftMotorBack, rightMotorBack);
        clawServos = new ClawServos(clawServo);
        controller = new ArmEncoder(armMotorLeft, armMotorRight);
        topServos = new TopServos(leftServo, rightServo);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        gamepadCalc = new GamepadCalc(this);

        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(PoseStorage.currentPose);


        initTfod();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

            while (!isStarted()) {

                telemetryTfod();

                // Push telemetry to the Driver Station.
                telemetry.update();
            }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();
        waitForStart();



        Pose2d initialPose = new Pose2d(-36, -62, Math.toRadians(90)); // Replace with your desired starting pose
        drive.setPoseEstimate(initialPose);
        if (opModeIsActive()) {
            if (detection == "l") {
                //zone 1 left red
                TrajectorySequence trajectory1RL = drive.trajectorySequenceBuilder(new Pose2d(-36, -62, Math.toRadians(90)))
                        .addDisplacementMarker(() -> {
                            clawServos.Open();
                            topServos.Close();
                        })
                        .lineToLinearHeading(new Pose2d(-40, -32, Math.toRadians(180)))
                        .lineToLinearHeading(new Pose2d(-40, -35, Math.toRadians(180)))
                        .lineTo(new Vector2d(10, -35))
                        .addDisplacementMarker(() -> {
                            clawServos.Open();
                            topServos.Open();
                            armCurrentDirection = "up";
                            controller.goTo(1100, 1100);
                        })
                        .lineToConstantHeading(new Vector2d(52, -25))
                        .waitSeconds(1)
                        //i want these 2 to happen after it arrives at the previous coordinate
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            clawServos.Close();
                        })
                        .waitSeconds(1)
                        .strafeLeft(4)
                        .forward(4)
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            topServos.Close();
                            armCurrentDirection = "down";
                            controller.goTo(0, 0);
                        })
                        .waitSeconds(1)
                        .turn(Math.toRadians(-90))
                        .lineToConstantHeading(new Vector2d(48, -14))
                        .build();
                drive.followTrajectorySequence(trajectory1RL);

            }
            else if (detection == "c") {
                //zone 2 left red
                TrajectorySequence trajectory1RC = drive.trajectorySequenceBuilder(new Pose2d(-36, -62, Math.toRadians(90)))
                        .addDisplacementMarker(() -> {
                            clawServos.Open();
                            topServos.Close();
                        })
                        .lineTo(new Vector2d(-35, -33))
                        .lineTo(new Vector2d(-35, -38))
                        .turn(Math.toRadians(90))
                        .lineTo(new Vector2d(-35, -35))

                        .lineTo(new Vector2d(10, -35))
                        .addDisplacementMarker(() -> {
                            clawServos.Open();
                            topServos.Open();
                            armCurrentDirection = "up";
                            controller.goTo(1100, 1100);
                        })
                        .lineToLinearHeading(new Pose2d(52, -34.5
                                , Math.toRadians(180)))
                        .waitSeconds(1)
                        //i want these 2 to happen after it arrives at the previous coordinate
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            clawServos.Close();
                        })
                        .waitSeconds(1)
                        .strafeRight(4)
                        .forward(4)
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            topServos.Close();
                            armCurrentDirection = "down";
                            controller.goTo(0, 0);
                        })
                        .waitSeconds(1)
                        .turn(Math.toRadians(-90))
                        .lineToConstantHeading(new Vector2d(48, -14))
                                .build();
                drive.followTrajectorySequence(trajectory1RC);

            }
            else {
                TrajectorySequence trajectory1RR = drive.trajectorySequenceBuilder(new Pose2d(-36, -62, Math.toRadians(90)))
                        .addDisplacementMarker(() -> {
                            clawServos.Open();
                            topServos.Close();
                        })
                        .strafeLeft(4)
                        .splineToLinearHeading(new Pose2d(-33, -35), Math.toRadians(0))
                        .back(4)
                        .turn(Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(-36, -12), Math.toRadians(0))
                        .lineToConstantHeading(new Vector2d(20, -12))
                        .addDisplacementMarker(() -> {
                            clawServos.Open();
                            topServos.Open();
                            armCurrentDirection = "up";
                            controller.goTo(1100, 1100);
                        })
                        .lineToConstantHeading(new Vector2d(53, -37.5))
                        .waitSeconds(1)
                        //i want these 2 to happen after it arrives at the previous coordinate
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            clawServos.Close();
                        })
                        .waitSeconds(1)
                        .strafeRight(4)
                        .forward(5)
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            topServos.Close();
                            armCurrentDirection = "down";
                            controller.goTo(0, 0);
                        })
                        .waitSeconds(1)
                        .turn(Math.toRadians(-90))
                        .lineToConstantHeading(new Vector2d(48, -14))
                        .build();
                drive.followTrajectorySequence(trajectory1RR);


            }


        }

    }

    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()
                .setModelFileName(TFOD_MODEL_FILE)
                .setModelLabels(LABELS)
                .setIsModelTensorFlow2(true)
                .setIsModelQuantized(true)
                .setModelInputSize(300)
                .setModelAspectRatio(16.0 / 9.0)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        builder.setAutoStopLiveView(true);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        visionPortal.setProcessorEnabled(tfod, true);

    }

    private void telemetryTfod() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        if (!currentRecognitions.isEmpty()) {
            // Assuming the model returns only one classification result for the whole image
            Recognition classification = currentRecognitions.get(0);

            // Get the image width from the recognition object (assuming it's the same as the camera resolution you set)
            int imageWidth = 640; // For example, if your camera resolution is set to 640x480

            // Calculate the midpoint of the detected object
            float objectMidpoint = (classification.getLeft() + classification.getRight()) / 2;

            // Determine the position of the object
            String position;
            if (objectMidpoint < imageWidth / 3) {
                position = "Left";
                detection = "l";
            } else if (objectMidpoint < (2 * imageWidth / 3)) {
                position = "Center";
                detection = "c";
            } else {
                position = "Right";
                detection = "r";
            }

            telemetry.addData("Classified Image", "%s (%.0f %% Conf.)",
                    classification.getLabel(), classification.getConfidence() * 100);
            telemetry.addData("Position", position);
        } else {
            telemetry.addData("No Classification", "");
        }
    }  // end method telemetryTfod()
}