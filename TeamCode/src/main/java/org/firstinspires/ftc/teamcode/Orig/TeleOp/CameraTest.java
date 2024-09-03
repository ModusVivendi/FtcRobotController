package org.firstinspires.ftc.teamcode.Orig.TeleOp;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@TeleOp(name = "CameraTest", group = "Test")
public class CameraTest extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private static final String TFOD_MODEL_FILE = "/data/local/tmp/orig.tflite";
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

    public void runOpMode() throws InterruptedException {
        initTfod();
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                telemetryTfod();

                // Push telemetry to the Driver Station.
                telemetry.update();

                // Save CPU resources; can resume streaming when needed.
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }

                // Share the CPU.
                sleep(20);
            }
        }

        // Save more CPU resources when camera is no longer needed.
        //visionPortal.close();
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

    }   // end method initTfod()

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
                position = "Right";
            } else if (objectMidpoint < (2 * imageWidth / 3)) {
                position = "Center";
            } else {
                position = "Left";
            }

            telemetry.addData("Classified Image", "%s (%.0f %% Conf.)",
                    classification.getLabel(), classification.getConfidence() * 100);
            telemetry.addData("Position", position);
        } else {
            telemetry.addData("No Classification", "");
        }
    }  // end method telemetryTfod()
}
