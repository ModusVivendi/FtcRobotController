package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
@Disabled
@Autonomous(name = "Custom TFOD OpMode")
public class CustomTFOD extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "prop.tflite";
    private static final String[] LABELS = {"left", "center", "right"};
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        initTfod();
        if (tfod != null) {
            tfod.activate();
        }

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    List<Recognition> recognitions = tfod.getUpdatedRecognitions();
                    if (recognitions != null) {
                        telemetry.addData("# Object Detected", recognitions.size());
                        int i = 0;
                        for (Recognition recognition : recognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());

                            // Check label position
                            if (recognition.getLeft() < recognition.getImageWidth() / 3) {
                                telemetry.addData("Object Position", "Left");
                            } else if (recognition.getLeft() > 2 * recognition.getImageWidth() / 3) {
                                telemetry.addData("Object Position", "Right");
                            } else {
                                telemetry.addData("Object Position", "Center");
                            }
                        }
                        telemetry.update();
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    private void initTfod() {
//        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
//                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
//        tfodParameters.minResultConfidence = 0.8f;
//        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, hardwareMap.get(WebcamName.class, "Webcam 1"));
//        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}
