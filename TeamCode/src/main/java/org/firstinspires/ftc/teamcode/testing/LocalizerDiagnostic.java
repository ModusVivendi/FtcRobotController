package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

@TeleOp(name="Localizer Diagnostic", group = "drive")
public class LocalizerDiagnostic extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            // Use a single axis at a time
            double forward = 0;
            double strafe = 0;
            double turn = 0;

            // Only use one control at a time
            if (Math.abs(gamepad1.left_stick_y) > 0.5) {
                forward = -gamepad1.left_stick_y;
                telemetry.addLine("TESTING FORWARD/BACKWARD");
            } else if (Math.abs(gamepad1.left_stick_x) > 0.5) {
                strafe = -gamepad1.left_stick_x;
                telemetry.addLine("TESTING STRAFE");
            } else if (Math.abs(gamepad1.right_stick_x) > 0.5) {
                turn = -gamepad1.right_stick_x;
                telemetry.addLine("TESTING ROTATION");
            }

            drive.setWeightedDrivePower(new Pose2d(forward, strafe, turn));
            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();

            // Clear display for readability
            telemetry.addLine("=== PUSH ONE STICK AT A TIME ===");
            telemetry.addLine("Only use left stick Y OR left stick X OR right stick X");
            telemetry.addLine("");

            // Display control inputs
            telemetry.addData("Forward input", forward);
            telemetry.addData("Strafe input", strafe);
            telemetry.addData("Turn input", turn);
            telemetry.addLine("");

            // Display expected vs actual movement
            telemetry.addData("X position (should increase with forward)", poseEstimate.getX());
            telemetry.addData("Y position (should increase with right strafe)", poseEstimate.getY());
            telemetry.addData("Heading (should decrease with clockwise turn)", poseEstimate.getHeading());

            telemetry.addLine("");
            telemetry.addData("X change rate",
                    Math.abs(poseEstimate.getX()) > 0.05 ? "ACTIVE" : "---");
            telemetry.addData("Y change rate",
                    Math.abs(poseEstimate.getY()) > 0.05 ? "ACTIVE" : "---");
            telemetry.addData("Heading change",
                    Math.abs(poseEstimate.getHeading()) > 0.05 ? "ACTIVE" : "---");

            telemetry.update();
        }
    }
}
