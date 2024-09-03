package org.firstinspires.ftc.teamcode.Autonomous;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SleeveDetection extends OpenCvPipeline {
    /*
    YELLOW  = Parking Left
    CYAN    = Parking Middle
    MAGENTA = Parking Right
     */
    public enum RedPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    // Screen dimensions (assuming a standard 640x480 stream)
    private static final int IMG_WIDTH = 432;
    private static final int IMG_HEIGHT = 240;

    // Divide the screen into three regions (left, center, right)
    private static final int REGION_WIDTH = IMG_WIDTH / 3;
    private static final int REGION_HEIGHT = IMG_HEIGHT;

    // Running variable storing the position with the most red
    private volatile RedPosition redPosition = RedPosition.LEFT;
    // Running variable storing the red position
    @Override
    public Mat processFrame(Mat input) {
        // Calculate average red value in each section
        double redSumLeft = calculateAverageRed(input, 0);
        double redSumCenter = calculateAverageRed(input, REGION_WIDTH);
        double redSumRight = calculateAverageRed(input, 2 * REGION_WIDTH);

        // Determine the region with the most red
        if (redSumLeft > redSumCenter && redSumLeft > redSumRight) {
            redPosition = RedPosition.LEFT;
        } else if (redSumCenter > redSumRight) {
            redPosition = RedPosition.CENTER;
        } else {
            redPosition = RedPosition.RIGHT;
        }

        return input;
    }

    private double calculateAverageRed(Mat image, int startX) {
        // Extract region
        Rect region = new Rect(startX, 0, REGION_WIDTH, REGION_HEIGHT);
        Mat regionMat = image.submat(region);

        // Calculate average
        Scalar averageColor = Core.mean(regionMat);
        regionMat.release();

        // Return the red value (assuming BGR format)
        return averageColor.val[2];
    }

    public RedPosition getPosition() {
        return redPosition;
    }
}