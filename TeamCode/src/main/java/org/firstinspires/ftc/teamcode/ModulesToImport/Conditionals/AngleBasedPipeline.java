package org.firstinspires.ftc.teamcode.ModulesToImport.Conditionals;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class AngleBasedPipeline extends OpenCvPipeline {

    // Visualization colors
    static final Scalar GREEN = new Scalar(0, 255, 0);
    static final Scalar RED = new Scalar(255, 0, 0);

    // Region for cropping the frame
    static final int CROP_WIDTH = 60;
    static final int CROP_HEIGHT = 20;
    static final int OFFSET_X = 30; // Shift right (+) or left (-)
    static final int OFFSET_Y = -10; // Shift down (+) or up (-)

    Mat HSV = new Mat();
    Scalar averageHSV = new Scalar(0, 0, 0);

    String dominantColor = "Unknown";
    double blockAngle = -1.0;

    @Override
    public Mat processFrame(Mat input) {
        // Crop the frame
        int startX = ((input.width() - CROP_WIDTH) / 2) + OFFSET_X;
        int startY = ((input.height() - CROP_HEIGHT) / 2) + OFFSET_Y;
        startX = Math.max(0, Math.min(startX, input.width() - CROP_WIDTH));
        startY = Math.max(0, Math.min(startY, input.height() - CROP_HEIGHT));
        Rect cropRect = new Rect(startX, startY, CROP_WIDTH, CROP_HEIGHT);
        Mat croppedInput = input.submat(cropRect);

        // Detect color
        Imgproc.cvtColor(croppedInput, HSV, Imgproc.COLOR_RGB2HSV);
        averageHSV = Core.mean(HSV);
        dominantColor = detectColor(averageHSV);

        // Detect contours and compute orientation
        Mat gray = new Mat();
        Imgproc.cvtColor(croppedInput, gray, Imgproc.COLOR_RGB2GRAY);

        Mat binary = new Mat();
        Imgproc.threshold(gray, binary, 100, 255, Imgproc.THRESH_BINARY);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(binary, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Find the largest contour to determine the block's orientation
        if (!contours.isEmpty()) {
            MatOfPoint largestContour = contours.get(0);
            double largestArea = 0;

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > largestArea) {
                    largestArea = area;
                    largestContour = contour;
                }
            }

            // Calculate the bounding box and angle
            RotatedRect boundingBox = Imgproc.minAreaRect(new MatOfPoint2f(largestContour.toArray()));
            blockAngle = boundingBox.angle;

            // Adjust angle to be within [0, 180]
            if (blockAngle < -45) {
                blockAngle += 180;
            }

            // Draw the bounding box on the cropped input for visualization
            Point[] boxPoints = new Point[4];
            boundingBox.points(boxPoints);
            for (int i = 0; i < 4; i++) {
                Imgproc.line(croppedInput, boxPoints[i], boxPoints[(i + 1) % 4], GREEN, 2);
            }

            Imgproc.putText(
                    croppedInput,
                    "Angle: " + String.format("%.2f", blockAngle),
                    new Point(10, 20),
                    Imgproc.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    RED,
                    1
            );
        }

        return croppedInput; // Return the cropped frame with annotations
    }

    private String detectColor(Scalar avgHSV) {
        double hue = avgHSV.val[0];
        if (hue < 23 || hue > 160) {
            return "Red";
        } else if (hue > 23 && hue < 40) {
            return "Yellow";
        } else if (hue > 100 && hue < 140) {
            return "Blue";
        }
        return "Unknown";
    }

    public String getDominantColor() {
        return dominantColor;
    }

    public double getBlockAngle() {
        return blockAngle;
    }
}
