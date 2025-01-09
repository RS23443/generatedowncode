package org.firstinspires.ftc.teamcode.ModulesToImport.Conditionals;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class EnhancedPipeline extends OpenCvPipeline {

    // Visualization colors
    static final Scalar GREEN = new Scalar(0, 255, 0);
    static final Scalar RED = new Scalar(255, 0, 0);

    // Block attributes
    String dominantColor = "Unknown";
    String orientation = "Unknown";

    @Override
    public Mat processFrame(Mat input) {
        // Define the cropping region (focus on the middle of the field)
        int cropWidth = 80;  // Width of the cropped region
        int cropHeight = 60; // Height of the cropped region
        int cropStartX = (input.width() - cropWidth) / 2;  // Centered horizontally
        int cropStartY = (input.height() - cropHeight) / 2; // Centered vertically

        Rect cropRect = new Rect(cropStartX, cropStartY, cropWidth, cropHeight);
        Mat croppedField = input.submat(cropRect);

        // Convert cropped field to grayscale for contour detection
        Mat gray = new Mat();
        Imgproc.cvtColor(croppedField, gray, Imgproc.COLOR_RGB2GRAY);

        // Threshold the grayscale image
        Mat binary = new Mat();
        Imgproc.threshold(gray, binary, 100, 255, Imgproc.THRESH_BINARY);

        // Find contours in the binary image
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(binary, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        if (!contours.isEmpty()) {
            MatOfPoint largestContour = contours.get(0);
            double largestArea = 0;

            // Find the largest contour
            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > largestArea) {
                    largestArea = area;
                    largestContour = contour;
                }
            }

            // Calculate the bounding rectangle
            Rect boundingRect = Imgproc.boundingRect(largestContour);
            double aspectRatio = (double) boundingRect.width / boundingRect.height;

            // Determine orientation based on aspect ratio
            if (aspectRatio > 1.3) {
                orientation = "Horizontal";
            } else if (aspectRatio < 1.3) {
                orientation = "Vertical";
            } else {
                orientation = "Diagonal";
            }

            // Determine color
            Mat hsv = new Mat();
            Imgproc.cvtColor(croppedField, hsv, Imgproc.COLOR_RGB2HSV);
            Mat blockRegion = hsv.submat(boundingRect);
            Scalar avgHSV = Core.mean(blockRegion);
            dominantColor = detectColor(avgHSV);
            blockRegion.release();

            // Draw the bounding rectangle and annotate it
            Point rectStart = new Point(cropStartX + boundingRect.x, cropStartY + boundingRect.y);
            Point rectEnd = new Point(rectStart.x + boundingRect.width, rectStart.y + boundingRect.height);
            Imgproc.rectangle(input, rectStart, rectEnd, GREEN, 2);

            Imgproc.putText(input, "Color: " + dominantColor,
                    new Point(rectStart.x, rectStart.y - 20),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, RED, 2);
            Imgproc.putText(input, "Orientation: " + orientation,
                    new Point(rectStart.x, rectStart.y - 5),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, RED, 2);
        }

        return input;
    }

    /**
     * Classify color based on average HSV values.
     */
    private String detectColor(Scalar avgHSV) {
        double hue = avgHSV.val[0];
        if (hue < 15 || hue > 160) {
            return "Red";
        } else if (hue > 15 && hue < 40) {
            return "Yellow";
        } else if (hue > 100 && hue < 140) {
            return "Blue";
        }
        return "Unknown";
    }

    public String getDominantColor() {
        return dominantColor;
    }

    public String getOrientation() {
        return orientation;
    }
}

