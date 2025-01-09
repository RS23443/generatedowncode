package org.firstinspires.ftc.teamcode.ModulesToImport.Conditionals;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class CombinedDetectionPipeline extends OpenCvPipeline {

    // Visualization colors
    static final Scalar GREEN = new Scalar(0, 255, 0);
    static final Scalar RED = new Scalar(255, 0, 0);

    // Region for color detection
    static final Point REGION_TOPLEFT_ANCHOR_POINT = new Point(140, 100);
    static final int REGION_WIDTH = 40;
    static final int REGION_HEIGHT = 40;

    Point region_pointA = new Point(
            REGION_TOPLEFT_ANCHOR_POINT.x,
            REGION_TOPLEFT_ANCHOR_POINT.y);
    Point region_pointB = new Point(
            REGION_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    Mat HSV = new Mat();
    Mat region_HSV;
    Scalar averageHSV = new Scalar(0, 0, 0);

    String dominantColor = "Unknown";
    String orientation = "Unknown";

    @Override
    public void init(Mat firstFrame) {
        // Convert first frame to HSV for color analysis
        Imgproc.cvtColor(firstFrame, HSV, Imgproc.COLOR_RGB2HSV);
        region_HSV = HSV.submat(new Rect(region_pointA, region_pointB));
    }

    @Override
    public Mat processFrame(Mat input) {
        // Define the cropped region (e.g., center 160x120 area)
        int cropWidth = 160;
        int cropHeight = 120;

        // Adjust for offsets to fine-tune the center
        int offsetX = 30; // Shift right (+) or left (-)
        int offsetY = -10; // Shift down (+) or up (-)

        int startX = ((input.width() - cropWidth) / 2) + offsetX;
        int startY = ((input.height() - cropHeight) / 2) + offsetY;

        // Ensure cropping doesn't exceed frame bounds
        startX = Math.max(0, Math.min(startX, input.width() - cropWidth));
        startY = Math.max(0, Math.min(startY, input.height() - cropHeight));

        Rect cropRect = new Rect(startX, startY, cropWidth, cropHeight);
        Mat croppedInput = input.submat(cropRect);

        // Detect color in the cropped frame
        Imgproc.cvtColor(croppedInput, HSV, Imgproc.COLOR_RGB2HSV);
        averageHSV = Core.mean(HSV);
        dominantColor = detectColor(averageHSV);

        // Detect orientation in the cropped frame
        Mat gray = new Mat();
        Imgproc.cvtColor(croppedInput, gray, Imgproc.COLOR_RGB2GRAY);

        Mat binary = new Mat();
        Imgproc.threshold(gray, binary, 100, 255, Imgproc.THRESH_BINARY);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(binary, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint contour : contours) {
            // Get bounding rectangle
            Rect boundingRect = Imgproc.boundingRect(contour);

            // Draw rectangle for visualization
            Imgproc.rectangle(croppedInput, boundingRect, GREEN, 2);

            // Calculate aspect ratio for orientation
            double aspectRatio = (double) boundingRect.width / boundingRect.height;
            if (aspectRatio > 1.25) {
                orientation = "Horizontal";
                Imgproc.putText(croppedInput, "Horizontal", new Point(boundingRect.x, boundingRect.y - 10),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, RED, 2);
            } else if (aspectRatio < 1.25) {
                orientation = "Vertical";
                Imgproc.putText(croppedInput, "Vertical", new Point(boundingRect.x, boundingRect.y - 10),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, RED, 2);
            } else {
                orientation = "Unknown";
            }
        }

        return croppedInput; // Return the cropped input for display
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

    public String getOrientation() {
        return orientation;
    }
}

// bigger frame of reference:
/*
public  class CombinedDetectionPipeline extends OpenCvPipeline {

        // Visualization colors
        static final Scalar GREEN = new Scalar(0, 255, 0);
        static final Scalar RED = new Scalar(255, 0, 0);

        // Region for color detection
        static final Point REGION_TOPLEFT_ANCHOR_POINT = new Point(140, 100);
        static final int REGION_WIDTH = 40;
        static final int REGION_HEIGHT = 40;

        Point region_pointA = new Point(
                REGION_TOPLEFT_ANCHOR_POINT.x,
                REGION_TOPLEFT_ANCHOR_POINT.y);
        Point region_pointB = new Point(
                REGION_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        Mat HSV = new Mat();
        Mat region_HSV;
        Scalar averageHSV = new Scalar(0, 0, 0);

        String dominantColor = "Unknown";
        String orientation = "Unknown";

        @Override
        public void init(Mat firstFrame) {
            // Convert first frame to HSV for color analysis
            Imgproc.cvtColor(firstFrame, HSV, Imgproc.COLOR_RGB2HSV);
            region_HSV = HSV.submat(new Rect(region_pointA, region_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {
            // Detect color
            Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);
            averageHSV = Core.mean(region_HSV);
            dominantColor = detectColor(averageHSV);

            // Detect orientation
            Mat gray = new Mat();
            Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2GRAY);

            Mat binary = new Mat();
            Imgproc.threshold(gray, binary, 100, 255, Imgproc.THRESH_BINARY);

            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(binary, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            for (MatOfPoint contour : contours) {
                // Get bounding rectangle
                Rect boundingRect = Imgproc.boundingRect(contour);

                // Draw rectangle for visualization
                Imgproc.rectangle(input, boundingRect, GREEN, 2);

                // Calculate aspect ratio for orientation
                double aspectRatio = (double) boundingRect.width / boundingRect.height;
                if (aspectRatio > 1.25) {
                    orientation = "Horizontal";
                    Imgproc.putText(input, "Horizontal", new Point(boundingRect.x, boundingRect.y - 10),
                            Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, RED, 2);
                } else if (aspectRatio < 1.25) {
                    orientation = "Vertical";
                    Imgproc.putText(input, "Vertical", new Point(boundingRect.x, boundingRect.y - 10),
                            Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, RED, 2);
                } else {
                    orientation = "Unknown";
                }
            }

            return input; // Return the frame with visual annotations
        }

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
}



 */