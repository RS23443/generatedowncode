package org.firstinspires.ftc.teamcode.ModulesToImport.GeneratedCode;

import org.firstinspires.ftc.teamcode.ModulesToImport.GeneratedCode.PathingAlgorithm;

import java.util.List;
import java.util.ArrayList;

public class SplineGenerator {

    // Represents a single cubic spline segment
    public static class SplineSegment {
        double ax, bx, cx, dx; // Coefficients for x
        double ay, by, cy, dy; // Coefficients for y
        double ah, bh, ch, dh; // Coefficients for heading

        public SplineSegment(double ax, double bx, double cx, double dx,
                             double ay, double by, double cy, double dy,
                             double ah, double bh, double ch, double dh) {
            this.ax = ax; this.bx = bx; this.cx = cx; this.dx = dx;
            this.ay = ay; this.by = by; this.cy = cy; this.dy = dy;
            this.ah = ah; this.bh = bh; this.ch = ch; this.dh = dh;
        }

        // Evaluate the spline at parameter t (0 <= t <= 1)
        public double[] evaluate(double t) {
            double x = ax + bx * t + cx * t * t + dx * t * t * t;
            double y = ay + by * t + cy * t * t + dy * t * t * t;
            double heading = ah + bh * t + ch * t * t + dh * t * t * t;
            return new double[]{x, y, heading};
        }
    }

    // Generate cubic spline segments from a list of points
    public static List<SplineSegment> generateSpline(List<PathingAlgorithm.Point> points) {
        List<SplineSegment> segments = new ArrayList<>();

        for (int i = 0; i < points.size() - 1; i++) {
            PathingAlgorithm.Point p0 = points.get(i);
            PathingAlgorithm.Point p1 = points.get(i + 1);

            // Extract world coordinates and heading from points
            double[] p0World = p0.toWorldCoordinate(); // {x, y} in inches
            double[] p1World = p1.toWorldCoordinate(); // {x, y} in inches

            // Calculate cubic spline coefficients
            double[] xCoefficients = computeCubicCoefficients(p0World[0], p1World[0]);
            double[] yCoefficients = computeCubicCoefficients(p0World[1], p1World[1]);
            double[] headingCoefficients = computeCubicCoefficients(p0.heading, p1.heading);

            // Add the segment to the list
            segments.add(new SplineSegment(
                    xCoefficients[0], xCoefficients[1], xCoefficients[2], xCoefficients[3],
                    yCoefficients[0], yCoefficients[1], yCoefficients[2], yCoefficients[3],
                    headingCoefficients[0], headingCoefficients[1], headingCoefficients[2], headingCoefficients[3]
            ));
        }

        return segments;
    }

    // Compute cubic spline coefficients for a single dimension
    private static double[] computeCubicCoefficients(double p0, double p1) {
        double a = p0;  // Starting position
        double b = 0;   // Initial velocity
        double c = 3 * (p1 - p0);
        double d = -2 * (p1 - p0);
        return new double[]{a, b, c, d};
    }
}
