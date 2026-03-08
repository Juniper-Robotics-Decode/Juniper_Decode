package org.firstinspires.ftc.teamcode.Swerve.Geo;

public class MathUtils {

    /**
     * Finds the maximum value in an array of doubles.
     * Used primarily for normalizing wheel speeds in the SwerveDrivetrain.
     * * @param values An array of double values (e.g., wheel speeds).
     * @return The highest value found in the array.
     */
    public static double max(double[] values) {
        if (values == null || values.length == 0) {
            return 0.0;
        }

        double maxValue = values[0];
        for (int i = 1; i < values.length; i++) {
            if (values[i] > maxValue) {
                maxValue = values[i];
            }
        }
        return maxValue;
    }
}