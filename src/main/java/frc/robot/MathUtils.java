package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;

public class MathUtils {
  public static double clamp(double value, double min, double max) {
    return Math.min(Math.max(value, min), max);
  }

  public static double smoothInput(double input, double power) {
    return Math.abs(Math.pow(input, power)) * Math.signum(input);
  }

  public static double downwardCurve(double x, double minY, double maxX) {
    double p = (minY - maxX) / (1 - minY);

    return p / (x + p);
  }

  public static Rotation2d shortestRotationDifference(
    Rotation2d start,
    Rotation2d end
  ) {
    double startRad = start.getRadians();
    double endRad = end.getRadians();

    double diff = endRad - startRad;
    double shortest = Math.atan2(
      Math.sin(diff),
      Math.cos(diff)
    );

    return new Rotation2d(shortest);
  }
}