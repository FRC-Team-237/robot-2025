package frc.robot;

public class MathUtils {
  public static double clamp(double value, double min, double max) {
    return Math.min(Math.max(value, min), max);
  }

  public static double smoothInput(double input, double power) {
    return Math.abs(Math.pow(input, power)) * Math.signum(input);
  }
}