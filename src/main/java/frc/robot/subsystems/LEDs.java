package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PWM;

public class LEDs {

  public static Optional<Double> currentColor = Optional.empty();

  private static final PWM pwm = new PWM(0);

  public static double RAINBOW_GLITTER = -0.89;
  public static double RAINBOW_TWINKLES = -0.55;

  public static double LAVA_WAVES = -0.39;
  public static double OCEAN_WAVES = -0.41;

  public static double BREATH_RED = -0.17;
  public static double BREATH_BLUE = -0.15;
  
  public static double BLINK_RED = -0.11;
  public static double BLINK_BLUE = -0.09;
  public static double BLINK_WHITE = -0.05;

  public static double HOT_PINK = 0.57;
  public static double DARK_RED = 0.59;
  public static double RED = 0.61;
  public static double RED_ORANGE = 0.63;
  public static double ORANGE = 0.65;
  public static double GOLD = 0.67;
  public static double YELLOW = 0.69;
  public static double LAWN_GREEN = 0.71;
  public static double LIME = 0.73;
  public static double DARK_GREEN = 0.75;
  public static double GREEN = 0.77;
  public static double BLUE_GREEN = 0.79;
  public static double AQUA = 0.81;
  public static double SKY_BLUE = 0.83;
  public static double DARK_BLUE = 0.85;
  public static double BLUE = 0.87;
  public static double BLUE_VIOLET = 0.89;
  public static double VIOLET = 0.91;
  public static double WHITE = 0.93;
  public static double GRAY = 0.95;
  public static double DARK_GRAY = 0.97;
  public static double BLACK = 0.99;

  public static double getDefaultColor() {
    var color = DriverStation.getAlliance();
    if(color.isPresent()) {
      if(color.get() == DriverStation.Alliance.Blue) return BLUE;
      if(color.get() == DriverStation.Alliance.Red) return RED;
    }

    return GRAY;
  }

  public static void set(double value) {
    currentColor = Optional.of(value);
    pwm.setSpeed(value);
  }

  public static void off() {
    pwm.setSpeed(0);
  }
}