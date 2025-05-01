package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Placer extends SubsystemBase {

  public static Placer instance;

  public static Placer getInstance() {
    if (instance == null) {
      instance = new Placer();
    }
    return instance;
  }

  private SparkMax motor1 = new SparkMax(32, MotorType.kBrushless);
  private SparkMax motor2 = new SparkMax(33, MotorType.kBrushless);

  private DigitalInput coralSensor = new DigitalInput(0);
  public boolean seenCoral = false;

  public boolean hasCoral() {
    return seenCoral && !coralSensor.get();
  }

  public boolean coralGoingThrough = false;
  private Trigger coralSensorTrigger = new Trigger(() -> coralSensor.get());

  public Placer() {
    var config = new SparkMaxConfig();
    config.closedLoop.p(1);
    config.closedLoop.i(0);
    config.closedLoop.d(0);
    motor1.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motor2.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    coralSensorTrigger
      .debounce(0.3, DebounceType.kRising)
      .onTrue(new InstantCommand(() -> {
        seenCoral = false;
        coralGoingThrough = true;
      }).ignoringDisable(true))
      .onFalse(new InstantCommand(() -> {
        if(!DriverStation.isAutonomous()) {
          stop();
        }
        seenCoral = true;
        coralGoingThrough = false;
      }).ignoringDisable(true));
  }

  public void stop() {
    seenCoral = false;
    motor1.set(0);
    motor2.set(0);
  }

  public void intake() {
    if(coralSensor.get() && coralGoingThrough) {
      seenCoral = true;
    }
    motor1.set(-0.25);
    motor2.set(0.25);
  }

  public void spitCoral() {
    double speed = Elevator.getHeight() > Elevator.MAX_HEIGHT - 0.3 ? 0.45 : 0.6;
    motor1.set(-speed);
    motor2.set(speed);
    LEDs.set(LEDs.getDefaultColor());
  }

  public void outtake() {
    double speed = Elevator.getHeight() > Elevator.MAX_HEIGHT - 0.5 ? 0.5 : 1.0;
    motor1.set(speed);
    motor2.set(-speed);
    LEDs.set(LEDs.getDefaultColor());
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Placer/Has Coral", hasCoral());
    SmartDashboard.putBoolean("Placer/Seen Coral", seenCoral);

    SmartDashboard.putBoolean("Placer/Coral Going Through", coralGoingThrough);

    // var goalHeight = SmartDashboard.getNumber("Elevator/Goal Height", 0.0);
    // var manuallyMoving = SmartDashboard.getBoolean("Elevator/Manual Moving", false);

    // if(Math.abs(goalHeight - Elevator2.INTAKE_HEIGHT) > 0.1 || manuallyMoving) {
    //   motor1.set(0);
    //   motor2.set(0);
    // }
  }
}
