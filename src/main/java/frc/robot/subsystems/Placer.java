package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Placer extends SubsystemBase {
  private SparkMax motor1 = new SparkMax(32, MotorType.kBrushless);
  private SparkMax motor2 = new SparkMax(33, MotorType.kBrushless);

  private DigitalInput coralSensor = new DigitalInput(0);
  public boolean seenCoral = false;

  public boolean hasCoral() {
    return seenCoral && !coralSensor.get();
  }

  public Placer() {
    var config = new SparkMaxConfig();
    config.closedLoop.p(1);
    config.closedLoop.i(0);
    config.closedLoop.d(0);
    motor1.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motor2.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void stop() {
    seenCoral = false;
    motor1.set(0);
    motor2.set(0);
  }

  public void intake() {
    if(coralSensor.get()) {
      seenCoral = true;
    }
    motor1.set(-0.25);
    motor2.set(0.25);
  }

  public void spitCoral() {
    motor1.set(-0.6);
    motor2.set(0.6);
  }

  public void outtake() {
    motor1.set(1.0);
    motor2.set(-1.0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Placer/Has Coral", coralSensor.get());
    // var goalHeight = SmartDashboard.getNumber("Elevator/Goal Height", 0.0);
    // var manuallyMoving = SmartDashboard.getBoolean("Elevator/Manual Moving", false);

    // if(Math.abs(goalHeight - Elevator.INTAKE_HEIGHT) > 0.1 || manuallyMoving) {
    //   motor1.set(0);
    //   motor2.set(0);
    // }
  }
}
