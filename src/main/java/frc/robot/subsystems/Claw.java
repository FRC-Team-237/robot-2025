package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {

  private SparkMax clawIntake = new SparkMax(35, MotorType.kBrushless);
  private SparkMax clawPosition = new SparkMax(34, MotorType.kBrushless);

  public Claw() {
    var intakeConfig = new SparkMaxConfig();
    intakeConfig.closedLoop.p(1);
    intakeConfig.closedLoop.i(0);
    intakeConfig.closedLoop.d(0);
    
    clawIntake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void outtake() {
    clawIntake.set(0.5);
  }

  public void intake() {
    clawIntake.set(-0.5);
  }

  public void stopIntake() {
    clawIntake.set(0);
  }

  public void stopClawPosition() {
    clawPosition.set(0);
  }

  public void raiseClaw() {
    clawPosition.set(0.5);
  }

  public void lowerClaw() {
    clawPosition.set(-0.5);
  }

  @Override
  public void periodic() {}
}
