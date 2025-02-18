package frc.robot.sysId;

import java.util.Optional;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorProfiler extends SubsystemBase {

  private final TalonFX motor1 = new TalonFX(30);
  private final TalonFX motor2 = new TalonFX(31);

  private TrapezoidProfile trapezoidProfile;

  private double maxVelocity = 0.0;
  private double maxAcceleration = 0.0;

  private double kG = 0.0;
  private double kV = 0.0;
  private double kA = 0.0;

  private double kP = 0.0;
  private double kI = 0.0;
  private double kD = 0.0;

  private Optional<Double> setpoint = Optional.empty();

  public ElevatorProfiler() {
    SmartDashboard.putNumber("Elevator/Profiler/maxVelocity", 0.0);
    SmartDashboard.putNumber("Elevator/Profiler/maxAcceleration", 0.0);

    SmartDashboard.putNumber("Elevator/Profiler/kG", 0.0);
    SmartDashboard.putNumber("Elevator/Profiler/kV", 0.0);
    SmartDashboard.putNumber("Elevator/Profiler/kA", 0.0);

    SmartDashboard.putNumber("Elevator/Profiler/kP", 0.0);
    SmartDashboard.putNumber("Elevator/Profiler/kI", 0.0);
    SmartDashboard.putNumber("Elevator/Profiler/kD", 0.0);

    setValues();

    SmartDashboard.putData(
      "Elevator/Update Profile Values",
      new InstantCommand(this::setValues)
    );
  }

  // public void getValuesFromDashboard() {
  //   maxVelocity = SmartDashboard.getNumber("Elevator/Profiler/maxVelocity", 0.0);
  //   maxAcceleration = SmartDashboard.getNumber("Elevator/Profiler/maxAcceleration", 0.0);

  //   kG = SmartDashboard.getNumber("Elevator/Profiler/kG", 0.0);
  //   kV = SmartDashboard.getNumber("Elevator/Profiler/kV", 0.0);
  //   kA = SmartDashboard.getNumber("Elevator/Profiler/kA", 0.0);

  //   kP = SmartDashboard.getNumber("Elevator/Profiler/kP", 0.0);
  //   kI = SmartDashboard.getNumber("Elevator/Profiler/kI", 0.0);
  //   kD = SmartDashboard.getNumber("Elevator/Profiler/kD", 0.0);

  //   var configs = new TalonFXConfiguration();
  //   var slot0Configs = configs.Slot0;
  //   slot0Configs.kG = kG;
  //   slot0Configs.kV = kV;
  //   slot0Configs.kA = kA;
  //   slot0Configs.kP = kP;
  //   slot0Configs.kI = kI;
  //   slot0Configs.kD = kD;
  //   motor1.getConfigurator().apply(configs);
  //   motor2.getConfigurator().apply(configs);

  //   motor1.setNeutralMode(NeutralModeValue.Brake);
  //   motor2.setNeutralMode(NeutralModeValue.Brake);

  //   trapezoidProfile = new TrapezoidProfile(
  //     new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration)
  //   );

  //   System.out.println(
  //     "maxVelocity: " + maxVelocity +
  //     ", maxAcceleration: " + maxAcceleration +
  //     ", kG: " + kG +
  //     ", kV: " + kV +
  //     ", kA: " + kA +
  //     ", kP: " + kP +
  //     ", kI: " + kI +
  //     ", kD: " + kD
  //   );
  // }

  public void setValues() {
    maxVelocity = 1.0;
    maxAcceleration = 0.5;

    kG = 1.0;
    kV = 0.0;
    kA = 0.0;

    kP = 0.0;
    kI = 0.0;
    kD = 0.0;

    var configs = new TalonFXConfiguration();
    var slot0Configs = configs.Slot0;
    slot0Configs.kG = kG;
    slot0Configs.kV = kV;
    slot0Configs.kA = kA;
    slot0Configs.kP = kP;
    slot0Configs.kI = kI;
    slot0Configs.kD = kD;
    motor1.getConfigurator().apply(configs);
    motor2.getConfigurator().apply(configs);

    motor1.setNeutralMode(NeutralModeValue.Brake);
    motor2.setNeutralMode(NeutralModeValue.Brake);

    trapezoidProfile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration)
    );

    System.out.println(
      "maxVelocity: " + maxVelocity +
      ", maxAcceleration: " + maxAcceleration +
      ", kG: " + kG +
      ", kV: " + kV +
      ", kA: " + kA +
      ", kP: " + kP +
      ", kI: " + kI +
      ", kD: " + kD
    );
  }

  public void goTo(double height) {
    this.setpoint = Optional.of(height);
  }

  public void stop() {
    this.setpoint = Optional.empty();
  }

  @Override
  public void periodic() {
    if(this.setpoint.isPresent()) {
      var goal = new TrapezoidProfile.State(this.setpoint.get(), 0);
      var setpoint = new TrapezoidProfile.State();
      var request = new PositionVoltage(0).withSlot(0);

      setpoint = trapezoidProfile.calculate(1, setpoint, goal);

      request.Position = setpoint.position;
      request.Velocity = setpoint.velocity;

      this.motor1.setControl(request);
      this.motor2.setControl(request);
    }
  }
}
