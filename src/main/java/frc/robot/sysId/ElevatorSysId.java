package frc.robot.sysId;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

public class ElevatorSysId extends SubsystemBase {
  private final TalonFX motor1 = new TalonFX(30);
  private final TalonFX motor2 = new TalonFX(31);

  private SysIdRoutine routine;

  public ElevatorSysId() {
    SysIdRoutine.Config config = new SysIdRoutine.Config(
      Volts.of(0.075).per(Second),
      Volts.of(0.95),
      Seconds.of(30)
    );

    Mechanism mechanism = new Mechanism(
      (voltage) -> {
        motor1.setControl(new VoltageOut(voltage));
        motor2.setControl(new VoltageOut(voltage));
      },
      (log) -> {
        log.motor("Motor 1")
          .voltage(motor1.getMotorVoltage().getValue())
          .current(motor1.getStatorCurrent().getValue())
          .angularPosition(motor1.getPosition().getValue())
          .angularVelocity(motor1.getVelocity().getValue());

        log.motor("Motor 2")
          .voltage(motor2.getMotorVoltage().getValue())
          .current(motor2.getStatorCurrent().getValue())
          .angularPosition(motor2.getPosition().getValue())
          .angularVelocity(motor2.getVelocity().getValue());
      },
      this,
      "Elevator"
    );

    routine = new SysIdRoutine(config, mechanism);

    SmartDashboard.putData(
      "Elevator/SysId Profile/Quasistatic Forward",
      routine.quasistatic(Direction.kForward)
    );

    SmartDashboard.putData(
      "Elevator/SysId Profile/Dynamic Forward",
      routine.dynamic(Direction.kForward)
    );
  }
}