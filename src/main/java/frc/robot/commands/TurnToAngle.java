package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class TurnToAngle extends Command {

  private Timer timer;
  private double seconds;
  private Rotation2d angle;

  public TurnToAngle(Rotation2d angle) {
    this(angle, Seconds.of(1));
  }

  public TurnToAngle(Rotation2d angle, Time timeout) {
    this.angle = angle;
    this.timer = new Timer();
    this.seconds = timeout.in(Seconds);
  }

  @Override
  public void execute() {
    Swerve.getInstance().setAngleSetpoint(angle);
  }

  @Override
  public void initialize() {
    this.timer.reset();
    this.timer.start();
  }

  @Override
  public boolean isFinished() {
    var timerDone = this.timer.hasElapsed(seconds);
    var atTargetAngle = Swerve.getInstance().atTargetAngle();
    return timerDone || atTargetAngle;
  }

  @Override
  public void end(boolean interrupted) {
    Swerve.getInstance().clearAngleSetpoint();
  }
}