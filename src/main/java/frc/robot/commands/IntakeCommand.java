package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Placer;

public class IntakeCommand extends Command {
  private Placer placer;

  public IntakeCommand(Placer placer) {
    this.placer = placer;
    addRequirements(placer);
  }

  @Override
  public void initialize() {
    placer.intake();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return placer.hasCoral();
  }
}
