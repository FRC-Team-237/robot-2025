package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.MathUtils;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {
  private Swerve s_Swerve;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
  private BooleanSupplier robotCentricSup;

  public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;
  }

  @Override
  public void execute() {
    /* Get Values, Deadband*/
    double translationVal = MathUtils.smoothInput(MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband), 2.0);
    double strafeVal = MathUtils.smoothInput(MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband), 2.0);
    double rotationVal = MathUtils.smoothInput(MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband), 2.0);

    if(Math.abs(rotationVal) > 0.1) {
      s_Swerve.clearAngleSetpoint();
    }

    /* Drive */
    s_Swerve.drive(
      new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
      rotationVal * Constants.Swerve.maxAngularVelocity, 
      !robotCentricSup.getAsBoolean(), 
      true
    );
  }
}