package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Camera;
import frc.robot.subsystems.Swerve;

public class LockOnTag extends Command {

  PhotonTrackedTarget capturedTarget;
  Transform3d targetTransform;
  Rotation2d targetRotation;
  boolean hasTarget = true;

  public LockOnTag() {
    // addRequirements(Swerve.getInstance());
  }

  @Override
  public void initialize() {
    this.capturedTarget = Camera.getBestResult();
    this.hasTarget = true;
    if(this.capturedTarget == null) {
      hasTarget = false;
      return;
    }

    this.targetTransform = capturedTarget.getBestCameraToTarget();
    this.targetRotation = Rotation2d.fromDegrees(capturedTarget.getYaw());
  }

  @Override
  public void execute() {
    if(this.capturedTarget == null) {
      this.hasTarget = false;
    }
    var currentAngle = Swerve.getInstance().getGyroYaw();

    var angleSign = Math.signum(currentAngle.getDegrees());
    var angleAbs = Rotation2d.fromDegrees(Math.abs(currentAngle.getDegrees()));
    var difference = (Rotation2d.k180deg.minus(angleAbs))
      .times(angleSign);

    Swerve.getInstance().setAngleSetpoint(difference);
  }

  @Override
  public void end(boolean interrupted) {
    Swerve.getInstance().clearAngleSetpoint();
  }

  @Override
  public boolean isFinished() {
    if(!hasTarget || Swerve.getInstance().atTargetAngle()) {
      return true;
    }

    return false;
  }
}