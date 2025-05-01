package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class NewDriveToPose extends Command {
  private static final double driveP = 0.8;
  private static final double driveD = 0;
  private static final double angleP = 4.0;
  private static final double angleD = 0;

  private static final double driveMaxVelocity = 3.8;
  private static final double driveMaxVelocitySlow = 1.25;
  private static final double driveMaxAcceleration = 3.0;

  private static final double angleMaxVelocity = Units.degreesToRadians(360.0);
  private static final double angleMaxVelocitySlow = Units.degreesToRadians(180.0);
  private static final double angleMaxAcceleration = 8.0;

  private static final double driveTolerance = 0.01;
  private static final double angleTolerance = Units.degreesToRadians(1.0);

  private static final double ffMinRadius = 0.05;
  private static final double ffMaxRadius = 0.1;

  private final Swerve swerve = Swerve.getInstance();

  private final Supplier<Pose2d> target;

  private final ProfiledPIDController driveController =
    new ProfiledPIDController(
      driveP, 0.0, driveD,
      new TrapezoidProfile.Constraints(driveMaxVelocity, driveMaxAcceleration)
    );
  private final ProfiledPIDController angleController =
    new ProfiledPIDController(
      angleP, 0.0, angleD,
      new TrapezoidProfile.Constraints(angleMaxVelocity, angleMaxAcceleration)
    );
  
  private Translation2d lastSetpointTranslation = Translation2d.kZero;
  private Rotation2d lastSetpointRotation = Rotation2d.fromDegrees(0.0);
  private double lastTime = 0.0;
  private double driveErrorAbs = 0.0;
  private double angleErrorAbs = 0.0;
  private boolean running = false;
  public boolean isRunning() {
    return running;
  }
  private Supplier<Pose2d> robotPose = swerve::getPose;

  private Supplier<Translation2d> linearFF = () -> Translation2d.kZero;
  private DoubleSupplier angleFF = () -> 0.0;

  public NewDriveToPose(Supplier<Pose2d> target) {
    this.target = target;

    driveController.setTolerance(driveTolerance);
    angleController.setTolerance(angleTolerance);

    driveController.enableContinuousInput(-Math.PI, Math.PI);
    // addRequirements(swerve);
  }

  @Override
  public void initialize() {
    Pose2d currentPose = robotPose.get();
    ChassisSpeeds fieldVelocity = swerve.getChassisSpeeds();
    Translation2d linearFieldVelocity =
      new Translation2d(
        fieldVelocity.vxMetersPerSecond,
        fieldVelocity.vyMetersPerSecond
      );
    
    driveController.reset(
      currentPose.getTranslation().getDistance(target.get().getTranslation()),
      Math.min(
        0.0,
        -linearFieldVelocity
          .rotateBy(
            target
              .get()
              .getTranslation()
              .minus(currentPose.getTranslation())
              .getAngle()
              .unaryMinus()
          ).getX()
      )
    );
    angleController.reset(
      currentPose.getRotation().getRadians(),
      fieldVelocity.omegaRadiansPerSecond
    );
    lastSetpointTranslation = currentPose.getTranslation();
    lastSetpointRotation = target.get().getRotation();
    lastTime = Timer.getTimestamp();
  }

  @Override
  public void execute() {
    running = true;

    var currentPose = robotPose.get();
    var targetPose = target.get();

    var currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
    var ffScalar =
      MathUtil.clamp(
        (currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius),
        0.0,
        1.0
      );
    
    driveErrorAbs = currentDistance;
    driveController.reset(
      lastSetpointTranslation.getDistance(targetPose.getTranslation()),
      driveController.getSetpoint().velocity
    );
    double driveVelocityScalar =
      driveController.calculate(driveErrorAbs, 0.0)
        + driveController.getSetpoint().velocity * ffScalar;
    
    if(currentDistance < driveController.getPositionTolerance()) {
      driveVelocityScalar = 0.0;
    }

    lastSetpointTranslation = new Pose2d(
      targetPose.getTranslation(),
      new Rotation2d(
        Math.atan2(
          currentPose.getTranslation().getY() - targetPose.getTranslation().getY(),
          currentPose.getTranslation().getX() - targetPose.getTranslation().getX()
        )
      )
    )
    .transformBy(new Transform2d(
      driveController.getSetpoint().position,
      0.0,
      Rotation2d.kZero
    ))
    .getTranslation();

    double angleVelocity = angleController.calculate(
      currentPose.getRotation().getRadians(),
      new TrapezoidProfile.State(
        targetPose.getRotation().getRadians(),
        (targetPose.getRotation().minus(lastSetpointRotation)).getRadians()
          / (Timer.getTimestamp() - lastTime)
      )
    ) + angleController.getSetpoint().velocity * ffScalar;

    angleErrorAbs = Math.abs(
      currentPose.getRotation().minus(
        targetPose.getRotation()
      ).getRadians()
    );

    if(angleErrorAbs < angleController.getPositionTolerance()) {
      angleVelocity = 0.0;
    }

    lastSetpointRotation = targetPose.getRotation();
    Translation2d driveVelocity = new Pose2d(
      Translation2d.kZero,
      new Rotation2d(
        Math.atan2(
          currentPose.getTranslation().getY() - targetPose.getTranslation().getY(),
          currentPose.getTranslation().getX() - targetPose.getTranslation().getX()
        )
      )
    )
    .transformBy(new Transform2d(
      driveVelocityScalar,
      0.0,
      Rotation2d.kZero
    ))
    .getTranslation();

    lastTime = Timer.getTimestamp();

    final double linearS = linearFF.get().getNorm() * 3.0;
    final double angleS = Math.abs(angleFF.getAsDouble()) * 3.0;

    driveVelocity =
      driveVelocity.interpolate(
        linearFF.get().times(Constants.Swerve.maxSpeed),
        linearS
      );
    
    angleVelocity = MathUtil.interpolate(
      angleVelocity,
      angleFF.getAsDouble() * Constants.Swerve.maxAngularVelocity,
      angleS
    );

    swerve.drive(
      driveVelocity,
      angleVelocity,
      false,
      true
    );
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(Translation2d.kZero, 0, false, true);
    running = false;
  }

  public boolean atGoal() {
    return running && driveController.atGoal() && angleController.atGoal();
  }

  public boolean withinTolerance(double driveTolerance, Rotation2d angleTolerance) {
    return running
      && Math.abs(driveErrorAbs) < driveTolerance
      && angleErrorAbs < angleTolerance.getRadians();
  }
}
