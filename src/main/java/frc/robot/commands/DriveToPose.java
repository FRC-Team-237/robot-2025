package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class DriveToPose extends Command {
  private static final TrapezoidProfile.Constraints X_CONSTRAINTS =
    new TrapezoidProfile.Constraints(1.0, 3);
  
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS =
    new TrapezoidProfile.Constraints(1.0, 3);

  private static final TrapezoidProfile.Constraints R_CONSTRAINTS =
    new TrapezoidProfile.Constraints(8, 8);
  
  private final Swerve swerve;

  private final ProfiledPIDController xController =
    new ProfiledPIDController(0.5, 0, 0, X_CONSTRAINTS);
  
  private final ProfiledPIDController yController =
    new ProfiledPIDController(0.5, 0, 0, Y_CONSTRAINTS);
  
  private final ProfiledPIDController rController =
    new ProfiledPIDController(3.0, 0, 0, R_CONSTRAINTS);
  
  private Pose2d startPose;
  private final Transform2d relativeTransform;
  private final boolean fieldOriented;

  private Pose2d targetPose;

  private DoubleLogEntry xLog;
  private DoubleLogEntry yLog;
  private DoubleLogEntry rLog;
  private DoubleLogEntry xTargetLog;
  private DoubleLogEntry yTargetLog;
  private DoubleLogEntry rTargetLog;
  private DoubleLogEntry xErrorLog;
  private DoubleLogEntry yErrorLog;
  private DoubleLogEntry rErrorLog;
  
  public DriveToPose(
    Transform2d relativeTransform,
    boolean fieldOriented
  ) {
    swerve = Swerve.getInstance();
    
    this.relativeTransform = relativeTransform;
    this.fieldOriented = fieldOriented;

    xController.setTolerance(0.2);
    yController.setTolerance(0.2);
    rController.setTolerance(Units.degreesToRadians(3.0));
    rController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    startPose = swerve.getPose();

    xController.reset(startPose.getX());
    yController.reset(startPose.getY());
    rController.reset(startPose.getRotation().getRadians());

    if(fieldOriented) {
      targetPose = new Pose2d(
        startPose.getTranslation().plus(relativeTransform.getTranslation()),
        relativeTransform.getRotation()
      );
    } else {
      targetPose = startPose.transformBy(relativeTransform);
    }

    xController.setGoal(targetPose.getX());
    yController.setGoal(targetPose.getY());
    rController.setGoal(targetPose.getRotation().getRadians());

    DataLogManager.start();
    var log = DataLogManager.getLog();
    xLog = new DoubleLogEntry(log, "A/Log/X");
    yLog = new DoubleLogEntry(log, "A/Log/Y");
    rLog = new DoubleLogEntry(log, "A/Log/R");

    xTargetLog = new DoubleLogEntry(log, "A/Log/X Target");
    yTargetLog = new DoubleLogEntry(log, "A/Log/Y Target");
    rTargetLog = new DoubleLogEntry(log, "A/Log/R Target");

    xErrorLog = new DoubleLogEntry(log, "A/Log/X Error");
    yErrorLog = new DoubleLogEntry(log, "A/Log/Y Error");
    rErrorLog = new DoubleLogEntry(log, "A/Log/R Error");
  }

  @Override
  public void execute() {
    var currentPose = swerve.getPose();

    var x = xController.calculate(currentPose.getX());
    var y = yController.calculate(currentPose.getY());
    var r = rController.calculate(currentPose.getRotation().getRadians());

    SmartDashboard.putNumber("A/Log/R Value", currentPose.getRotation().getRadians());
    SmartDashboard.putNumber("A/Log/R Target", rController.getGoal().position);
    SmartDashboard.putNumber("A/Log/R Output", r);

    SmartDashboard.putNumber("A/Log/X Value", currentPose.getX());
    SmartDashboard.putNumber("A/Log/X Target", xController.getGoal().position);
    SmartDashboard.putNumber("A/Log/X Output", x);

    SmartDashboard.putNumber("A/Log/Y Value", currentPose.getY());
    SmartDashboard.putNumber("A/Log/Y Target", yController.getGoal().position);
    SmartDashboard.putNumber("A/Log/Y Output", y);

    xLog.append(x);
    yLog.append(y);
    rLog.append(r);

    xTargetLog.append(xController.getGoal().position);
    yTargetLog.append(yController.getGoal().position);
    rTargetLog.append(rController.getGoal().position);

    xErrorLog.append(xController.getPositionError());
    yErrorLog.append(yController.getPositionError());
    rErrorLog.append(rController.getPositionError());

    swerve.drive(
      new Translation2d(x, y),
      r,
      fieldOriented,
      true
    );
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(Translation2d.kZero, 0, false, true);
    System.out.println("At setpoint");
  }

  @Override
  public boolean isFinished() {
    var isXDone = xController.atGoal();
    var isYDone = yController.atGoal();
    var isRDone = rController.atGoal();

    return isXDone && isYDone && isRDone;
  }
}
