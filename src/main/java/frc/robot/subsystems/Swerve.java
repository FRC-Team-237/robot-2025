package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
  public SwerveDriveOdometry swerveOdometry;
  public SwerveModule[] mSwerveMods;
  public ADIS16470_IMU gyro;

  private static final double MAX_HEIGHT = 4.35;
  private static final double MIN_SPEED = 0.1;

  private PIDController anglePIDController = new PIDController(0.06, 0, 0.1);
  private TalonFX elevatorMotor = new TalonFX(30);
  private boolean angleTargetEnabled = false;

  private double initialAngleOffset = 0;

  private static Swerve instance;

  public static Swerve getInstance() {
    if(Swerve.instance == null) {
      Swerve.instance = new Swerve();
    }

    return Swerve.instance;
  }

  public Swerve() {
    gyro = new ADIS16470_IMU();
    gyro.setGyroAngle(IMUAxis.kYaw, 0);

    SmartDashboard.putData("Swerve/Zero Gyro", new InstantCommand(() -> gyro.setGyroAngle(IMUAxis.kYaw, 0)).ignoringDisable(true));

    mSwerveMods = new SwerveModule[] {
      new SwerveModule(Constants.Swerve.FrontLeft.ordinal, Constants.Swerve.FrontLeft.constants),
      new SwerveModule(Constants.Swerve.FrontRight.ordinal, Constants.Swerve.FrontRight.constants),
      new SwerveModule(Constants.Swerve.BackLeft.ordinal, Constants.Swerve.BackLeft.constants),
      new SwerveModule(Constants.Swerve.BackRight.ordinal, Constants.Swerve.BackRight.constants)
    };

    swerveOdometry = new SwerveDriveOdometry(
      Constants.Swerve.swerveKinematics,
      getGyroYaw(),
      getModulePositions()
    );

    anglePIDController.setTolerance(2.5);
    anglePIDController.enableContinuousInput(-180, 180);
  }

  private double heightSpeedMultiplier() {
    double height = elevatorMotor.getPosition().getValueAsDouble();

    // LINEAR
    // return ((MIN_SPEED - 1) / MAX_HEIGHT) * elevatorMotor.getPosition().getValueAsDouble() + 1;

    // INVERSE
    double power = (MIN_SPEED * MAX_HEIGHT) / (1 - MIN_SPEED);
    return power / (height + power);
  }

  public boolean atTargetAngle() {
    return anglePIDController.atSetpoint(); 
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {

    anglePIDController.calculate(getGyroYaw().getDegrees());
    
    if(angleTargetEnabled) {
      if(atTargetAngle()) {
        clearAngleSetpoint();
      } else {
        rotation = anglePIDController.calculate(getGyroYaw().getDegrees());
      }
    }

    var multiplier = heightSpeedMultiplier();

    translation = translation.times(multiplier);

    SwerveModuleState[] swerveModuleStates =
      Constants.Swerve.swerveKinematics.toSwerveModuleStates(
        fieldRelative

          ? ChassisSpeeds.fromFieldRelativeSpeeds(
              translation.getX(), 
              translation.getY(), 
              rotation, 
              getHeading()
            )
          : new ChassisSpeeds(
              translation.getX(), 
              translation.getY(), 
              rotation
            )
      );
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    for(SwerveModule mod : mSwerveMods){
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  public void setAngleSetpoint(Rotation2d angleSetpoint) {
    anglePIDController.setSetpoint(angleSetpoint.getDegrees());
    angleTargetEnabled = true;
  }

  public void clearAngleSetpoint() {
    this.angleTargetEnabled = false;
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
    
    for(SwerveModule mod : mSwerveMods){
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for(SwerveModule mod : mSwerveMods){
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for(SwerveModule mod : mSwerveMods){
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void setPose(Pose2d pose) {
    swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
  }

  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  public void setHeading(Rotation2d heading){
    swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
  }

  public void zeroHeading() {
    gyro.setGyroAngle(IMUAxis.kYaw, 180);
    // swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
  }

  public Rotation2d getGyroYaw() {
    return Rotation2d.fromDegrees(gyro.getAngle(IMUAxis.kYaw)).plus(Rotation2d.fromDegrees(initialAngleOffset));
  }

  public void resetModulesToAbsolute() {
    for(SwerveModule mod : mSwerveMods){
      mod.resetToAbsolute();
    }
  }

  @Override
  public void periodic() {
    swerveOdometry.update(getGyroYaw(), getModulePositions());

    SmartDashboard.putNumber("Swerve/Elevator Height", elevatorMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Swerve/Elevator heightSpeedMultiplier", heightSpeedMultiplier());

    SmartDashboard.putNumber("Swerve/Heading", getGyroYaw().getDegrees());

    for(SwerveModule mod : mSwerveMods){
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);  
    }
  }
}