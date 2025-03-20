package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.Optional;

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

  private static final double MIN_SPEED = 0.1;

  private PIDController anglePIDController = new PIDController(0.06, 0, 0.1);
  private TalonFX elevatorMotor = new TalonFX(30);
  private boolean angleTargetEnabled = false;

  private Optional<Translation2d> storedPosition = Optional.empty();

  public void setStoredPosition() {
    storedPosition = Optional.of(swerveOdometry.getPoseMeters().getTranslation());
  }

  public Optional<Translation2d> getStoredPosition() {
    return storedPosition;
  }

  public Optional<Double> getStoredDistance() {
    if(storedPosition.isEmpty()) return Optional.empty();

    return Optional.of(
      storedPosition.get().getDistance(
        swerveOdometry.getPoseMeters().getTranslation()
      )
    );
  }

  private double initialAngleOffset = 0;

  private static Swerve instance;

  public static Swerve getInstance() {
    if(Swerve.instance == null) {
      Swerve.instance = new Swerve();
    }

    return Swerve.instance;
  }

  public Swerve() {
    SmartDashboard.putData("Swerve/Store Distance", new InstantCommand(this::setStoredPosition));
    SmartDashboard.putData("Swerve/Get Stored Distance", new InstantCommand(() -> {
      if(getStoredPosition().isEmpty()) return;
      System.out.println(getStoredDistance());
    }));

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
    // double height = elevatorMotor.getPosition().getValueAsDouble();

    // // if(height <= Elevator.INTAKE_HEIGHT) return 1;

    // // // LINEAR
    // // // return ((MIN_SPEED - 1) / MAX_HEIGHT) * elevatorMotor.getPosition().getValueAsDouble() + 1;

    // // // INVERSE
    // // // double power = (MIN_SPEED * Elevator.MAX_HEIGHT) / (1 - MIN_SPEED);
    // // // return power / (height + power);

    // // double hScale = (Elevator.MAX_HEIGHT - Elevator.INTAKE_HEIGHT) / Elevator.MAX_HEIGHT;
    
    // // double value = -MathUtils.downwardCurve(
    // //   (height - Elevator.INTAKE_HEIGHT) / hScale,
    // //   MIN_SPEED,
    // //   Elevator.INTAKE_HEIGHT
    // // );
    
    // // SmartDashboard.putNumber("Swerve/Height Speed Multiplier", value);
    
    // // return value;

    // return MathUtils.downwardCurve(
    //   height,
    //   MIN_SPEED,
    //   Elevator.INTAKE_HEIGHT
    // );

    double height = elevatorMotor.getPosition().getValueAsDouble() - 0.15;

    if(height <= Elevator.INTAKE_HEIGHT) return 1;
    if(height <= Elevator.LOW_ALGAE_HEIGHT) return 0.8;
    if(height <= Elevator.LOW_HEIGHT) return 0.4;
    if(height <= Elevator.HIGH_ALGAE_HEIGHT) return 0.25;
    if(height <= Elevator.MID_HEIGHT) return 0.175;

    return 0.135;
  }

  // private double heightSpeedMultiplier() {
  //   double height = elevatorMotor.getPosition().getValueAsDouble();

  //   double power = (MIN_SPEED * Elevator.MAX_HEIGHT) / (1 - MIN_SPEED);
    
  //   double scale = Elevator.MAX_HEIGHT / (Elevator.MAX_HEIGHT - Elevator.INTAKE_HEIGHT);

  //   return -power / (scale * (height - Elevator.INTAKE_HEIGHT) + power);
  // }

  public boolean atTargetAngle() {
    return anglePIDController.atSetpoint(); 
  }

  public double getTargetAngle() {
    return anglePIDController.getSetpoint();
  }

  public void driveUnsafe(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {

    // if(Math.abs(rotation) > 0.1) {
    //   clearAngleSetpoint();
    // }

    SmartDashboard.putNumber(
      "Swerve/Speed/X",
      getVelocity().getX()
    );

    SmartDashboard.putNumber(
      "Swerve/Speed/Y",
      getVelocity().getY()
    );

    SmartDashboard.putNumber(
      "Swerve/Speed/Magnitude",
      getVelocity().getNorm()
    );

    if(storedPosition.isPresent()) {
      var p = storedPosition.get();
      SmartDashboard.putNumber("Swerve/Position Delta/X", p.getX() - swerveOdometry.getPoseMeters().getTranslation().getX());
      SmartDashboard.putNumber("Swerve/Position Delta/Y", p.getY() - swerveOdometry.getPoseMeters().getTranslation().getY());
    } else {
      SmartDashboard.putNumber("Swerve/Position Delta/X", 0);
      SmartDashboard.putNumber("Swerve/Position Delta/Y", 0);
    }

    anglePIDController.calculate(getGyroYaw().getDegrees());
    
    if(angleTargetEnabled) {
      if(atTargetAngle()) {
        // clearAngleSetpoint();
      } else {
        rotation = anglePIDController.calculate(getGyroYaw().getDegrees());
      }
    }

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

  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    var multiplier = heightSpeedMultiplier();
    translation = translation.times(multiplier);
    driveUnsafe(translation, rotation, fieldRelative, isOpenLoop);
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

  public Rotation2d getRotationalVelocity() {
    return Rotation2d.fromDegrees(gyro.getRate(IMUAxis.kYaw));
  }

  public void resetModulesToAbsolute() {
    for(SwerveModule mod : mSwerveMods){
      mod.resetToAbsolute();
    }
  }

  public Translation2d getVelocity() {
    var vec = new Translation2d();

    for(SwerveModule mod : mSwerveMods) {
      vec = vec.plus(
        new Translation2d(
          mod.getState().speedMetersPerSecond,
          mod.getState().angle
        )
      );
    }

    return vec.div(4.0);
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