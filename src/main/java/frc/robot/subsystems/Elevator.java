package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Optional;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

  private static Elevator instance;

  public static Elevator getInstance() {
    if (instance == null) {
      instance = new Elevator();
    }
    return instance;
  }

  // public static double MAX_HEIGHT = 4.2414;
  public static double MAX_HEIGHT = 4.335;
  public static double MID_HEIGHT = 2.76;
  public static double LOW_HEIGHT = 1.95;
  public static double LOW_ALGAE_HEIGHT = 1.28;
  public static double HIGH_ALGAE_HEIGHT = 2.35;
  public static double INTAKE_HEIGHT = 0.72;

  private final double LOWEST_POSITION = 0.2;
  private final double HIGHEST_POSITION = 4.35;
  private final int POSITION_SLOT = 0;
  private final int VELOCITY_SLOT = 1;
  private final int NEUTRAL_SLOT = 2;
  private final TalonFX motor1 = new TalonFX(30);
  private final TalonFX motor2 = new TalonFX(31);

  private final double minVoltageToMove = 0.25;
  private final double gearRatio = 7.75/1.0;
  private final double dt = 0.02;

  private Timer easeTimer = new Timer();

  private final double dynamicG = 0.5;
  private final double dynamicV = 0.1;
  private final double dynamicA = 0.0;

  private final double dynamicP = 1.0;
  private final double dynamicI = 0.0;
  private final double dynamicD = 0.0;

  private TrapezoidProfile profile;
  private TrapezoidProfile.State currentState = new TrapezoidProfile.State();
  public Optional<TrapezoidProfile.State> goal = Optional.empty();

  private double desiredVelocity = 0.0;

  private final double maxVelocity = 3.0;
  private final double maxAcceleration = 2.5;

  private final double staticG = 0.7;
  private final double staticV = 3.5;
  private final double staticA = 0.5;

  private final double staticP = 14.0;
  private final double staticI = 1.4;
  private final double staticD = 0.14;

  private final DigitalInput zeroSensor = new DigitalInput(1);
  
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(minVoltageToMove, 0.25);

  private boolean isManualMoving = false;
  private boolean isMovingDown = false;

  public static double getHeight() {
    return instance.motor1.getPosition().getValueAsDouble();
  }
  
  private TalonFXConfiguration configs = new TalonFXConfiguration()
    // position control
    .withSlot0(
      new Slot0Configs()
        .withKG(staticG)
        .withKV(staticV)
        .withKA(staticA)
        .withKP(staticP)
        .withKI(staticI)
        .withKD(staticD)
    )
    // velocity control
    .withSlot1(
      new Slot1Configs()
        .withKG(dynamicG)
        .withKV(dynamicV)
        .withKA(dynamicA)
        .withKP(dynamicP)
        .withKI(dynamicI)
        .withKD(dynamicD)
    )
    // neutral control
    .withSlot2(
      new Slot2Configs()
        .withKG(0.0)
        .withKV(0.0)
        .withKA(0.0)
        .withKP(0.0)
        .withKI(0.0)
        .withKD(0.0)
    )
    .withFeedback(
      new FeedbackConfigs()
        .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
        .withSensorToMechanismRatio(gearRatio)
    );

  private void setMotorProfile() {
    profile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(
        maxVelocity,
        maxAcceleration
      )
    );
    motor1.getConfigurator().apply(configs);
    motor2.getConfigurator().apply(configs);
    motor1.setNeutralMode(NeutralModeValue.Brake);
    motor2.setNeutralMode(NeutralModeValue.Brake);
  }

  public Elevator() {
    setMotorProfile();
    drop();
    motor1.setPosition(0);
    motor2.setPosition(0);
  }

  public void setGoal(double height) {
    height = MathUtil.clamp(height, 0, HIGHEST_POSITION);
    isManualMoving = false;
    double position = motor1.getPosition().getValueAsDouble();
    double velocity = motor1.getVelocity().getValueAsDouble();

    isMovingDown = position > height;

    currentState = new TrapezoidProfile.State(position, velocity);

    goal = Optional.of(new TrapezoidProfile.State(height, 0));
  }

  public void move(double rps) {
    var isSwitchingDirections = !isManualMoving || Math.signum(motor1.getVelocity().getValueAsDouble()) != Math.signum(rps);
    isManualMoving = true;
    isMovingDown = rps < 0;
    goal = Optional.empty();
    desiredVelocity = rps;
    if(isSwitchingDirections) {
      easeTimer.reset();
      easeTimer.start();
    }
  }

  public void stop() {
    var currentPosition = motor1.getPosition().getValueAsDouble();

    setGoal(currentPosition);
  }

  public void drop() {
    isManualMoving = false;
    isMovingDown = false;
    goal = Optional.empty();
    motor1.setControl(
      new VelocityVoltage(0)
        .withSlot(NEUTRAL_SLOT)
    );
    motor2.setControl(
      new VelocityVoltage(0)
        .withSlot(NEUTRAL_SLOT)
    );
  }

  @Override
  public void periodic() {
    double sensorPosition = motor1.getPosition().getValueAsDouble();
    double sensorVelocity = motor1.getVelocity().getValueAsDouble();
    SmartDashboard.putNumber("Elevator/Height", sensorPosition);
    SmartDashboard.putNumber("Elevator/Velocity", sensorVelocity);
    SmartDashboard.putBoolean("Elevator/Zero", !zeroSensor.get());
    SmartDashboard.putNumber("Elevator/Ease Timer", MathUtil.clamp(easeTimer.get() * 2.0, 0, 1));
    SmartDashboard.putBoolean("Elevator/Manual Moving", isManualMoving);
    SmartDashboard.putBoolean("Elevator/Moving Down", isMovingDown);
    // if(goal.isPresent()) {
    //   SmartDashboard.putNumber("Elevator/Goal Height", goal.get().position);
    // } else {
    //   SmartDashboard.putNumber("Elevator/Goal Height", 0.0);
    // }
    if(!zeroSensor.get()) {
      motor1.setPosition(0);
      motor2.setPosition(0);
    }

    // stop motors if it's moving down and almost near the bottom
    if(isMovingDown && !zeroSensor.get() /*&& sensorPosition < LOWEST_POSITION*/) {
      motor1.setControl(new VelocityVoltage(0).withSlot(NEUTRAL_SLOT));
      motor2.setControl(new VelocityVoltage(0).withSlot(NEUTRAL_SLOT));
      return;
    }
  
    // stop motors if it's moving up and almost near the top (soft stop)
    if(!isMovingDown && sensorPosition > HIGHEST_POSITION) {
      motor1.setControl(new VelocityVoltage(0).withSlot(NEUTRAL_SLOT));
      motor2.setControl(new VelocityVoltage(0).withSlot(NEUTRAL_SLOT));
      return;
    }
    
    if(goal.isPresent()) {
      currentState = profile.calculate(dt, currentState, goal.get());
      double ffVoltage = feedforward.calculate(currentState.velocity);
      SmartDashboard.putNumber("Elevator/Hold FF Voltage", ffVoltage);

      if(Math.abs(sensorPosition - goal.get().position) > 0.025) {
        var request = new PositionVoltage(currentState.position)
          .withFeedForward(ffVoltage)
          .withSlot(POSITION_SLOT);
        
        motor1.setControl(request);
        motor2.setControl(request);
      } else {
        var request = new PositionVoltage(sensorPosition)
          .withFeedForward(ffVoltage)
          .withSlot(POSITION_SLOT);
        
        motor1.setControl(request);
        motor2.setControl(request);
      }

    } else {
      if(!isManualMoving) {
        motor1.setControl(new VelocityVoltage(0).withSlot(NEUTRAL_SLOT));
        motor2.setControl(new VelocityVoltage(0).withSlot(NEUTRAL_SLOT));
      } else {
        double easeValue = MathUtil.clamp(easeTimer.get() * 2.0, 0, 1);
        double newVelocity = easeValue * desiredVelocity;

        motor1.setControl(new VelocityVoltage(RotationsPerSecond.of(newVelocity)).withSlot(VELOCITY_SLOT));
        motor2.setControl(new VelocityVoltage(RotationsPerSecond.of(newVelocity)).withSlot(VELOCITY_SLOT));
      }
    }
  }
  
  public boolean isGoingToIntake() {
    if(goal.isEmpty()) return false;

    return !isManualMoving && Math.abs(goal.get().position - Elevator.INTAKE_HEIGHT) < 0.1;
  }
}