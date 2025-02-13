package frc.robot.subsystems;

import java.util.Optional;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  public enum ElevatorDirection { UP, DOWN, STOP }

  private TalonFX motor1 = new TalonFX(30);
  private TalonFX motor2 = new TalonFX(31);

  double desiredVelocity = 0.0;

  public boolean isMovingDown = false;
  public boolean isManualMoving = false;

  private Timer easeTimer = new Timer();
  private PIDController heightPID = new PIDController(16, 0.8, 0.08);
  private Optional<Double> heightSetpoint = Optional.empty();

  private final double lowestValue = 1.0;
  private final double highestValue = 34.0;
  private final double weightOffest = 9.5;

  private DigitalInput zeroSensor = new DigitalInput(1);


  public Elevator() {
    var configs = new TalonFXConfiguration();
    var slot0Configs = configs.Slot0;
    slot0Configs.kS = 0.01;
    slot0Configs.kV = 0.12;
    slot0Configs.kP = 0.002;

    motor1.getConfigurator().apply(configs);
    motor2.getConfigurator().apply(configs);

    motor1.setNeutralMode(NeutralModeValue.Brake);
    motor2.setNeutralMode(NeutralModeValue.Brake);

    resetPosition();
  }

  public void resetPosition() {
    motor1.setPosition(0);
    motor2.setPosition(0);
  }

  @Override
  public void periodic() {
    if (zeroSensor.get()) {
      resetPosition();
    }
    double currentPosition = motor1.getPosition().getValueAsDouble();
    double easeValue = MathUtil.clamp(easeTimer.get() * 2, 0, 1);

    if(isManualMoving) {
      double newVelocity = easeValue * desiredVelocity;
  
      setVelocity(newVelocity);
    } else {

      if(heightSetpoint.isPresent()) {
        double delta = heightPID.calculate(currentPosition);

        double clampedDelta = Math.min(Math.max(delta, -5), 25);

        double distance = Math.abs(currentPosition - heightSetpoint.get());
        if(distance < 0.5) {
          clampedDelta += weightOffest;
        }

        double newVelocity = easeValue * clampedDelta;

        setVelocity(newVelocity);
      }
    }

    if(currentPosition >= highestValue) {
      setVelocity(0);
      setHeight(highestValue);
    }

    if(isMovingDown && currentPosition <= lowestValue) {
      letGo();
    }

    SmartDashboard.putNumber("Elevator/Height", currentPosition);
  }

  public void letGo() {
    heightSetpoint = Optional.empty();
    heightPID.setSetpoint(0);
    isManualMoving = false;
    isMovingDown = false;
    setVelocity(0);
  }

  public void setHeight(double height) {
    if(height < lowestValue) height = lowestValue;
    if(height > highestValue) height = highestValue;

    easeTimer.reset();
    easeTimer.start();

    double currentPosition = motor1.getPosition().getValueAsDouble();
    isMovingDown = currentPosition > height;
    isManualMoving = false;
    heightSetpoint = Optional.of(height);
    heightPID.setSetpoint(height);
  }

  public void setVelocity(double rps) {
    motor1.setControl(
      new VelocityVoltage(rps)
    );
    motor2.setControl(
      new VelocityVoltage(rps)
    );
  }

  ElevatorDirection previousDirection = ElevatorDirection.STOP;

  public void moveElevator(ElevatorDirection direction) {
    double upSpeed = 15;
    double downSpeed = 10;
    var currentPosition = motor1.getPosition().getValueAsDouble();
    var isSwitchingDirections = direction != previousDirection;

    isMovingDown = false;
    isManualMoving = true;

    if(isSwitchingDirections) {
      easeTimer.reset();
      easeTimer.start();
    }

    heightSetpoint = Optional.empty();
    heightPID.setSetpoint(0);

    switch(direction) {
      case UP: {
        desiredVelocity = upSpeed;
        break;
      }
      case DOWN: {
        desiredVelocity = -downSpeed;
        isMovingDown = true;
        break;
      }
      case STOP: {
        desiredVelocity = 0;
        setVelocity(0);
        isManualMoving = false;
        break;
      }
      default: {
        desiredVelocity = 0;
        setVelocity(0);
        isManualMoving = false;
        break;
      }
    }

    previousDirection = direction;
  }
}
