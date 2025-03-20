package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class Claw extends SubsystemBase {
  public final Joystick driver = new Joystick(0);
  private final JoystickButton raiseOverride = new JoystickButton(driver, XboxController.Button.kBack.value);

  private boolean isCorrecting = false; 
  private boolean lastMoveOveride = false; 

  private SparkMax clawIntake = new SparkMax(35, MotorType.kBrushless);
  public SparkMax clawPosition = new SparkMax(34, MotorType.kBrushless);
  private final DigitalInput zeroSensor = new DigitalInput(2);
  public Claw() {
    var intakeConfig = new SparkMaxConfig();
    intakeConfig.closedLoop.p(1);
    intakeConfig.closedLoop.i(0);
    intakeConfig.closedLoop.d(0);
    intakeConfig.idleMode(IdleMode.kBrake);

    clawIntake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    var positionConfig = new SparkMaxConfig();
    positionConfig.idleMode(IdleMode.kBrake);

    clawPosition.configure(positionConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void outtake() {
    clawIntake.set(1.0);
  }

  public void intake() {
    clawIntake.set(-1.0);
  }

  public void stopIntake() {
    clawIntake.set(0);
  }

  public void stopClawPosition() {
    clawPosition.set(0);
  }

  public void raiseClaw() {
    lastMoveOveride = raiseOverride.getAsBoolean();

    if(!zeroSensor.get()){
      clawPosition.set(0.05);
    } else {
      clawPosition.set(0.5);
    };
  }

  public boolean isZero() {
    return !zeroSensor.get();
  }

  public void lowerClaw() {
    lastMoveOveride = false;
    clawPosition.set(-0.5);
  }

  public void getClawOutOfTheWay() {
    lastMoveOveride = false;
  }

  @Override
  public void periodic() {
    if(!zeroSensor.get()) {
      clawPosition.getEncoder().setPosition(0);
      
      if(!raiseOverride.getAsBoolean() && !lastMoveOveride) {
        isCorrecting = true;
        clawPosition.set(-.1);
      } else if(clawPosition.get() > 0) {
        clawPosition.set(0.05);
      }
    } else {
      if(isCorrecting) {
        clawPosition.set(0);
        isCorrecting = false;
      }
    }

    if(clawPosition.getEncoder().getPosition() < -48) {
      if(clawPosition.get() < 0) {
        clawPosition.set(0);
      }
    }

    SmartDashboard.putNumber ("Claw/Pos",clawPosition.getEncoder().getPosition());
    SmartDashboard.putBoolean ("Claw/Zero",!zeroSensor.get());
  }
}
