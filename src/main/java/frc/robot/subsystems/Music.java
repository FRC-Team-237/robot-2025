package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Music extends SubsystemBase {
  private final Orchestra orchestra = new Orchestra();

  public Music() {
    TalonFX[] motors = new TalonFX[] {
      new TalonFX(1),
      new TalonFX(2),
      new TalonFX(3),
      new TalonFX(4),
      new TalonFX(5),
      new TalonFX(6),
      new TalonFX(7),
      new TalonFX(8),
      new TalonFX(30),
      new TalonFX(31)
    };

    
    for (TalonFX motor : motors) {
      motor.getConfigurator().apply(new AudioConfigs().withAllowMusicDurDisable(true));
      orchestra.addInstrument(motor);
      
    }

    SmartDashboard.putData("Music/Imperial March", new InstantCommand(this::play));
  }

  public void play() {
    var status = orchestra.loadMusic("march.chrp");
    if(!status.isOK()) {
      System.out.println(status.getDescription());
      return;
    }

    orchestra.play();
  }
}
