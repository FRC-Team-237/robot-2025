// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.*;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static final CTREConfigs ctreConfigs = new CTREConfigs();

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    CameraServer.startAutomaticCapture();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    var panelButtonCount = m_robotContainer.interlink.getButtonCount();
    for(var i = 1; i <= panelButtonCount; i++) {
      SmartDashboard.putBoolean("BUTTON/" + i, m_robotContainer.interlink.getRawButton(i));
    }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    // m_robotContainer.elevator.letGo();
    Elevator.getInstance().drop();
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    Swerve.getInstance().clearAngleSetpoint();

    // m_robotContainer.claw.getClawOutOfTheWay();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.claw.getClawOutOfTheWay();
    
    new InstantCommand(() -> {
      var alliance = DriverStation.getAlliance();
      double color = LEDs.BLINK_WHITE;
      if(alliance.isPresent()) {
        if(alliance.get() == DriverStation.Alliance.Blue) {
          color = LEDs.BLINK_BLUE;
        } else if(alliance.get() == DriverStation.Alliance.Red) {
          color = LEDs.BLINK_RED;
        }
      }
      LEDs.set(color);
    })
    .andThen(new WaitCommand(0.8))
    .andThen(new InstantCommand(() -> {
      LEDs.set(LEDs.getDefaultColor());
    }))
    .schedule();
  }

  private Optional<Double> lastLEDColor = LEDs.currentColor;

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    var t = Camera.getLatestResult();

    if(t.isPresent()) {
      if(LEDs.currentColor.isPresent() && LEDs.currentColor.get() != LEDs.LIME) {
        lastLEDColor = LEDs.currentColor;
      }
      LEDs.set(LEDs.LIME);
    } else {
      if(lastLEDColor.isPresent()) {
        LEDs.set(lastLEDColor.get());
      }
    }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
