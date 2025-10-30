// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import com.ctre.phoenix6.Orchestra;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.swerve.Orchestrate;
import frc.robot.subsystems.PhotonCameras;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private PhotonCameras m_cams;

  private AprilTagFieldLayout m_layout;

  public Robot() {
    try {
      m_layout = new AprilTagFieldLayout(
        "/home/lvuser/deploy/2025-reefscape.json"
      );
      m_cams = new PhotonCameras(m_layout);
    } catch (IOException exc) {
        System.out.println("Failed to load field layout!");
        m_cams = null;
    }

    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {

    try{
    Orchestra orchestra = new Orchestra();
    String midiString = "put midi here";
    Orchestrate music = new Orchestrate(m_robotContainer.drivetrain, orchestra, midiString);
    CommandScheduler.getInstance().schedule(music);
    } catch (Exception e){
        System.out.println("Failed to play music" + e.getMessage());
    }
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    try {
    if(m_cams.frontCamHasTarget()) {
        m_robotContainer.drivetrain.addVisionMeasurement(
          m_cams.getPoseRelativeFrontCam(),
          m_cams.getLatestTimeStamp()
        );
      }
    } catch (Exception e) {
        //System.out.println("Failed to add vision measurement");
    }
      try{
      SmartDashboard.putNumber("Camera area", m_cams.getFrontArea());
      SmartDashboard.putBoolean("Has target", m_cams.frontCamHasTarget());
      }
      catch (Exception e){
        SmartDashboard.putNumber("Camera area", -1);
        SmartDashboard.putBoolean("Has target", false);
      }
    }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
