// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.LimelightHelpers.PoseEstimate;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // Update vision pose estimation (MegaTag2)
    updateVisionMeasurement();
  }

  /**
   * Updates robot pose estimation using AprilTag vision (MegaTag2)
   * Called every robot periodic loop (~20ms / 50Hz)
   * Disabled in simulation - no Limelight hardware
   */
  private void updateVisionMeasurement() {
    // Skip vision in simulation - Limelight hardware not present
    if (Utils.isSimulation()) {
      return;
    }

    // Get current robot state
    double currentYawDegrees = m_robotContainer.drivetrain.getState().Pose.getRotation().getDegrees();
    double angularVelocityDegPerSec = Math.toDegrees(m_robotContainer.drivetrain.getState().Speeds.omegaRadiansPerSecond);

    // Get MegaTag2 pose estimate with gyro fusion
    PoseEstimate mt2 = m_robotContainer.vision.getMegaTag2Estimate(currentYawDegrees);

    // Check if we should reject this vision update
    boolean rejectUpdate = m_robotContainer.vision.shouldRejectVisionUpdate(
        angularVelocityDegPerSec,
        mt2.tagCount
    );

    // Add vision measurement to pose estimator if valid
    if (!rejectUpdate) {
      m_robotContainer.drivetrain.addVisionMeasurement(
          mt2.pose,
          mt2.timestampSeconds,
          m_robotContainer.vision.getVisionStdDevs(mt2.tagCount)
      );
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
