// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.pathplanner.lib.commands.PathfindingCommand;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Candles.CandleSide;

public class Robot extends TimedRobot {
  private Command autoCommand;

  private RobotContainer robotContainer;

  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();
    robotContainer.CANDLES.setAnimation(
        CandleSide.BOTH, new LarsonAnimation(255, 255, 0, 0, 1.0, 8, BounceMode.Front, 1));
    PathfindingCommand.warmupCommand().schedule();
  }

  @Override
  public void robotPeriodic() {
    RobotContainer.DRIVETRAIN.updateOdometry();
    // robotContainer.updatePoseVision();
    RobotContainer.DRIVETRAIN.megatag2Update();
    CommandScheduler.getInstance().run();
    // System.out.println("LobRpm: " + RobotContainer.get().lobRPM.getDouble(100.0));
    // System.out.println("Distance to Lob: " +
    // Util.getDistanceToAllianceLob(RobotContainer.get().getPose()));
  }

  @Override
  public void disabledInit() {
    robotContainer.CANDLES.setAnimation(
        CandleSide.BOTH, new SingleFadeAnimation(255, 0, 0, 0, 0.7, 8));
    robotContainer.DRIVETRAIN.setShouldMegatag2Update(false);
  }

  @Override
  public void disabledPeriodic() {
    // System.out.println(Util.getRotationToAllianceSpeaker(new Pose2d(new Translation2d(7.21,
    // 2.24), new Rotation2d(90.0))));
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    // m_robotContainer.CANDLES.setAnimationBoth(new SingleFadeAnimation(255, 255, 255, 0, 1.5, 8));
    robotContainer.CANDLES.setAnimation(
        CandleSide.RIGHT, new LarsonAnimation(255, 255, 255, 0, 0.1, 8, BounceMode.Front, 1));
    autoCommand = robotContainer.getAutonomousCommand();
    if (autoCommand != null) {
      autoCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    robotContainer.CANDLES.stop();
    robotContainer.configureDefaultCommands();
    if (autoCommand != null) {
      autoCommand.cancel();
    }
    robotContainer.DRIVETRAIN.setShouldMegatag2Update(true);
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
