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

public class Robot extends TimedRobot {
  private Command autoCommand;

  private RobotContainer robotContainer;

  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();
    robotContainer.CANDLES.setAnimationBoth(
        new LarsonAnimation(255, 255, 0, 0, 1.0, 8, BounceMode.Front, 1));
    PathfindingCommand.warmupCommand().schedule();
  }

  @Override
  public void robotPeriodic() {
    // robotContainer.updatePoseVision();
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    robotContainer.CANDLES.setAnimationBoth(new SingleFadeAnimation(255, 0, 0, 0, 0.7, 8));
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    // m_robotContainer.CANDLES.setAnimationBoth(new SingleFadeAnimation(255, 255, 255, 0, 1.5, 8));
    robotContainer.CANDLES.setAnimationRight(
        new LarsonAnimation(255, 255, 255, 0, 0.1, 8, BounceMode.Front, 1));
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
