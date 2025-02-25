// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private CommandXboxController m_controller;
  private SwerveSubsystem m_swerveSubsystem;

  private RobotDrive m_defaultDrive;
  private FieldDrive m_slowDrive;
  private FieldDrive m_fieldDrive;

  public RobotContainer() {
    m_swerveSubsystem = new SwerveSubsystem();

    m_slowDrive = new FieldDrive(
      m_swerveSubsystem,
      () -> 0.5 * getLeftY(),
      () -> 0.5 * getLeftX(),
      () -> 0.5 * getRightX());

  m_defaultDrive = new RobotDrive(
      m_swerveSubsystem,
      () -> 3 * getLeftY(),
      () -> 3 * getLeftX(),
      () -> 3 * getRightX()); 

  m_fieldDrive = new FieldDrive(
      m_swerveSubsystem,
      () -> 3 * getLeftY(),
      () -> 3 * getLeftX(),
      () -> 3 * getRightX());
      
    configureBindings();
  }

  private void configureBindings() {
    // drives slow while left bumper pressed
    m_controller.leftBumper().whileTrue(m_slowDrive);

    // toggles between robot and field oriented
    m_controller.a().toggleOnTrue(m_defaultDrive);

    // sets the forward facing angle for field oriented drive
    m_controller.y().onTrue(Commands.runOnce(m_swerveSubsystem::zeroYaw));
  }

  private double getLeftY() {
    double leftY = m_controller.getLeftY();
    if (Math.hypot(m_controller.getLeftX(), leftY) < 0.1) {
      return 0;
    }
    return leftY;
  }

  private double getLeftX() {
    double leftX = m_controller.getLeftX();
    if (Math.hypot(leftX, m_controller.getLeftY()) < 0.1) {
      return 0;
    }
    return leftX;
  }

  private double getRightX() {
    double rightX = m_controller.getRightX();
    if (Math.abs(rightX) < 0.1) {
      rightX = 0;
    }
    return rightX;
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
