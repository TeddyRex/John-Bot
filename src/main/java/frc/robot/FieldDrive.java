// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FieldDrive extends Command {
  private SwerveSubsystem m_swerveSubsystem;
  private DoubleSupplier m_forward;
  private DoubleSupplier m_strafe;
  private DoubleSupplier m_rotation;

  /** Creates a new FieldDrive. */
  public FieldDrive(SwerveSubsystem subsystem, DoubleSupplier forward, 
                    DoubleSupplier strafe, DoubleSupplier rotation) {
    m_swerveSubsystem = subsystem;
    m_forward = forward;
    m_strafe = strafe;
    m_rotation = rotation;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_swerveSubsystem.driveFieldOriented(new ChassisSpeeds(
        m_forward.getAsDouble(), m_strafe.getAsDouble(), m_rotation.getAsDouble()));
  }

}
