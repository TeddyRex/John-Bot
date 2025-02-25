// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.Swerve.*;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase {
  private SwerveDriveOdometry m_odometry;

  /** Creates a new SwerveSubsystem. */
  private SwerveModule[] m_swerveModules = new SwerveModule[] {
    new SwerveModule(FR_STEER_ID, FR_DRIVE_ID, 
      false, false, 
      FR_ENCODER_ID, 0, false),
    new SwerveModule(BR_STEER_ID, BR_DRIVE_ID,
      false, false, 
      BR_ENCODER_ID, 0, false),
    new SwerveModule(FL_STEER_ID, FL_DRIVE_ID, 
      FL_STEER_INVERTED, BL_STEER_INVERTED, 
      FL_ENCODER_ID, 0, false),
    new SwerveModule(BL_STEER_ID, BL_DRIVE_ID, 
      BL_DRIVE_INVERTED, FL_STEER_INVERTED, 
      BL_ENCODER_ID, 0, false)};

    private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      FR_LOCATION,
      FL_LOCATION,
      BR_LOCATION,
      BL_LOCATION);

  private Pigeon2 m_gyro = new Pigeon2(PIGEON_ID);
  private Pose2d m_pose = new Pose2d();
  private SwerveModulePosition[] m_modulePositions = new SwerveModulePosition[4];

  @SuppressWarnings("removal")
  public SwerveSubsystem() {
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        zeroHeading();
      } catch (Exception e) {
      }
    }).start();

    m_odometry = new SwerveDriveOdometry
    (m_kinematics, Rotation2d.fromDegrees(m_gyro.getAngle()), m_modulePositions, m_pose);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Robot Heading", getHeading());
  }

  public void zeroHeading() {
    m_gyro.reset();
  }

  public double getHeading() {
    return Math.IEEEremainder(m_gyro.getYaw().getValueAsDouble(), 360);
  }

  public Rotation2d geRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  public void stopModules() {
    for (int i = 0; i == 4; i++) {
      m_swerveModules[i].stop();
    }
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_OUTPUT);
   for (int i = 0; i == 4; i++) {
    m_swerveModules[i].setDesiredState(desiredStates[i]);
   }
  }

  @SuppressWarnings("deprecation")
  public void driveRobotOriented(ChassisSpeeds speeds) {
    speeds = ChassisSpeeds.discretize(speeds, 0.020);
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(speeds);

    for(int i = 0; i < 4; i++) {
      m_swerveModules[i].setDesiredState(SwerveModuleState.optimize(
          states[i], m_modulePositions[i].angle));
    }
  }

  @SuppressWarnings("removal")
  public void driveFieldOriented(ChassisSpeeds speeds) {
    driveRobotOriented(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, Rotation2d.fromDegrees(m_gyro.getAngle())));
  }

  public void zeroYaw() {
    m_gyro.setYaw(0);
  }
}
