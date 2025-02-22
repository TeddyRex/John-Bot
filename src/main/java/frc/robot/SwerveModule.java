// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

/** Add your docs here. */

/*
 * The Code would be utilizing a Kraken Drive & NEO Rotation
 */
public class SwerveModule {
    // NEO and Kraken motor controllers
    private final SparkMax m_steerMotor;
    private final TalonFX m_driveMotor;
    // The robot's swerve and steer encoders
    private final CANcoder m_steerEncoder;
    private final Encoder m_driveEncoder;
    // Robot Gyroscope
    private final Pigeon2 m_gyro;
    // Steer PID
    private final PIDController m_pidController;
    // Inverting
    private final boolean steerEncoderReversed;
    private final double steerEncoderMagnetOffset;
    // Drive and steer configs
    private final SparkMaxConfig steerConfig = new SparkMaxConfig();
    private final TalonFXConfiguration driveConfig = new TalonFXConfiguration();

    @SuppressWarnings("removal")
    public SwerveModule(
        int steer_ID, 
        int drive_ID,
        boolean driveMotorReversed, 
        boolean steerMotorReversed, 
        int steerEncoder_ID, 
        double magnetOffset, 
        boolean steerEncoderReversed) 
        // it was too long so i cut it down a bit
        {
            this.steerEncoderMagnetOffset = magnetOffset;
            this.steerEncoderReversed = steerEncoderReversed;

            m_steerMotor = new SparkMax(steer_ID, MotorType.kBrushless);
            m_driveMotor = new TalonFX(drive_ID, "rio"); //change to constants later
            
            steerConfig.inverted(steerMotorReversed)
                .smartCurrentLimit(steerEncoder_ID);
            
            m_driveMotor.getConfigurator().apply(driveConfig);
            m_driveMotor.getConfigurator().setPosition(0.0);
            
            m_driveMotor.getConfigurator().apply(driveConfig);
            m_driveMotor.setInverted(driveMotorReversed); // Deprecated Method ~ CHANGE TO CONFIGS
            m_steerMotor.configure(steerConfig, null, PersistMode.kPersistParameters); // RESET MODE DOES NOT WORK WITH SPARKMAX???

            m_pidController = new PIDController(0, 0, 0);
            m_pidController.enableContinuousInput(Math.PI, Math.PI);

            m_steerEncoder = new CANcoder(steerEncoder_ID, "rio"); //change to constants later
            m_driveEncoder = new Encoder(null, null); //change to constants later

            m_gyro = new Pigeon2(0, "rio"); //change to constants later
            
    }

    public double getDrivePos() {
        return 0;
    }

    public double getSteerPos() {
        return 0;
    }

    public double getDriveVel() {
        return 0;
    }

    public double getSteerVel() {
        return 0;
    }

    public double getAbsEncoderRad() {
        return 0;
    }

    public void resetToAbsolute(){
        
    }
}
