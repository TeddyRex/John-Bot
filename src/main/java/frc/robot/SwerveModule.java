// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.rmi.server.RemoteObjectInvocationHandler;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
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
    //private final RelativeEncoder m_steerIntegratedEncoder;
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
    private final CANcoderConfiguration steerEncoderConfig = new CANcoderConfiguration();
    //private final TalonFXConfiguration driveEncoderConfig = new TalonFXConfiguration(); 

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

            /* Motor Stuff */
            m_steerMotor = new SparkMax(steer_ID, MotorType.kBrushless);
            m_driveMotor = new TalonFX(drive_ID, "canivore"); //change to constants later
            
            steerConfig.inverted(steerMotorReversed)
                .smartCurrentLimit(steerEncoder_ID);
            
            m_driveMotor.getConfigurator().apply(driveConfig);
            m_driveMotor.getConfigurator().setPosition(0.0);
            m_driveMotor.setInverted(driveMotorReversed); // Deprecated Method ~ CHANGE TO CONFIGS

            m_steerMotor.configure(steerConfig, null, PersistMode.kPersistParameters); // RESET MODE DOES NOT WORK WITH SPARKMAX???

            /* PID Stuff */
            m_pidController = new PIDController(0, 0, 0);
            m_pidController.enableContinuousInput(-Math.PI, Math.PI);

            /* Encoder Stuff */
            m_steerEncoder = new CANcoder(steerEncoder_ID, "rio"); //change to constants later
            m_driveEncoder = new Encoder(null, null); //change to constants 
            
            steerConfig.encoder.positionConversionFactor(0);
            steerConfig.encoder.velocityConversionFactor(0);

            // Need to add drive encoder postion conversion factor
            m_steerEncoder.getConfigurator().apply(steerEncoderConfig);

            /* Gyro */
            m_gyro = new Pigeon2(0, "rio"); //change to constants later
            
    }

    public double getDrivePos() {
        return m_driveMotor.getPosition().getValueAsDouble();
    }

    public double getSteerPos() {
        return m_steerEncoder.getPosition().getValueAsDouble();
    }

    public double getDriveVel() {
        return m_driveMotor.getVelocity().getValueAsDouble();
    }

    public double getSteerVel() {
        return m_steerEncoder.getPosition().getValueAsDouble();
    }

    public double getAbsEncoderRad() {
        double angle = m_steerEncoder.getSupplyVoltage().getValueAsDouble() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= steerEncoderMagnetOffset;
        return angle * (steerEncoderReversed ? -1.0 : 1.0); //Multiply negative 1 if reversed
    }

    public void resetEncoders(){
        
        
    }
}
