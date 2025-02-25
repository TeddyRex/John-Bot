// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.Swerve.*;
/** Add your docs here. */

/*
 * The Code would be utilizing a Kraken Drive & NEO Rotation
 */
public class SwerveModule {
    // NEO and Kraken motor controllers
    private final SparkMax m_steerMotor;
    private final TalonFX m_driveMotor;
    // The robot's swerve and steer encoders
    private final CANcoder m_steerAbsEncoder;
    private final RelativeEncoder m_steerIntegratedEncoder;
    // Steer PID
    private final PIDController m_pidController;
    // Inverting
    private final boolean steerEncoderReversed;
    private final double steerEncoderMagnetOffset;
    // Drive and steer configs
    private final TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    private final SparkMaxConfig steerConfig = new SparkMaxConfig();
    //misc
    private VelocityVoltage m_request = new VelocityVoltage(0);


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
            
            m_steerAbsEncoder = new CANcoder(steerEncoder_ID, "rio"); //change to constants later
            m_steerIntegratedEncoder = m_steerMotor.getEncoder();
            m_steerIntegratedEncoder.setPosition(m_steerAbsEncoder.getAbsolutePosition().getValueAsDouble());

            steerConfig.inverted(steerMotorReversed);
            steerConfig.encoder.positionConversionFactor(STEER_POSITION_CONVERSION);
            m_steerMotor.configure(steerConfig, null, PersistMode.kPersistParameters); // RESET MODE DOES NOT WORK WITH SPARKMAX???

            var driveConfigurator = m_driveMotor.getConfigurator();
            driveConfigurator.apply(new TalonFXConfiguration());
            driveConfigurator.apply(driveConfig);
            m_driveMotor.setInverted(driveMotorReversed); // Deprecated Method ~ CHANGE TO CONFIGS

            /* PID Stuff */
            m_pidController = new PIDController(STEER_kP, STEER_kI, STEER_kD);
            m_pidController.enableContinuousInput(-Math.PI, Math.PI);

            /* Magnet Sensor Configs */
            MagnetSensorConfigs m_magnetSensorConfig = new MagnetSensorConfigs();
            m_magnetSensorConfig.MagnetOffset = magnetOffset;
            m_magnetSensorConfig.SensorDirection = steerMotorReversed ? 
            SensorDirectionValue.CounterClockwise_Positive : SensorDirectionValue.Clockwise_Positive;
            m_steerAbsEncoder.getConfigurator().apply(m_magnetSensorConfig);

            var drivePIDFConfigs = new Slot0Configs(); 
            drivePIDFConfigs.kP = DRIVE_kP; 
            drivePIDFConfigs.kI = DRIVE_kI; 
            drivePIDFConfigs.kD = DRIVE_kD; 
            drivePIDFConfigs.kS = DRIVE_kS; 
            drivePIDFConfigs.kV = DRIVE_kV; 
            var driveRampConfigs = new ClosedLoopRampsConfigs();
            driveRampConfigs.VoltageClosedLoopRampPeriod = DRIVE_RAMP_RATE;
            var driveCurrentLimitConfigs = new CurrentLimitsConfigs();
            driveCurrentLimitConfigs.StatorCurrentLimit = DRIVE_CURRENT_LIMIT;
            driveCurrentLimitConfigs.StatorCurrentLimitEnable = true;
            driveConfigurator.apply(drivePIDFConfigs);
            driveConfigurator.apply(driveRampConfigs);
            driveConfigurator.apply(driveCurrentLimitConfigs);

            m_driveMotor.optimizeBusUtilization();
            m_driveMotor.getRotorPosition().setUpdateFrequency(50);
            m_driveMotor.getRotorVelocity().setUpdateFrequency(50);
            m_driveMotor.getStatorCurrent().setUpdateFrequency(50);
            m_driveMotor.getDutyCycle().setUpdateFrequency(50);
            
            resetEncoders();
    }

    public double getDrivePos() {
        return m_driveMotor.getPosition().getValueAsDouble();
    }

    public double getSteerPos() {
        return m_steerIntegratedEncoder.getPosition();
    }

    public double getDriveVel() {
        return m_driveMotor.getVelocity().getValueAsDouble();
    }

    public double getSteerVel() {
        return m_steerIntegratedEncoder.getVelocity();
    }

    public double getAbsEncoderRad() {
        double angle = m_steerAbsEncoder.getSupplyVoltage().getValueAsDouble() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= steerEncoderMagnetOffset;
        return angle * (steerEncoderReversed ? -1.0 : 1.0); //Multiply negative 1 if reversed
    }

    public void resetEncoders(){
        m_driveMotor.setPosition(0);
        m_steerIntegratedEncoder.setPosition(getAbsEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVel(), new Rotation2d(getSteerPos()));
    }

    public void stop() {
        m_driveMotor.set(0);
        m_steerMotor.set(0);
    }

    @SuppressWarnings("deprecation") //Might need to modify due to deprecated method
    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state, getState().angle);
        m_request.Velocity = state.speedMetersPerSecond / DRIVE_POSITION_CONVERSION;
        m_driveMotor.setControl(m_request);
        m_steerMotor.set(m_pidController.calculate(getSteerPos(), state.angle.getRadians()));
        // I HATE SMARTDASHBOARD GRR
        SmartDashboard.putString("Swerve: [" + m_steerAbsEncoder.getDeviceID() + "]", state.toString());
    }
}
