// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.LazyUtil;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

// Too lazy to do it all myself, taken from Team 364
public class LazyTalon extends TalonFX{
    public LazyTalon(int deviceNumber, TalonFXConfiguration allConfigs, boolean slowStatusFrame){
        super(deviceNumber);
        TalonFXConfigurator configurator = super.getConfigurator();
        configurator.apply(allConfigs);
        //Slow status womp womp
        if (slowStatusFrame){
            super.getPosition().setUpdateFrequency(255, 30);
        }
    }

    /**
     * Config using individual parameters. This should only be used for Swerve modules as they have their own SwerveModuleConstants file.
     * @param talonFXConstants Contains various motor initialization vars 
     * @param slowStatusFrame
     */
    public LazyTalon(TalonFxConstants talonFXConstants){
        super(talonFXConstants.deviceNumber);
        super.getConfigurator().apply(talonFXConstants.allConfigs);

        if (talonFXConstants.slowStatusFrame){
            super.getPosition().setUpdateFrequency(255, 30);
        }
    }
}
