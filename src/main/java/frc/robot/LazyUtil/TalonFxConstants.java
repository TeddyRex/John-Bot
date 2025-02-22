// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.LazyUtil;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

// Taken from 364 Code (Because im too lazy to do it myself)
// Used for Lazy Talon for ease 

public class TalonFxConstants {
    public final int deviceNumber;
    public final TalonFXConfiguration allConfigs;
    public final NeutralModeValue neutralMode;
    public final InvertedValue invertType;
    public final boolean slowStatusFrame;    
    
    /**
     * Constants to be used with LazyTalonFX Util
     * @param deviceNumber
     * @param allConfigs
     * @param neutralMode
     * @param invertType
     * @param slowStatusFrames
     */
    public TalonFxConstants(int deviceNumber, TalonFXConfiguration allConfigs, NeutralModeValue neutralMode, InvertedValue invertType, boolean slowStatusFrame) {
        this.deviceNumber = deviceNumber;
        this.allConfigs = allConfigs;
        this.neutralMode = neutralMode;
        this.invertType = invertType;
        this.slowStatusFrame = slowStatusFrame;
    }
}
