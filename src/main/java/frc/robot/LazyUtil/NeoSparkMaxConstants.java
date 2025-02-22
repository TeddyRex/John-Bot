package frc.robot.LazyUtil;

import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;

public class NeoSparkMaxConstants {
    public final double wheelDiameter;
    public final double wheelCircumference;
    public final double angleGearRatio;
    public final double angleKP;
    public final double angleKI;
    public final double angleKD;
    public final double angleKF;
    public final boolean angleMotorInvert;
    public SensorDirectionValue cancoderInvert;
    //* Taken From 364 Code, removed Drive (Drive is used for TalonFX instead) */
    public NeoSparkMaxConstants(
        double wheelDiameter, 
        double angleGearRatio,
        double angleKP,
        double angleKI,
        double angleKD, 
        double angleKF,  
        boolean angleMotorInvert, 
        SensorDirectionValue canCoderInvert){

        this.wheelDiameter = wheelDiameter;
        this.wheelCircumference = wheelDiameter * Math.PI;
        this.angleGearRatio = angleGearRatio;
        this.angleKP = angleKP;
        this.angleKI = angleKI;
        this.angleKD = angleKD;
        this.angleKF = angleKF;
        this.angleMotorInvert = angleMotorInvert;
        this.cancoderInvert = canCoderInvert;
    }
    
    public static NeoSparkMaxConstants SDSMK4(double driveGearRatio){
        double wheelDiameter = Units.inchesToMeters(4.0);
 
        /** 12.8 : 1 */
        double angleGearRatio = (12.8 / 1.0);
 
        double angleKP = 0.2;
        double angleKI = 0.0;
        double angleKD = 0.0;
        double angleKF = 0.0;
 
        boolean angleMotorInvert = false;
        SensorDirectionValue cancoderInvert = SensorDirectionValue.CounterClockwise_Positive;
        return new NeoSparkMaxConstants(wheelDiameter, angleGearRatio, angleKP, angleKI, angleKD, angleKF, angleMotorInvert, cancoderInvert);
    }
}