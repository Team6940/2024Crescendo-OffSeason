package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.fasterxml.jackson.annotation.JsonGetter;

import frc.robot.Library.team2910.math.MathUtils;
import frc.robot.Library.team1678.math.Conversions;
import frc.robot.Library.NumberLimiter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase{
    public static Arm m_Instance;
    private static TalonFX m_ArmLeft = new TalonFX(ArmConstants.ArmLeft_ID, "canivore");
    private static TalonFX m_ArmRight = new TalonFX(ArmConstants.ArmRight_ID, "rio");

    private TalonFXConfiguration m_ArmLeftConfig = new TalonFXConfiguration();
    private TalonFXConfiguration m_ArmRightConfig = new TalonFXConfiguration();
    private MotionMagicDutyCycle m_MotionMagicDutyCycle = new MotionMagicDutyCycle(0, false, 0., 0, true, false, false);

    private double _rotation=ArmConstants.ArmDefaultDegree/360.;

    private double m_OffsetDeg = 0.;

    public static Arm GetInstance()
    {
        return m_Instance==null?m_Instance=new Arm():m_Instance;
    }

    Arm(){
        ArmConfig();
    }

    private void ArmConfig(){
        m_ArmLeftConfig.MotorOutput.NeutralMode=NeutralModeValue.Brake;
        m_ArmLeftConfig.MotorOutput.Inverted=InvertedValue.CounterClockwise_Positive;//TODO
        m_ArmLeftConfig.MotorOutput.PeakForwardDutyCycle=1;
        m_ArmLeftConfig.MotorOutput.PeakReverseDutyCycle=-1;
        m_ArmLeftConfig.Feedback.SensorToMechanismRatio=200;
        m_ArmLeftConfig.MotorOutput.DutyCycleNeutralDeadband=0.02;

        m_ArmLeftConfig.Slot0.kG=ArmConstants.kG;
        m_ArmLeftConfig.Slot0.kP=ArmConstants.kP;
        m_ArmLeftConfig.Slot0.kI=ArmConstants.kI;
        m_ArmLeftConfig.Slot0.kD=ArmConstants.kD;
        m_ArmLeftConfig.Slot0.kS=ArmConstants.kS;
        m_ArmLeftConfig.Slot0.GravityType=GravityTypeValue.Arm_Cosine;//TODO
        m_ArmLeftConfig.MotionMagic.MotionMagicAcceleration=ArmConstants.ArmAcceleration;
        m_ArmLeftConfig.MotionMagic.MotionMagicCruiseVelocity=ArmConstants.ArmVelocity;

        m_ArmRightConfig=m_ArmLeftConfig;
        m_ArmLeft.getConfigurator().apply(m_ArmLeftConfig);

        m_ArmRightConfig.MotorOutput.Inverted=InvertedValue.Clockwise_Positive;//TODO
        m_ArmRight.getConfigurator().apply(m_ArmRightConfig);

        m_ArmLeft.setPosition(ArmConstants.ArmDefaultDegree/360.);
        m_ArmRight.setPosition(ArmConstants.ArmDefaultDegree/360.);
    }

    public void ZeroArmPosition(){
        m_ArmLeft.setPosition(ArmConstants.ArmDefaultDegree/360.);
        m_ArmRight.setPosition(ArmConstants.ArmDefaultDegree/360.);
    }

    public void SetArmDegree(double _degree){
        _degree = NumberLimiter.Limit(-72.29, 100., _degree);
        _degree=_degree+m_OffsetDeg;
        _rotation=_degree/360.;
    }

    public void SetPCT(double _PCT){
        m_ArmLeft.set(_PCT);
        m_ArmRight.set(_PCT);
    }

    public double GetArmDegree(){
        return m_ArmLeft.getPosition().getValue()*360.;
    }

    public double GetTargetDegree(){
        return _rotation*360.;
    }

    public double GetArmOffsetDeg(){
        return m_OffsetDeg;
    }

    public void SetArmOffsetDeg(double _Deg){
        m_OffsetDeg=_Deg;
    }

    public boolean IsAtTargetDegree(){
        return Math.abs(GetArmDegree()-GetTargetDegree())<ArmConstants.ArmTolerence;
    }

    public boolean IsAtDefaultDegree(){
        return Math.abs(GetArmDegree()-ArmConstants.ArmDefaultDegree)<ArmConstants.ArmTolerence;
    }

    @Override
    public void periodic(){
        m_ArmLeft.setControl(m_MotionMagicDutyCycle.withPosition(_rotation));
        m_ArmRight.setControl(m_MotionMagicDutyCycle.withPosition(_rotation));
        SmartDashboard.putNumber("LeftArmPos", m_ArmLeft.getPosition().getValue());
        SmartDashboard.putNumber("RightArmPos", m_ArmRight.getPosition().getValue());
        SmartDashboard.putNumber("LeftArmAngle", m_ArmLeft.getPosition().getValue()*360);
        SmartDashboard.putNumber("RightArmAngle", m_ArmRight.getPosition().getValue()*360);
        SmartDashboard.putNumber("TargetRota", _rotation);
    }
}