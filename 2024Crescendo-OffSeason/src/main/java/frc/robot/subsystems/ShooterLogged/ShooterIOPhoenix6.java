package frc.robot.subsystems.ShooterLogged;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.ShooterConstants;

public class ShooterIOPhoenix6 implements ShooterIOInterface{
    private static TalonFX m_leftMotor;
    private static TalonFX m_rghtMotor;

    private static final VoltageOut m_VoltageDutyCycle = new VoltageOut(0, false, false, false, false);
    private static final VelocityVoltage m_VelocityDutyCycle = new VelocityVoltage(0, 0, false, 0, 0, false, false, false);

    ShooterIOPhoenix6(){
        m_leftMotor = new TalonFX(ShooterConstants.ShooterLeft_ID, "*");
        m_rghtMotor = new TalonFX(ShooterConstants.ShooterRight_ID, "*");

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.PeakReverseDutyCycle = 0.0;
        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0;
        config.Slot0.kP = ShooterConstants.kP;
        config.Slot0.kI = ShooterConstants.kI;
        config.Slot0.kD = ShooterConstants.kD;
        config.Slot0.kV = ShooterConstants.kV;
        config.Slot0.kS = ShooterConstants.kS;
        
        m_leftMotor.getConfigurator().apply(config);
        m_rghtMotor.getConfigurator().apply(config);

        m_leftMotor.setInverted(true);
        m_rghtMotor.setInverted(false);

    }

    @Override
    public void setVoltage(double lVoltage, double rVoltage){
        m_leftMotor.setControl(m_VoltageDutyCycle.withOutput(lVoltage));
        m_rghtMotor.setControl(m_VoltageDutyCycle.withOutput(rVoltage));
    }

    @Override
    public void setRPS(double lRPS, double rRPS){
        m_leftMotor.setControl(m_VelocityDutyCycle.withVelocity(lRPS));
        m_rghtMotor.setControl(m_VelocityDutyCycle.withVelocity(rRPS));
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs){
        
        inputs.leftconnected = m_leftMotor.isAlive();
        inputs.rghtconnected = m_rghtMotor.isAlive();   //Not sure which one is correct

        // inputs.leftconnected=BaseStatusSignal.refreshAll(
        //     m_leftMotor.getMotorVoltage(),
        //     m_leftMotor.getVelocity(),
        //     m_leftMotor.getSupplyCurrent()
        // ).isOK();
        // inputs.rghtconnected=BaseStatusSignal.refreshAll(
        //     m_rghtMotor.getMotorVoltage(),
        //     m_rghtMotor.getVelocity(),
        //     m_rghtMotor.getSupplyCurrent()
        // ).isOK();

        inputs.leftVoltage = m_leftMotor.getMotorVoltage().getValue();
        inputs.rghtVoltage = m_rghtMotor.getMotorVoltage().getValue();
        inputs.leftSensorRPS = m_leftMotor.getVelocity().getValue();
        inputs.rghtSensorRPS = m_rghtMotor.getVelocity().getValue();
        inputs.leftCurrentAmps = m_leftMotor.getSupplyCurrent().getValue();
        inputs.rghtCurrentAmps = m_rghtMotor.getSupplyCurrent().getValue();
    }
}
