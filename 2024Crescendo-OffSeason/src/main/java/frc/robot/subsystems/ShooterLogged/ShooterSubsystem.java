package frc.robot.subsystems.ShooterLogged;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.AutoPassNoteConstants;
import frc.robot.Constants.AutoShootConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterLogged.ShooterIOInterface.ShooterIOInputs;

import javax.swing.text.StyledEditorKit.BoldAction;

import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {

    private static ShooterSubsystem m_Instance = null;

    private final ShooterIOInterface io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private double m_leftTargetRPS = 0., m_rghtTargetRPS = 0.;

    ShooterSubsystem() {
        if (Robot.isReal()) {
            io = new ShooterIOPhoenix6();
        } else {
            // TODO Simulation
            io = new ShooterIOPhoenix6();
        }

        // Process the Interpolation table
        for (var a : AutoShootConstants.ArmPoints)
            AutoShootConstants.ArmTable.put(a.getX(), a.getY());
        for (var a : AutoShootConstants.RPSPoints)
            AutoShootConstants.RPSTable.put(a.getX(), a.getY());
        for (var a : AutoShootConstants.DisToArmPoints)
            AutoShootConstants.DisToArmTable.put(a.getX(), a.getY());
        for (var a : AutoShootConstants.DisToRPSPoints)
            AutoShootConstants.DisToRPSTable.put(a.getX(), a.getY());
        for (var a : AutoPassNoteConstants.DisToArmPoints)
            AutoPassNoteConstants.DisToArmTable.put(a.getX(), a.getY());

    }

    public void setRPS(double leftRPS, double rghtRPS) {
        m_leftTargetRPS = leftRPS;
        m_rghtTargetRPS = rghtRPS;
        io.setRPS(leftRPS, rghtRPS);
    }

    public void setRPS(double RPS) {
        m_leftTargetRPS = RPS;
        m_rghtTargetRPS = RPS + 5;
        io.setRPS(m_leftTargetRPS, m_rghtTargetRPS);
    }
    
    public void stop(){
        setRPS(0, 0);
    }

    /** THIS SHOULD NOT BE USED IN NORMAL OPERATION */
    public void setVoltage(double leftVoltage, double rghtVoltage){
        io.setVoltage(leftVoltage, rghtVoltage);
    }

    /** THIS SHOULD NOT BE USED IN NORMAL OPERATION */
    public void SetPCT(double leftPCT, double rghtPCT){
        io.setVoltage(leftPCT*12., rghtPCT*12.);
    }

    public boolean isLeftAtTargetRPS() {
        return MathUtil.isNear(m_leftTargetRPS, inputs.leftSensorRPS, ShooterConstants.ShooterSpeedTolerence);
    }

    public boolean isRghtAtTargetRPS() {
        return MathUtil.isNear(m_rghtTargetRPS, inputs.rghtSensorRPS, ShooterConstants.ShooterSpeedTolerence);
    }

    public boolean isAtTargetRPS() {
        return isLeftAtTargetRPS() && isRghtAtTargetRPS();
    }

    @Override
    public void periodic() {
        processLog();
        processDashboard();

    }

    private void processLog() {
        // Log the sensor inputs
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        // Log the outputs
        Logger.recordOutput("Shooter/LeftTargetRPS", m_leftTargetRPS);
        Logger.recordOutput("Shooter/RghtTargetRPS", m_rghtTargetRPS);

        // Log the status
        Logger.recordOutput("Shooter/AtTargetRPS", isAtTargetRPS());
        // Logger.recordOutput("Shooter/LeftAtTargetRPS", isLeftAtTargetRPS());
        // Logger.recordOutput("Shooter/RghtAtTargetRPS", isRghtAtTargetRPS());
    }

    private void processDashboard() {
        // TODO Dashboard
        SmartDashboard.putNumber("Shooter/LeftRPS", inputs.leftSensorRPS);
        SmartDashboard.putNumber("Shooter/RghtRPS", inputs.rghtSensorRPS);
        SmartDashboard.putNumber("Shooter/LeftTargetRPS", m_leftTargetRPS);
        SmartDashboard.putNumber("Shooter/RghtTargetRPS", m_rghtTargetRPS);
    }

    public static ShooterSubsystem getInstance() {
        return m_Instance == null ? m_Instance = new ShooterSubsystem() : m_Instance;
    }

}
