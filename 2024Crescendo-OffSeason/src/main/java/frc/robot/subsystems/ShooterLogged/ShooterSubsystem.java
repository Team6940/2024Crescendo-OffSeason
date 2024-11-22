package frc.robot.subsystems.ShooterLogged;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.ShooterLogged.ShooterIOInterface.ShooterIOInputs;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {

    private static ShooterSubsystem m_Instance = null;

    private final ShooterIOInterface io;
    // private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputs();

    ShooterSubsystem(){
        if(Robot.isReal()){
            io = new ShooterIOPhoenix6();
        }
        else{
            //TODO Simulation
            io = new ShooterIOPhoenix6();
        }
    }


    @Override
    public void periodic(){

    }


    public static ShooterSubsystem getInstance() {
        return m_Instance == null ? m_Instance = new ShooterSubsystem() : m_Instance;
    }

}
