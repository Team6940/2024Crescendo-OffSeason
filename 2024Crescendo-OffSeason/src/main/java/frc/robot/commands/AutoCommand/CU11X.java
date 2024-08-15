package frc.robot.commands.AutoCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.IntakeCommands.NoteIntake;
import frc.robot.commands.SPKCommands.NewAutoSPKUP;

public class CU11X extends SequentialCommandGroup{
    int FirstP = 0, SecondP = 0;

    public CU11X(int _FirstP, int _SecondP){
        if(DriverStation.getAlliance().get()==DriverStation.Alliance.Blue)
        { addCommands(new InstantCommand(()->RobotContainer.m_Swerve.setPose(RobotContainer.m_Swerve.generateChoreoPath("C103-1").getPreviewStartingHolonomicPose())));
        
      
        }
        else
        {
            Pose2d m_Pose=RobotContainer.m_Swerve.generateChoreoPath("C103-1").flipPath().getPreviewStartingHolonomicPose();
            Pose2d flipm_Pose=new Pose2d(m_Pose.getX(), m_Pose.getY(), m_Pose.getRotation().plus(new Rotation2d(Math.PI)));
            addCommands(new InstantCommand(()->RobotContainer.m_Swerve.setPose(flipm_Pose)));
        }
        FirstP = _FirstP;
        SecondP = _SecondP;
                addCommands(RobotContainer.m_Swerve.followPathCommand(RobotContainer.m_Swerve.generateChoreoPath("CU103-1"))
                .raceWith(new NoteIntake(0)));
        addCommands(new NewAutoSPKUP(0).withTimeout(1.5));
        addCommands(RobotContainer.m_Swerve.followPathCommand(RobotContainer.m_Swerve.generateChoreoPath("CU103-2"))
                .raceWith(new NoteIntake(0)));
        addCommands(new NewAutoSPKUP(0).withTimeout(1.5));
        addCommands(RobotContainer.m_Swerve.followPathCommand(RobotContainer.m_Swerve.generateChoreoPath("CU103-P"+Integer.toString(FirstP)))
                .raceWith(new NoteIntake(0)));
        addCommands(new NewAutoSPKUP(0).withTimeout(1.5));
        addCommands(RobotContainer.m_Swerve.followPathCommand(RobotContainer.m_Swerve.generateChoreoPath("CU103-P"+Integer.toString(SecondP)))
                .raceWith(new NoteIntake(0)));
        addCommands(new NewAutoSPKUP(0).withTimeout(1.5));

    }
    
}
