package frc.robot.commands.AutoCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.IntakeCommands.NoteIntake;
import frc.robot.commands.SPKCommands.AutoSPKUP;
import frc.robot.commands.SPKCommands.NewAutoSPKUP;
import frc.robot.commands.SPKCommands.TestSPKUP;
import frc.robot.subsystems.Chassis.DriveSubsystem;

public class C13X extends SequentialCommandGroup{
    int FirstP,SecondP;
    public C13X(int _FirstP,int _SecondP)
    {
        FirstP=_FirstP;
        SecondP=_SecondP;
        if(DriverStation.getAlliance().get()==DriverStation.Alliance.Blue)
        { addCommands(new InstantCommand(()->RobotContainer.m_Swerve.setPose(RobotContainer.m_Swerve.generateChoreoPath("C131-1").getPreviewStartingHolonomicPose())));
        
      
             }
        else
        {
            Pose2d m_Pose=RobotContainer.m_Swerve.generateChoreoPath("C131-1").flipPath().getPreviewStartingHolonomicPose();
            Pose2d flipm_Pose=new Pose2d(m_Pose.getX(), m_Pose.getY(), m_Pose.getRotation().plus(new Rotation2d(Math.PI)));
 addCommands(new InstantCommand(()->RobotContainer.m_Swerve.setPose(flipm_Pose)));
        
      
        }
        addCommands(new TestSPKUP(-33,40,0).withTimeout(1.5));
        if(SecondP==0)
        {
        addCommands(RobotContainer.m_Swerve.followPathCommand(RobotContainer.m_Swerve.generateChoreoPath("C131-1")).raceWith(new NoteIntake(0)));
        addCommands(new TestSPKUP(-40,40,0).withTimeout(1.5));
        addCommands(RobotContainer.m_Swerve.followPathCommand(RobotContainer.m_Swerve.generateChoreoPath("C131-2")).raceWith(new NoteIntake(0)));
        addCommands(new TestSPKUP(-33,40,0).withTimeout(1.5));
        addCommands(RobotContainer.m_Swerve.followPathCommand(RobotContainer.m_Swerve.generateChoreoPath("C131-3")).raceWith(new NoteIntake(0)));
        addCommands(new NewAutoSPKUP(0).withTimeout(1.5));
        
        }
        else
        {
            addCommands(RobotContainer.m_Swerve.followPathCommand(RobotContainer.m_Swerve.generateChoreoPath("C131-2F")).raceWith(new NoteIntake(0)));
            addCommands(new NewAutoSPKUP(0).withTimeout(1.5));
        
        }
        if(FirstP!=0){
            addCommands(RobotContainer.m_Swerve.followPathCommand(RobotContainer.m_Swerve.generateChoreoPath("CU103-P"+Integer.toString(FirstP)))
                    .raceWith(new NoteIntake(0)));
            addCommands(new NewAutoSPKUP(0).withTimeout(1.5));
        }
        if(SecondP!=0){
            addCommands(RobotContainer.m_Swerve.followPathCommand(RobotContainer.m_Swerve.generateChoreoPath("CU103-P"+Integer.toString(SecondP)))
                    .raceWith(new NoteIntake(0)));
            addCommands(new NewAutoSPKUP(0).withTimeout(1.5));
        }
    }
}
