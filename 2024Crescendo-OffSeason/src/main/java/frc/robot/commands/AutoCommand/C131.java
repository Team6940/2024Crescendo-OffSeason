package frc.robot.commands.AutoCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoSPKUP;
import frc.robot.commands.NoteIntake;
import frc.robot.commands.TestSPKUP;
import frc.robot.subsystems.Chassis.DriveSubsystem;

public class C131 extends SequentialCommandGroup{
    public C131()
    {
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
        addCommands(RobotContainer.m_Swerve.followPathCommand(RobotContainer.m_Swerve.generateChoreoPath("C131-1")).raceWith(new NoteIntake(0)));
        addCommands(new TestSPKUP(-33,40,0).withTimeout(1.5));
        addCommands(RobotContainer.m_Swerve.followPathCommand(RobotContainer.m_Swerve.generateChoreoPath("C131-2")).raceWith(new NoteIntake(0)));
        addCommands(new AutoSPKUP(0).withTimeout(1.5));
        addCommands(RobotContainer.m_Swerve.followPathCommand(RobotContainer.m_Swerve.generateChoreoPath("C131-3")).raceWith(new NoteIntake(0)));
        addCommands(new AutoSPKUP(0).withTimeout(1.5));
        addCommands(RobotContainer.m_Swerve.followPathCommand(RobotContainer.m_Swerve.generateChoreoPath("C131-4")).raceWith(new NoteIntake(0)));
        addCommands(new AutoSPKUP(0).withTimeout(1.5));
    }
}
