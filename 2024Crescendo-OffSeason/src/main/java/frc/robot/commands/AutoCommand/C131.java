package frc.robot.commands.AutoCommand;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoSPKUP;
import frc.robot.commands.NoteIntake;
import frc.robot.subsystems.Chassis.DriveSubsystem;

public class C131 extends SequentialCommandGroup{
    public C131()
    {
        addCommands(new InstantCommand(()->RobotContainer.m_Swerve.setPose(RobotContainer.m_Swerve.generateChoreoPath("C131-1").getStartingDifferentialPose())));
        addCommands(new AutoSPKUP(0).withTimeout(0.7));
        addCommands(RobotContainer.m_Swerve.followPathCommand(RobotContainer.m_Swerve.generateChoreoPath("C131-1")).alongWith(new NoteIntake(0)));
        addCommands(new AutoSPKUP(0).withTimeout(0.7));
        addCommands(RobotContainer.m_Swerve.followPathCommand(RobotContainer.m_Swerve.generateChoreoPath("C131-2")).alongWith(new NoteIntake(0)));
        addCommands(new AutoSPKUP(0).withTimeout(0.7));
        addCommands(RobotContainer.m_Swerve.followPathCommand(RobotContainer.m_Swerve.generateChoreoPath("C131-3")).alongWith(new NoteIntake(0)));
        addCommands(new AutoSPKUP(0).withTimeout(0.7));
        addCommands(RobotContainer.m_Swerve.followPathCommand(RobotContainer.m_Swerve.generateChoreoPath("C131-4")).alongWith(new NoteIntake(0)));
        addCommands(new AutoSPKUP(0).withTimeout(0.7));
    }
}
