package frc.robot.commands.AutoCommand;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AutoCommand.*;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoCommandGenerator {
    public static Command generate(int firstChoice, int SecondChoice, int thirdChoice){
        switch(firstChoice){
            case 1:
                return new CU11X(SecondChoice, thirdChoice);
            case 2:
                return new C13X(SecondChoice,thirdChoice);
            default:
                break;
        }
        return new WaitCommand(0);
    }
}
