package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * This should allow for attempting to drive to a specific position
 */
public class DoNothingCommand extends CommandBase {

    @Override
    public void initialize() {
        System.out.println("I will do nothing");    
    }

    @Override
    public void execute() {
        System.out.println("I am doing nothing");
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("I am done");
    }
    
}
