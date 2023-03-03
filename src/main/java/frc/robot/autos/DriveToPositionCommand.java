package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

/**
 * This should allow for attempting to drive to a specific position
 */
public class DriveToPositionCommand extends CommandBase {
    private Swerve swerve;

    public DriveToPositionCommand(Swerve swerveSubsystem){
        swerve = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize(){
        swerve.resetRelativeEncoders();
    }

    @Override
    public void execute(){
        
    }

    @Override
    public void end(boolean interrupted){

    }
}
