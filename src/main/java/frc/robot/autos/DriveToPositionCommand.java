package frc.robot.autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

/**
 * This should allow for attempting to drive to a specific position
 */
public class DriveToPositionCommand extends CommandBase {
    private Swerve swerve;
    private int pidSlot = 1;
    private double distance = 72 / 39.37;
    private Rotation2d angle = new Rotation2d(0);
    private SwerveModulePosition [] listOfModulePositions = {new SwerveModulePosition(distance, angle), 
                                                            new SwerveModulePosition(distance, angle), 
                                                            new SwerveModulePosition(distance, angle),
                                                            new SwerveModulePosition(distance, angle)};
    public DriveToPositionCommand(Swerve swerveSubsystem){
        swerve = swerveSubsystem;
        swerve.setPID(10, 0, 0, pidSlot);
        swerve.setMinMax(-0.2, 0.2, pidSlot);
        swerve.burnFlash();
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize(){
        swerve.resetRelativeEncoders();
    }

    @Override
    public void execute(){
        swerve.setModulePositions(listOfModulePositions, pidSlot);;
    }

    @Override
    public void end(boolean interrupted){

    }
}
