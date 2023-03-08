package frc.robot.autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import java.lang.*;

/**
 * This should allow for attempting to drive to a specific position
 */
public class DriveToPositionCommand extends CommandBase {
    private static final double CONVERSION_FACTOR = 39.37;
    private Swerve swerve;
    private int pidSlot = 1;
    private double distance = 36 / CONVERSION_FACTOR;
    private int step = 0;
    Timer timer;
    private Rotation2d angle = Rotation2d.fromDegrees(0.0d);
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
        System.out.println("Initializing");
        swerve.resetRelativeEncoders();
        timer = new Timer();
        timer.start();
    }

    @Override
    public void execute(){
        switch(step){
            case 0:
                System.out.println("Executing step 0");
                swerve.setModulePositions(listOfModulePositions, pidSlot);
                if(timer.get() > 2){
                    step++;
                }
                break;
            case 1:
                System.out.println("Executing step 1");
                timer.reset();
                step++;
                break;  
            case 2:     
                System.out.println("Executing step 2");
                double newDistance = 24 / CONVERSION_FACTOR;
                Rotation2d newAngle = Rotation2d.fromDegrees(45);
                listOfModulePositions[0] = new SwerveModulePosition(newDistance, newAngle);
                listOfModulePositions[1] = new SwerveModulePosition(newDistance, newAngle);
                listOfModulePositions[2] = new SwerveModulePosition(newDistance, newAngle);
                listOfModulePositions[3] = new SwerveModulePosition(newDistance, newAngle);
                swerve.setModulePositions(listOfModulePositions, pidSlot);    
                step++;
                break;
            default:
                System.out.println("default");
                break;
        }                         
    }

    @Override
    public void end(boolean interrupted){
        System.out.println("end");
    }
}
