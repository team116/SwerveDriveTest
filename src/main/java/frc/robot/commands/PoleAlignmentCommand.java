package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class PoleAlignmentCommand extends CommandBase {
    private Swerve swerve;
    private Limelight limelight;

    public PoleAlignmentCommand(Swerve swerveSubstem, Limelight limelightSubsystem) {
        this.swerve = swerveSubstem;
        this.limelight = limelightSubsystem;
        addRequirements(swerveSubstem, limelightSubsystem);
    }

    @Override
    public void initialize() {
        limelight.ledOn();  // NOTE: Not sure what the delay is between asking to turn on and it being on
    }

    @Override
    public void execute() {
        double offsetAngleDegrees = limelight.horizontalOffsetFromCrosshairAsDegrees();
        while (stillNeedToMove(offsetAngleDegrees)) {
            double metersPerSecondSpeed = 0.5d;
            if (offsetAngleDegrees < 0.0d) {
                metersPerSecondSpeed = -metersPerSecondSpeed;
            }
            swerve.drive(new Translation2d(0.0d, metersPerSecondSpeed), 0, false, false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        limelight.ledOff();
    }

    private boolean stillNeedToMove(double offsetAngleDegrees) {
        return limelight.hasValidTarget() &&
               Math.abs(limelight.horizontalOffsetFromCrosshairAsDegrees()) > 0.25d;
    }
}
