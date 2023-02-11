package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;

public class DefaultLimelightCommand extends CommandBase {
    private Limelight limelight;

    public DefaultLimelightCommand(Limelight limelight) {
        this.limelight = limelight;
        addRequirements(this.limelight);
    }

    @Override
    public void initialize() {
        limelight.setStreamMode(Limelight.STREAM_MODE_PIP_SECONDARY);
        limelight.ledOn();
    }

    @Override
    public void end(boolean interrupted) {
        limelight.ledOff();
    }
}
