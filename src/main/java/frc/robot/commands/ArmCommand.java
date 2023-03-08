package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmCommand extends CommandBase{
    private Arm arm;
    public ArmCommand(Arm armSubSystem){
        this.arm = armSubSystem;
        addRequirements(armSubSystem);
    }

    @Override
    public void initialize(){
        arm.resetArmEncoder();
    }

    @Override
    public void execute(){
        SmartDashboard.putNumber("Arm Motor Encoder", arm.getEncoder());
    }

    @Override 
    public void end(boolean interrupted){
        arm.stop();
    }
}
