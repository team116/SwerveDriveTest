package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Grabber extends SubsystemBase{
    private CANSparkMax grabberMotor;

    public Grabber(int canID){
        grabberMotor = new CANSparkMax(canID, MotorType.kBrushless);
    }

    public void intakeGamePiece(){
        grabberMotor.set(0.02);
    }

    public void getRidOfGamePiece(){
        grabberMotor.set(-0.02);
    }

    public void openClaw(){

    }

    public void closeClaw(){
        
    }
    
}
