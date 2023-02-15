package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase{
    private CANSparkMax armMotor;
    private CANCoder armCanCoder;

    public Arm(int armCanID, int armCanCoderID){
        armMotor = new CANSparkMax(armCanID, MotorType.kBrushless);
        armCanCoder = new CANCoder(armCanCoderID);
    }

    public void moveUp(){
        armMotor.set(0.02);
    }

    public void stop(){
        armMotor.set(0);
    }

    public void moveDown(){
        armMotor.set(-0.02);
    }

    public void highGoal(){
        
    }

    public void midGoal(){

    }

    public void lowestPoint(){

    }

    public void humanPlayerStation(){

    }



}
