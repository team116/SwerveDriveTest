package frc.robot.subsystems;

import java.lang.invoke.VarHandle.AccessMode;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase{
    private CANSparkMax armMotor;
    private CANCoder armCanCoder;
    private SparkMaxLimitSwitch armTopLimitSwitch;
    private SparkMaxLimitSwitch armBottomLimitSwitch;
    private SparkMaxPIDController armMotorController;
    private RelativeEncoder armEncoder;

    public enum Position{
        HIGH_GOAL(0.0), 
        MID_GOAL(0.0),
        HUMAN_PLAYER_STATION(0.0),
        PICK_UP(0.0),
        LOWEST_POINT(0.0),
        CHARGING_STATION(0.0),
        DRIVE(180);
        private final double angleDegrees;

        Position(double angleDegrees){
            this.angleDegrees = angleDegrees;
        }

        public double getAngleDegrees(){
            return angleDegrees;
        }
    }

    public Arm(int armCanID){
        armMotor = new CANSparkMax(armCanID, MotorType.kBrushless);
        armMotor.setIdleMode(IdleMode.kBrake);

        // armCanCoder = new CANCoder(armCanCoderID);

        armEncoder = armMotor.getEncoder();

        armTopLimitSwitch = armMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
        armTopLimitSwitch.enableLimitSwitch(true);
        armBottomLimitSwitch = armMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
        armBottomLimitSwitch.enableLimitSwitch(true);
        
        armMotorController = armMotor.getPIDController();
        armMotorController.setP(0.001);
        armMotor.burnFlash();
    }

    public void moveUp(){
        moveToPos(Position.DRIVE);
    }

    public void stop(){
        armMotor.set(0);
    }

    public void moveDown(){
        armMotor.set(-0.02);
    }

    public void moveToPos(Arm.Position desiredPosition){
        Rotation2d angle = Rotation2d.fromDegrees(180);
        armMotorController.setReference(22, CANSparkMax.ControlType.kPosition);
        SmartDashboard.putNumber("Arm Motor Encoder", armMotor.getEncoder().getPosition());
    }
    public void highGoal(){
        
    }

    public void midGoal(){

    }

    public void lowestPoint(){

    }

    public void humanPlayerStation(){

    }

    public void enableLimitSwitches(){
        armBottomLimitSwitch.enableLimitSwitch(true);
        armTopLimitSwitch.enableLimitSwitch(true);
    }

    public void disableLimitSwitches(){
        armBottomLimitSwitch.enableLimitSwitch(false);
        armTopLimitSwitch.enableLimitSwitch(false);
    }

    public double getEncoder(){
        return armMotor.getEncoder().getPosition();
    }
}
