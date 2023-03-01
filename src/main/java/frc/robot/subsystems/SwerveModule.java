package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.config.SwerveModuleConstants;
import frc.lib.math.OnboardModuleState;
import frc.lib.util.CANCoderUtil;
import frc.lib.util.CANCoderUtil.CCUsage;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.Robot;

public class SwerveModule {
  public int moduleNumber;
  private Rotation2d lastAngle;
  private Rotation2d angleOffset;

  private CANSparkMax angleMotor;
  private CANSparkMax driveMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder integratedAngleEncoder;
  private CANCoder angleEncoder;

  private final SparkMaxPIDController driveController;
  private final SparkMaxPIDController angleController;

  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          Constants.Swerve.DRIVE_KS, Constants.Swerve.DRIVE_KV, Constants.Swerve.DRIVE_KA);

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    angleOffset = moduleConstants.angleOffset;

    /* Angle Encoder Config */
    angleEncoder = new CANCoder(moduleConstants.cancoderID);
    configAngleEncoder();

    /* Angle Motor Config */
    angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
    integratedAngleEncoder = angleMotor.getEncoder();
    angleController = angleMotor.getPIDController();
    configAngleMotor();

    /* Drive Motor Config */
    driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    driveController = driveMotor.getPIDController();
    configDriveMotor();

    lastAngle = getState().angle;
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    // Custom optimize command, since default WPILib optimize assumes continuous controller which
    // REV and CTRE are not
    desiredState = OnboardModuleState.optimize(desiredState, getState().angle);

    setAngle(desiredState);
    setSpeed(desiredState, isOpenLoop);
  }

  private void resetToAbsolute() {
    double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
    integratedAngleEncoder.setPosition(absolutePosition);
    lastAngle = Rotation2d.fromDegrees(absolutePosition);
    System.out.println("resetToAbsolute called: " + absolutePosition);
  }

  public void resetRelativeEncoders() {
    //integratedAngleEncoder.setPosition(0.0);
    //driveEncoder.setPosition(0.0);
    //lastAngle = getState().angle;
    resetToAbsolute();
  }

  public void setWheelToForward() {
    resetToAbsolute();

  }

  private void configAngleEncoder() {
    angleEncoder.configFactoryDefault();
    CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kMinimal);
    angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
  }

  private void configAngleMotor() {
    angleMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
    angleMotor.setSmartCurrentLimit(Constants.Swerve.ANGLE_CONTINUOUS_CURRENT_LIMIT);
    angleMotor.setInverted(Constants.Swerve.ANGLE_INVERT);
    angleMotor.setIdleMode(Constants.Swerve.ANGLE_NEUTRAL_MODE);
    integratedAngleEncoder.setPositionConversionFactor(Constants.Swerve.ANGLE_CONVERSION_FACTOR);
    angleController.setP(Constants.Swerve.ANGLE_KP);
    angleController.setI(Constants.Swerve.ANGLE_KI);
    angleController.setD(Constants.Swerve.ANGLE_KD);
    angleController.setFF(Constants.Swerve.ANGLE_KFF);
    angleMotor.enableVoltageCompensation(Constants.Swerve.VOLTAGE_COMP);
    angleMotor.burnFlash();
    resetToAbsolute();
  }

  private void configDriveMotor() {
    driveMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
    driveMotor.setSmartCurrentLimit(Constants.Swerve.DRIVE_CONTINUOUS_CURRENT_LIMIT);
    driveMotor.setInverted(Constants.Swerve.DRIVE_INVERT);
    driveMotor.setIdleMode(Constants.Swerve.DRIVE_NEUTRAL_MODE);
    driveEncoder.setVelocityConversionFactor(Constants.Swerve.DRIVE_CONVERSION_VELOCITY_FACTOR);
    SmartDashboard.putNumber("Mod " + moduleNumber + " velocity conversion factor actual", driveEncoder.getVelocityConversionFactor());
    SmartDashboard.putNumber("Mod " + moduleNumber + " velocity conversion factor desired", Constants.Swerve.DRIVE_CONVERSION_VELOCITY_FACTOR);
    driveEncoder.setPositionConversionFactor(Constants.Swerve.DRIVE_CONVERSION_POSITION_FACTOR);
    driveController.setP(Constants.Swerve.DRIVE_KP);
    driveController.setI(Constants.Swerve.DRIVE_KI);
    driveController.setD(Constants.Swerve.DRIVE_KD);
    driveController.setFF(Constants.Swerve.DRIVE_KFF);
    driveMotor.enableVoltageCompensation(Constants.Swerve.VOLTAGE_COMP);
    driveMotor.burnFlash();
    driveEncoder.setPosition(0.0);
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.MAX_SPEED;
      SmartDashboard.putNumber("Mod " + moduleNumber + " speed", desiredState.speedMetersPerSecond);
      SmartDashboard.putNumber("Mod " + moduleNumber + " % val", percentOutput);
      driveMotor.set(percentOutput);
    } else {
      SmartDashboard.putNumber("Auto " + moduleNumber + " speed", desiredState.speedMetersPerSecond);
      driveController.setReference(
          desiredState.speedMetersPerSecond,
          ControlType.kVelocity,
          0,
          0.0); //feedforward.calculate(desiredState.speedMetersPerSecond)
    }
  }

  private void setAngle(SwerveModuleState desiredState) {
    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    Rotation2d angle =
        (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.MAX_SPEED * 0.01))
            ? lastAngle
            : desiredState.angle;

    angleController.setReference(angle.getDegrees(), ControlType.kPosition);
    lastAngle = angle;
  }

  public double getAngleOffset() {
    return angleOffset.getDegrees();
  }

  public double getDesiredAngle() {
    return lastAngle.getDegrees();
  }

  private Rotation2d getAngle() {
    return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
  }

  public Rotation2d getCanCoder() {
    return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
  }

  public SwerveModulePosition getPosition() {
    // NOTE: the driveEncoder.getPosition() needs to be in meters, so ensure conversion factor does that
    return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
  }

  public double getDriveEncoder(){
    return driveEncoder.getPosition();
  }

  public double countsPerRev(){
    return driveEncoder.getCountsPerRevolution();
  }
}
