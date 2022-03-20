// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// 2800
//
package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.SparkMaxPIDController;
import frc.robot.commands.Shooter.HoodPosition;

public class ShooterSubsystem extends SubsystemBase {
  public final WPI_TalonFX shooterBottomTalon = new WPI_TalonFX(41);
  public final WPI_TalonFX shooterTopTalon = new WPI_TalonFX(40);
  public final CANSparkMax hood = new CANSparkMax(34, MotorType.kBrushless); //FIXME motor ID
  public SparkMaxPIDController hoodPIDController;
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  double closePosition = 0.1; //FIXME hood angle for close shot
  double farPosition = 0.9; //FIXME hood angle for far shot
  double closeTy = 8.8; //FIXME y coordinate of target on limelight cam for close shot
  double farTy = -8.8; //FIXME y coordinate of target on limelight cam for far shot
  double currentPosition;
  double targetPosition;
  RelativeEncoder hoodEncoder = hood.getEncoder();
  double positionAllowedError = 0;
  double testSpeed = 0.5;
  double kP = 2; 
  double kI = 0;
  double kD = 0; 
  double kIz = 0; 
  double kFF = 0; 
  double kMaxOutput = 1; 
  double kMinOutput = -1;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    /*
    TalonFXConfiguration flywheelTalonConfig = new TalonFXConfiguration();
    shooterBottomTalon.configAllSettings(flywheelTalonConfig);
    shooterBottomTalon.setNeutralMode(NeutralMode.Coast);
    shooterBottomTalon.setInverted(false);
    shooterBottomTalon.configSupplyCurrentLimit(
      new SupplyCurrentLimitConfiguration(true, 30, 35, 0.5));

      shooterTopTalon.configAllSettings(flywheelTalonConfig);
      shooterTopTalon.setNeutralMode(NeutralMode.Coast);
      shooterTopTalon.setInverted(true);
      shooterTopTalon.configSupplyCurrentLimit(
        new SupplyCurrentLimitConfiguration(true, 30, 35, 0.5));

    shooterBottomTalon.configSelectedFeedbackSensor(
      TalonFXFeedbackDevice.IntegratedSensor, // Sensor Type 
      Constants.shooterPID,      // PID Index
      Constants.shooterTimeout);      // Config Timeout

    shooterBottomTalon.config_kP(Constants.SLOT_0, Constants.kBottomGains.kP, Constants.shooterTimeout);
		shooterBottomTalon.config_kI(Constants.SLOT_0, Constants.kBottomGains.kI, Constants.shooterTimeout);
		shooterBottomTalon.config_kD(Constants.SLOT_0, Constants.kBottomGains.kD, Constants.shooterTimeout);
		shooterBottomTalon.config_kF(Constants.SLOT_0, Constants.kBottomGains.kF, Constants.shooterTimeout);

    shooterTopTalon.config_kP(Constants.SLOT_0, Constants.kTopGains.kP, Constants.shooterTimeout);
		shooterTopTalon.config_kI(Constants.SLOT_0, Constants.kTopGains.kI, Constants.shooterTimeout);
		shooterTopTalon.config_kD(Constants.SLOT_0, Constants.kTopGains.kD, Constants.shooterTimeout);
		shooterTopTalon.config_kF(Constants.SLOT_0, Constants.kTopGains.kF, Constants.shooterTimeout);
    
    shooterTopTalon.setInverted(TalonFXInvertType.OpposeMaster);*/
  
  }
/*
  public void shooterRunAtVelocity(int bottomShooterVelocity, int topShooterVelocity2){
    shooterBottomTalon.set(TalonFXControlMode.Velocity, bottomShooterVelocity);
    shooterTopTalon.set(TalonFXControlMode.Velocity, topShooterVelocity2);
  }

  public double getShooterVelocity(ShooterSubsystem shooterSubsystem){
    double velocity = shooterBottomTalon.getSelectedSensorVelocity();
    shooterTopTalon.getSelectedSensorVelocity();
    return velocity;
  }

  public void shootStop(){
    shooterBottomTalon.stopMotor();
    shooterTopTalon.stopMotor();
  }
*/
  public void hoodPosition(){
    hoodPIDController.setReference(targetPosition, CANSparkMax.ControlType.kPosition);//0 to 1
  }
  public void hoodPositionReset(){ //use in initialization
    //hoodPIDController.setReference(0, CANSparkMax.ControlType.kPosition);
    hoodEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    double currentTx = tx.getDouble(0.0);
    double currentTy = ty.getDouble(0.0);
    double currentTa = ta.getDouble(0.0);
    currentPosition = hoodEncoder.getPosition(); 
    hoodPIDController = hood.getPIDController();
    hoodPIDController.setP(kP);
    hoodPIDController.setI(kI);
    hoodPIDController.setD(kD);
    hoodPIDController.setIZone(kIz);
    hoodPIDController.setFF(kFF);
    hoodPIDController.setOutputRange(kMinOutput, kMaxOutput);
     
    targetPosition = closePosition + ((currentTy - closeTy) * ((farPosition - closePosition) / (farTy - closeTy)));

    if ((currentPosition != targetPosition /*<= (targetPosition - positionAllowedError)) || (currentPosition >= (targetPosition + positionAllowedError)*/)) {
      SmartDashboard.putString("Hood Status", "Waiting for motor to reach target");
      //hood.set(testSpeed);
    }
    else {
      SmartDashboard.putString("Hood Status", "at target");
    }
    SmartDashboard.putNumber("Current Hood Position", currentPosition);
    SmartDashboard.putNumber("Target Hood Position", targetPosition);
    SmartDashboard.getNumber("Target Hood Position", targetPosition);
    SmartDashboard.putNumber("LimelightX", currentTx);
    SmartDashboard.putNumber("LimelightY", currentTy);
    SmartDashboard.putNumber("LimelightArea", currentTa);
    // This method will be called once per scheduler run
  }
}