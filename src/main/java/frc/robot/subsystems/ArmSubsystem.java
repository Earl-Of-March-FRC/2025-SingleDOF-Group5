// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

  private final WPI_TalonSRX motor = new WPI_TalonSRX(Constants.ArmConstants.motorPort);
  private final TalonSRXSimCollection motorSim = motor.getSimCollection();

  private final PIDController posController = new PIDController(
      Constants.ArmConstants.poskP,
      Constants.ArmConstants.poskI, 
      Constants.ArmConstants.poskD);

  private final PIDController velController = new PIDController(
     Constants.ArmConstants.velkP,
     Constants.ArmConstants.velkI, 
     Constants.ArmConstants.velkD);

  /** Creates a new Arm. */
  public ArmSubsystem() {
    SmartDashboard.putNumber("Arm Pos P", posController.getP());
    SmartDashboard.putNumber("Arm Pos I", posController.getI());
    SmartDashboard.putNumber("Arm Pos D", posController.getD());

    SmartDashboard.putNumber("Arm Vel P", velController.getP());
    SmartDashboard.putNumber("Arm Vel I", velController.getI());
    SmartDashboard.putNumber("Arm Vel D", velController.getD());

    SmartDashboard.putNumber("Vel PID Negation", 1);
    

    posController.enableContinuousInput(0, 360);
    SmartDashboard.putNumber("Arm angle", 0);
    motor.setSelectedSensorPosition(0);
    motor.setNeutralMode(NeutralMode.Brake);
    motor.configFactoryDefault();
    // encoder.setDistancePerPulse(Constants.EncoderConstants.distancePerPulse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm velocity", getEncoderVelocity());
    SmartDashboard.putNumber("Arm RPM", getEncoderRPM());
    SmartDashboard.putNumber("Arm angle", getEncoderAngle());
    
    //this code is only for tuning
    posController.setP(SmartDashboard.getNumber("Arm Pos P", Constants.ArmConstants.poskP));
    posController.setI(SmartDashboard.getNumber("Arm Pos I", Constants.ArmConstants.poskI));
    posController.setD(SmartDashboard.getNumber("Arm Pos D", Constants.ArmConstants.poskD));
    
    velController.setP(SmartDashboard.getNumber("Arm Vel P", Constants.ArmConstants.velkP));
    velController.setI(SmartDashboard.getNumber("Arm Vel I", Constants.ArmConstants.velkI));
    velController.setD(SmartDashboard.getNumber("Arm Vel D", Constants.ArmConstants.velkD));
    
  }

  public void setSpeedPercent(double speed) {
    motor.set(MathUtil.clamp(speed, -1, 1));
  }

  public void setSpeedRPM(double RPM) {
    motor.set(TalonSRXControlMode.Velocity, RPM / 600 * Constants.EncoderConstants.ticksPerRev);
  }

  public void setVoltage(double voltage){
    motor.setVoltage(MathUtil.clamp(voltage, -12, 12));
  }

  public void rotateToAngle(double angle) {
    motor.set(TalonSRXControlMode.Position, angle / 360 * Constants.EncoderConstants.ticksPerRev);
  }

  public void rotateByAngle(double angle) {
    motor.set(TalonSRXControlMode.Position,
            //clamp between the physical min and max value allowed for the arm
            MathUtil.clamp(getEncoderAngle() + angle, Constants.EncoderConstants.minAngle, Constants.EncoderConstants.maxAngle)
            //convert angle to ticks
            / 360 * Constants.EncoderConstants.ticksPerRev);
  }

  public double getEncoderDistance() {
    return motor.getSelectedSensorPosition();
  }

  public double getEncoderVelocity() {
    return motor.getSelectedSensorVelocity();
  }

  public double getEncoderRPM() {
    return getEncoderVelocity() * 600 / Constants.EncoderConstants.ticksPerRev;
  }

  public double getEncoderAngle() {
    double angleWithSign = (getEncoderDistance() * 360 / Constants.EncoderConstants.ticksPerRev) % 360;
    return angleWithSign > 0? angleWithSign:360+angleWithSign;
  }

  
  public PIDController getPosController(){
    return posController;
  }
  
  public PIDController getVelController(){
    return velController;
  }
  
  @Override
  public void simulationPeriodic() {
        //Simulate enq  xszazcoder behavior
        double motorOutput = motorSim.getMotorOutputLeadVoltage() / RobotController.getBatteryVoltage();
        double simulatedTicksPer100ms = motorOutput * Constants.EncoderConstants.ticksPerRev;

        //Update the simulated velocity and position
        motorSim.setQuadratureVelocity((int) simulatedTicksPer100ms);
        motorSim.addQuadraturePosition((int) simulatedTicksPer100ms);
  }

}