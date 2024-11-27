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
    SmartDashboard.getNumber("Arm Pos P", 0.005),
    SmartDashboard.getNumber("Arm Pos I", 0), 
    SmartDashboard.getNumber("Arm Pos D", 0));

  private final PIDController velController = new PIDController(
    SmartDashboard.getNumber("Arm Vel P", 0.005),
    SmartDashboard.getNumber("Arm Vel I", 0), 
    SmartDashboard.getNumber("Arm Vel D", 0));

  /** Creates a new Arm. */
  public ArmSubsystem() {
    SmartDashboard.putNumber("Arm Pos P", 0.005);
    SmartDashboard.putNumber("Arm Pos I", 0);
    SmartDashboard.putNumber("Arm Pos D", 0);

    SmartDashboard.putNumber("Arm Vel P", 0.005);
    SmartDashboard.putNumber("Arm Vel I", 0);
    SmartDashboard.putNumber("Arm Vel D", 0);
    

    posController.enableContinuousInput(0, 360);

    motor.setSelectedSensorPosition(0);
    motor.setNeutralMode(NeutralMode.Brake);
    motor.configFactoryDefault();
    // encoder.setDistancePerPulse(Constants.EncoderConstants.distancePerPulse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm velocity", getEncoderVelocity());
    SmartDashboard.putNumber("Arm RPM", getEncoderVelocity() * 600 /  Constants.EncoderConstants.ticksPerRev);
    SmartDashboard.putNumber("Arm angle", getEncoderAngle() % 360);

    SmartDashboard.putNumber("Arm Vel P", 0.005);
    SmartDashboard.putNumber("Arm Vel I", 0);
    SmartDashboard.putNumber("Arm Vel D", 0);
  
    posController.setP(SmartDashboard.getNumber("Arm Pos  P", 0.005));
    posController.setI(SmartDashboard.getNumber("Arm Pos I", 0));
    posController.setD(SmartDashboard.getNumber("Arm Pos D", 0));
    
    velController.setP(SmartDashboard.getNumber("Arm Vel P", 0.005));
    velController.setI(SmartDashboard.getNumber("Arm Vel I", 0));
    velController.setD(SmartDashboard.getNumber("Arm Vel D", 0));

  }

  public void setSpeedPercent(double speed) {
    motor.set(MathUtil.clamp(speed, -1, 1));
  }

  public void setSpeedRPM(double RPM) {
    motor.set(TalonSRXControlMode.Velocity, RPM / 600 * Constants.EncoderConstants.ticksPerRev);
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

  public double getEncoderAngle() {
    return getEncoderDistance() * 360 / Constants.EncoderConstants.ticksPerRev;
  }

  
  public PIDController getPosController(){
    return posController;
  }
  
  public PIDController getVelController(){
    return velController;
  }
  
  @Override
  public void simulationPeriodic() {
        //Simulate encoder behavior
        double motorOutput = motorSim.getMotorOutputLeadVoltage() / RobotController.getBatteryVoltage();
        double simulatedTicksPer100ms = motorOutput * Constants.EncoderConstants.ticksPerRev;

        //Update the simulated velocity and position
        motorSim.setQuadratureVelocity((int) simulatedTicksPer100ms);
        motorSim.addQuadraturePosition((int) simulatedTicksPer100ms);
  }

}