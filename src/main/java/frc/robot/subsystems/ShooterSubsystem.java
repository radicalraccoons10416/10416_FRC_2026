package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    
    public ShooterSubsystem(){
        // set slot 0 gains
        var slot0Configs = new Slot0Configs();
        slot0Configs.kS = 0.15; // Add 0.1 V output to overcome static friction
        slot0Configs.kV = 0.14; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kP = 0.3; // An error of 1 rps results in 0.11 V output
        slot0Configs.kI = 0.4; // no output for integrated error
        slot0Configs.kD = 0.001; // no output for error derivative

        
        //Applies the configuration to the two motors then sets the second lift motor to follow
        //the voltage of the first lift motor
        shooterMotorLeft.getConfigurator().apply(slot0Configs);
        shooterMotorRight.getConfigurator().apply(slot0Configs);
        intakeMotor.getConfigurator().apply(slot0Configs);
        
        final Follower m_Follower = new Follower(52, MotorAlignmentValue.Opposed);
        
        shooterMotorLeft.setControl(m_Follower);
        shooterMotorLeft.setNeutralMode(NeutralModeValue.Coast);
        shooterMotorRight.setNeutralMode(NeutralModeValue.Coast);
        intakeMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    public final TalonFX shooterMotorLeft = new TalonFX(51, "rio");
    public final TalonFX shooterMotorRight = new TalonFX(52, "rio");
    public final TalonFX intakeMotor = new TalonFX(53, "rio");
    
    double shooterWheelSpeed = 0;
    double intakeWheelSpeed = 0;
    double defaultShooterSpeed = 20;

    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
    
    public Command runShooter(){
        return run(()-> {
            shooterWheelSpeed = defaultShooterSpeed;
            intakeWheelSpeed = 8;
        });
    }

    public void changeShooterSpeed(int change) {
        defaultShooterSpeed += change;
    }

    @Override
    public void periodic() {
        intakeMotor.setControl(new VoltageOut(intakeWheelSpeed));
        shooterMotorRight.setControl(m_request.withVelocity(-1 * shooterWheelSpeed));
        shooterWheelSpeed = 0;
        intakeWheelSpeed = 0;
    }    
}  