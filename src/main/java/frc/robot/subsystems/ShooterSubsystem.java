package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    public final TalonFX shooterMotorLeft = new TalonFX(51, "rio");
    public final TalonFX shooterMotorRight = new TalonFX(52, "rio");
    public final TalonFX intakeMotor = new TalonFX(53, "rio");
    double shooterWheelSpeed = 0;
    double intakeWheelSpeed = 0;

    public void runShooter() {
        shooterWheelSpeed = 3;
        intakeWheelSpeed = 5;
    }
    
    public void stopShooter() {
        shooterWheelSpeed = 0;
        intakeWheelSpeed = 0;
    }

    @Override
    public void periodic() {
        shooterMotorLeft.setControl(new VoltageOut(shooterWheelSpeed));
        shooterMotorRight.setControl(new Follower(shooterMotorLeft.getDeviceID(), MotorAlignmentValue.Opposed));
        intakeMotor.setControl(new VoltageOut(intakeWheelSpeed));
    }
}