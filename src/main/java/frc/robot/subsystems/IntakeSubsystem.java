package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer.States;

public class IntakeSubsystem extends SubsystemBase {
    CANBus rio = new CANBus("rio");
    public final TalonFX lowerIntakeMotor = new TalonFX(54, rio);
    public final TalonFX upperIntakeMotor = new TalonFX(55, rio);
    public final TalonFX storeMotor = new TalonFX(56, rio);

    private States state = States.NONE;
    private double wheelSpeed = 0.0;

    public IntakeSubsystem(){
        var slot0Configs = new Slot0Configs();
        
        slot0Configs.kP = 0;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;
        
        storeMotor.getConfigurator().apply(slot0Configs);
        storeMotor.setNeutralMode(NeutralModeValue.Brake);
    }


    // private final PositionTorqueCurrentFOC m_Request = new PositionTorqueCurrentFOC(0).withSlot(0);
    // double position = 0;

    // public Command setPos(double pos){
    //     return run(() -> {
    //         position = pos;
    //     });
    // }

    public Command intakeUp = Commands.run(
        () -> storeMotor.setControl(new VoltageOut(-1.5)))
        .withTimeout(0.5)   
        .andThen(() -> {
            storeMotor.setControl(new VoltageOut(0.0));
            storeMotor.setNeutralMode(NeutralModeValue.Brake);
        });

    public Command intakeDown = Commands.run(
        () -> {
            storeMotor.setControl(new VoltageOut(4));
            storeMotor.setNeutralMode(NeutralModeValue.Coast);
        })
        .withTimeout(0.5)
        .andThen(() -> {
            storeMotor.setControl(new VoltageOut(0.0));
            // storeMotor.setNeutralMode(NeutralModeValue.Brake);
        });

    public Command runIntake(States direction, double speed) { 
        return run(()-> {
            wheelSpeed = -1 * speed; // Set the speed when the trigger is pressed
            state = direction;
        });
    }

    @Override
    public void periodic() {
        double outputVoltage = wheelSpeed * state.getMultiplier();
        // * 0.8
        upperIntakeMotor.setControl(new VoltageOut(outputVoltage * 0.8));
        // * -1
        lowerIntakeMotor.setControl(new VoltageOut(-1 * outputVoltage));
        // storeMotor.setControl(m_Request.withPosition(position));
        state = States.NONE;
        wheelSpeed = 0;
        // intakeMotorMode = NeutralModeValue.Coast;
    }
}