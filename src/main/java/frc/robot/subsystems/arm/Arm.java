// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.lib.util.TunableNumber;

public class Arm implements AutoCloseable {
    // The P gain for the PID controller that drives this arm.
    private TunableNumber armKp = new TunableNumber("arm kP", 50.0);
    private TunableNumber armSetpointDegrees = new TunableNumber("arm goal", 75.0);

    // The arm gearbox represents a gearbox containing two Vex 775pro motors.
    private final DCMotor armGearbox = DCMotor.getVex775Pro(2);

    // Standard classes for controlling our arm
    private final ProfiledPIDController controller = new ProfiledPIDController(armKp.get(), 0, 0, new TrapezoidProfile.Constraints(20, 20));

    private final Encoder encoder = new Encoder(Constants.kEncoderAChannel, Constants.kEncoderBChannel);
    private final PWMSparkMax motor = new PWMSparkMax(Constants.kMotorPort);

    // Simulation classes help us simulate what's going on, including gravity.
    // This arm sim represents an arm that can travel from -75 degrees (rotated down front)
    // to 255 degrees (rotated down in the back).
    private final SingleJointedArmSim armSim =
            new SingleJointedArmSim(
                    armGearbox,
                    Constants.kArmReduction,
                    SingleJointedArmSim.estimateMOI(Constants.kArmLength, Constants.kArmMass),
                    Constants.kArmLength,
                    Constants.kMinAngleRads,
                    Constants.kMaxAngleRads,
                    true,
                    0,
                    VecBuilder.fill(0.003)
            );

    private final EncoderSim encoderSim = new EncoderSim(encoder);

    private final Mechanism2d mechanism2d = new Mechanism2d(60, 60);
    private final MechanismRoot2d armPivotPoint = mechanism2d.getRoot("ArmPivot", 30, 30);

    private final MechanismLigament2d bottomToPivot =
            armPivotPoint.append(new MechanismLigament2d("ArmTower", 30, -90));

    private final MechanismLigament2d pivotToArmEnd =
            armPivotPoint.append(
                    new MechanismLigament2d(
                            "Arm",
                            30,
                            Units.radiansToDegrees(armSim.getAngleRads()),
                            6,
                            new Color8Bit(Color.kYellow)));

    /**
     * Subsystem constructor.
     */
    public Arm() {
        encoder.setDistancePerPulse(Constants.kArmEncoderDistPerPulse);

        // Put Mechanism 2d to SmartDashboard
        SmartDashboard.putData("Arm Sim", mechanism2d);
        bottomToPivot.setColor(new Color8Bit(Color.kBlue));
    }

    /**
     * Update the simulation model.
     */
    public void simulationPeriodic() {
        controller.setP(armKp.get());

        if(armSetpointDegrees.hasChanged(hashCode())) {
            controller.reset(encoder.getDistance());
        }

        // In this method, we update our simulation of what our arm is doing
        // First, we set our "inputs" (voltages)
        armSim.setInput(motor.get() * RobotController.getBatteryVoltage());

        // Next, we update it. The standard loop time is 20ms.
        armSim.update(0.020);

        //todo: Log and see graphs.

        // Finally, we set our simulated encoder's readings and simulated battery voltage
        encoderSim.setDistance(armSim.getAngleRads());
        // SimBattery estimates loaded battery voltages
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));

        // Update the Mechanism Arm angle based on the simulated arm angle
        pivotToArmEnd.setAngle(Units.radiansToDegrees(armSim.getAngleRads()));

    }

    /**
     * Run the control loop to reach and maintain the setpoint
     */
    public void reachSetpoint() {
        var pidOutput = controller.calculate(encoder.getDistance(), Units.degreesToRadians(armSetpointDegrees.get()));

        motor.setVoltage(pidOutput);
    }

    public void stop() {
        motor.set(0.0);
    }

    @Override
    public void close() {
        motor.close();
        encoder.close();
        mechanism2d.close();
        armPivotPoint.close();
        pivotToArmEnd.close();
    }
}