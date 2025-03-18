package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Wrist extends SubsystemBase{
    public DoubleSolenoid solenoid;
    double pressure;
    PneumaticHub nhub;
    Compressor compressor;

    
    public Wrist() {
        solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 15);
        nhub = new PneumaticHub(1);
        compressor = new Compressor(PneumaticsModuleType.REVPH);
    }

    public Command setWristOpen() {
        return runOnce(() -> solenoid.set(DoubleSolenoid.Value.kReverse));
    }
   
    public Command setWristClose() {
        return runOnce(() -> solenoid.set(DoubleSolenoid.Value.kForward));
}
    
@Override
public void periodic() {
    //compressor.enableAnalog(80,115);
    //SmartDashboard.putNumber("Compressor psi", compressor.getPressure());
}



}