package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constantsGlobal.Constants;

public class Intake extends SubsystemBase{
    //private final IntakeVisualizer measuredVisualizer = new IntakeVisualizer("Measured", Color.kRed);
    //private final IntakeVisualizer setpointVisualizer = new IntakeVisualizer("Setpoint", Color.kBlue);
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();


    public boolean lowered = false;
    public boolean coralPassed = false;
    private boolean beamBreakTriggered = false;
    public static boolean simBeamBreak = false;
    public boolean stowed = true;
    private double positionSetpoint = stowedPositionDeg;


    public Intake(IntakeIO io){
        this.io = io;
    }

    public void periodic(){
        io.updateInputs(inputs);
        if(inputs.pivotPositionDeg > IntakeConstants.loweredPositionDeg){
            lowered = true;
        }
        else if(inputs.pivotPositionDeg < IntakeConstants.stowedPositionDeg){
            stowed = true;
        }
        else{
            stowed = false;
            lowered = false;
        }

        if(beamBreak() && !beamBreakTriggered){
            beamBreakTriggered = true;
        }
        else if(!beamBreak() && beamBreakTriggered){
            coralPassed = true;
        }
        else if(beamBreakTriggered && coralPassed){
            coralPassed = false;
        }
    }

    public boolean beamBreak() {
        return simBeamBreak;
    }

    public void lower(){
        positionSetpoint = loweredPositionDeg;
        io.setPivot(positionSetpoint);
    }
    public void stow(){
        positionSetpoint = stowedPositionDeg;
        io.setPivot(positionSetpoint);
    }
    private void rollersIn() {
        io.runRollers(rollersSpeedIn);
    }
    
      private void rollersOut() {
        io.runRollers(rollersSpeedOut);
      }
      
 
}
