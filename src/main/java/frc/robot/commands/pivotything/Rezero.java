package frc.robot.commands.pivotything;

import frc.robot.subsystems.PivotyThing;
import frc.robot.testingdashboard.Command;

public class Rezero extends Command{

    private PivotyThing m_PivotyThing = null;

    public Rezero(){
        super(PivotyThing.getInstance(), "PivotyThing", "Rezero");
        m_PivotyThing = PivotyThing.getInstance();
    }
    @Override
    public void initialize(){
        m_PivotyThing.reZero();
    }
    @Override
    public boolean isFinished(){
        return m_PivotyThing.lowLimitHit();
    }
    @Override
    public void end(boolean interrupted){
        m_PivotyThing.enableClosedLoop();
    }
    
}
