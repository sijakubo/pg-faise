package uni.oldenburg.server.agent;

import org.apache.log4j.Level;
import org.apache.log4j.Logger;

import uni.oldenburg.Debugging;
import uni.oldenburg.server.agent.helper.AgentHelper;
import uni.oldenburg.shared.model.ConveyorRamp;
import jade.core.Agent;
import jade.core.behaviours.CyclicBehaviour;
import uni.oldenburg.shared.model.Szenario;

@SuppressWarnings("serial")
public class RampRoutingAgent extends Agent {
	public final static String NAME = "RampRoutingAgent";

	private ConveyorRamp myConveyor;
	private Szenario mySzenario;
	private Logger logger = Logger.getLogger(RampRoutingAgent.class);
	
	@SuppressWarnings("unused")
	private int vehicleCount = 0;

	/**
	 * @author Matthias
	 */
	// init
	protected void setup() {
		Object[] args = getArguments();
		if (args != null) {
			mySzenario = (Szenario) args[0];
			myConveyor = (ConveyorRamp) args[1];
		}		
		
		addBehaviour(new Auction());
		
		vehicleCount = AgentHelper.getSimulationConveyorCounts(mySzenario).VehicleCount;

		String nickname = AgentHelper.getUniqueNickname(RampRoutingAgent.NAME, myConveyor.getID(), mySzenario.getId());
		AgentHelper.registerAgent(mySzenario.getId(), this, nickname);

		if (Debugging.showAgentStartupMessages)
			logger.log(Level.INFO, nickname + " started");
	}

	// destructor
	protected void takeDown() {
		AgentHelper.unregister(this);
	}
	
	/**
	 * handles complete auction process
	 * 
     * @author Matthias
     */
	private class Auction extends CyclicBehaviour {
		public void action() {
			
		}
	}
}
