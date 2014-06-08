package uni.oldenburg.server.agent;

import jade.core.Agent;
import org.apache.log4j.Level;
import org.apache.log4j.Logger;

import uni.oldenburg.Debugging;
import uni.oldenburg.server.agent.helper.AgentHelper;
import uni.oldenburg.shared.model.ConveyorVehicle;
import uni.oldenburg.shared.model.Szenario;

@SuppressWarnings("serial")
public class VehicleRoutingAgent extends Agent {
	public final static String NAME = "VehicleRoutingAgent";

	private ConveyorVehicle myConveyor;
	private Szenario mySzenario;
	
	private Logger logger = Logger.getLogger(VehicleRoutingAgent.class);

	/**
     * @author Matthias
     */
	// init
	protected void setup() {
		Object[] args = getArguments();
		if (args != null) {
			mySzenario = (Szenario) args[0];
			myConveyor = (ConveyorVehicle) args[1];
		}

		String nickname = AgentHelper.getUniqueNickname(VehicleRoutingAgent.NAME, myConveyor.getID(), mySzenario.getId());
		AgentHelper.registerAgent(mySzenario.getId(), this, nickname);

		if(Debugging.showAgentStartupMessages)
			logger.log(Level.INFO, nickname + " started");
	}

	// destructor
	protected void takeDown() {
		AgentHelper.unregister(this);
	}
}
