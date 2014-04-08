package uni.oldenburg.agent;

import uni.oldenburg.agent.helper.AgentHelper;
import jade.core.Agent;

@SuppressWarnings("serial")
public class VehicleRoutingAgent extends Agent {
	// init
	protected void setup() {
		AgentHelper.registerAgent(this, this.getClass().getSimpleName());
	}
	
	// destructor 
	protected void takeDown() {
		AgentHelper.unregister(this);
	}
}
