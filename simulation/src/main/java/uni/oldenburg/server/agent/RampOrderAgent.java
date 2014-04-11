package uni.oldenburg.server.agent;

import uni.oldenburg.server.agent.helper.AgentHelper;
import jade.core.Agent;

@SuppressWarnings("serial")
public class RampOrderAgent extends Agent {
	// init
	protected void setup() {
		AgentHelper.registerAgent(this, this.getClass().getSimpleName());
	}
	
	// destructor 
	protected void takeDown() {
		AgentHelper.unregister(this);
	}
}
