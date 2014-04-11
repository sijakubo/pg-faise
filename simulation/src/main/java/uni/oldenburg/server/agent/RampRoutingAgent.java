package uni.oldenburg.server.agent;

import uni.oldenburg.server.agent.helper.AgentHelper;
import jade.core.Agent;
import jade.core.behaviours.Behaviour;

@SuppressWarnings("serial")
public class RampRoutingAgent extends Agent {
	// init
	protected void setup() {
		AgentHelper.registerAgent(this, this.getClass().getSimpleName());
		addBehaviour(new AssignVehicleForPackageBehaviour());
		addBehaviour(new FindVehicleBehaviour());
		addBehaviour(new ReceiveEstimationBehaviour());
	}
	
	// destructor 
	protected void takeDown() {
		AgentHelper.unregister(this);
	}

	private class AssignVehicleForPackageBehaviour extends Behaviour {

		public void action() {
			
		}

		public boolean done() {
			return false;
		}
	}

	private class FindVehicleBehaviour extends Behaviour {

		public void action() {
			
		}

		public boolean done() {
			return false;
		}
	}
	
	private class ReceiveEstimationBehaviour extends Behaviour {

		public void action() {
			
		}

		public boolean done() {
			return false;
		}
	}
}
