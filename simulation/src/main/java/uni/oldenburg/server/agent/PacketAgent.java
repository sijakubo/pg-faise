package uni.oldenburg.server.agent;

import uni.oldenburg.server.agent.helper.AgentHelper;
import jade.core.Agent;
import jade.core.behaviours.Behaviour;

@SuppressWarnings("serial")
public class PacketAgent extends Agent {
	// init
	protected void setup() {
		AgentHelper.registerAgent(this, this.getClass().getSimpleName());
		addBehaviour(new AssignDestinationBehaviour());
		addBehaviour(new ExistsDestinationBehaviour());
		addBehaviour(new FindVehicleBehaviour());
	}
	
	// destructor 
	protected void takeDown() {
		AgentHelper.unregister(this);
	}

	private class AssignDestinationBehaviour extends Behaviour {

		public void action() {
			
		}

		public boolean done() {
			return false;
		}
	}

	private class ExistsDestinationBehaviour extends Behaviour {

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
}
