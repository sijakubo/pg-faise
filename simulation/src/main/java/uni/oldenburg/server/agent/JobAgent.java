package uni.oldenburg.server.agent;

import org.apache.log4j.Level;
import org.apache.log4j.Logger;

import uni.oldenburg.server.agent.helper.AgentHelper;
import jade.core.Agent;
import jade.core.behaviours.Behaviour;

@SuppressWarnings("serial")
public class JobAgent extends Agent {
	public final static String NAME = "JobAgent";
	
	private int szenarioID = 0;
	
	private Logger logger = Logger.getLogger(JobAgent.class);
	
	/**
     * @author Matthias
     */
	// init
	protected void setup() {
		Object[] args = getArguments();
		if (args != null) {
			szenarioID = (Integer) args[0];
		}
		
		addBehaviour(new SendJobBehaviour());		
		
		AgentHelper.registerAgent(szenarioID, this, JobAgent.NAME);
		
		logger.log(Level.INFO, JobAgent.NAME + " started");	
	}
	
	// destructor 
	protected void takeDown() {
		AgentHelper.unregister(this);
	}
	
	private class SendJobBehaviour extends Behaviour {
		public void action() {
			
		}

		public boolean done() {
			return false;
		}
	}
}
