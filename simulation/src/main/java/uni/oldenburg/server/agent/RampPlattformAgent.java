package uni.oldenburg.server.agent;

import org.apache.log4j.Level;
import org.apache.log4j.Logger;

import uni.oldenburg.server.agent.helper.AgentHelper;
import uni.oldenburg.server.agent.message.MessageType;
import uni.oldenburg.shared.model.Conveyor;
import uni.oldenburg.shared.model.ConveyorRamp;
import jade.core.Agent;
import jade.core.behaviours.OneShotBehaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;

@SuppressWarnings("serial")
public class RampPlattformAgent extends Agent {
	public final static String NAME = "RampPlattformAgent";
	
	private int conveyorID = 0;
	private int szenarioID = 0;
	private int rampType = 0;
	
	private Logger logger = Logger.getLogger(RampPlattformAgent.class);
	
	/**
     * @author Matthias
     */	
	// init
	protected void setup() {
		Object[] args = getArguments();
		if (args != null) {
			szenarioID = (Integer) args[0];
			
			Conveyor myConveyor = (Conveyor) args[1];
			conveyorID = myConveyor.getID();
			rampType = ((ConveyorRamp)myConveyor).getRampType();
		}
		
		addBehaviour(new SendRampInfoBehaviour());
		
		String nickname = AgentHelper.getUniqueNickname(RampRoutingAgent.NAME, conveyorID, szenarioID);
		AgentHelper.registerAgent(szenarioID, this, nickname);
	}
	
	// destructor 
	protected void takeDown() {
		AgentHelper.unregister(this);
	}
	
	private class SendRampInfoBehaviour extends OneShotBehaviour {
		public void action() {
			RampPlattformAgent currentAgent = (RampPlattformAgent)myAgent;
			
			// get message
			MessageTemplate mt = MessageTemplate.MatchPerformative(MessageType.REQUEST_RAMP_INFO);
			ACLMessage msg = myAgent.blockingReceive(mt);
			logger.log(Level.INFO, myAgent.getLocalName() + " <- REQUEST_RAMP_INFO");
			
			// send message
			ACLMessage msgReply = new ACLMessage(MessageType.RETRIEVE_RAMP_INFO);
			msgReply.setContent("" + currentAgent.rampType);
			msgReply.addReceiver(msg.getSender());
			logger.log(Level.INFO, "Sender: " + msg.getSender());
			
			logger.log(Level.INFO, myAgent.getLocalName() + " -> REQUEST_RAMP_INFO");
			send(msgReply);
		}
	}
}
