package uni.oldenburg.server.agent;

import java.util.ArrayList;
import java.util.List;

import org.apache.log4j.Level;
import org.apache.log4j.Logger;

import uni.oldenburg.Debugging;
import uni.oldenburg.server.agent.Behaviour.TimeoutReceiverBehaviour;
import uni.oldenburg.server.agent.helper.AgentHelper;
import uni.oldenburg.server.agent.message.MessageType;
import uni.oldenburg.shared.model.ConveyorRamp;
import jade.core.AID;
import jade.core.Agent;
import jade.core.behaviours.OneShotBehaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;

@SuppressWarnings("serial")
public class JobAgent extends Agent {
	public final static String NAME = "JobAgent";
	
	private int szenarioID = 0;
	
	private Logger logger = Logger.getLogger(JobAgent.class);
	
	private List<AID> lstRampIncoming = new ArrayList<AID>();
	private List<AID> lstRampOutgoing = new ArrayList<AID>();
	
	public int getSzenarioID() {
		return this.szenarioID;
	}
	
	public List<AID> getRampListIncoming() {
		return lstRampIncoming;
	}

	public List<AID> getRampListOutgoing() {
		return lstRampOutgoing;
	}

	/**
     * @author Matthias
     */
	// init
	protected void setup() {
		Object[] args = getArguments();
		if (args != null) {
			szenarioID = (Integer) args[0];
		}
		
		addBehaviour(new RequestRampInfosBehaviour());
		//addBehaviour(new ReceiveRampInfosBehaviour());
		addBehaviour(new ReceiveRampInfosBehaviour(this, 10000, MessageTemplate.MatchPerformative(MessageType.RETRIEVE_RAMP_INFO)));
		
		AgentHelper.registerAgent(szenarioID, this, JobAgent.NAME);
		
		if(Debugging.showAgentStartupMessages)
			logger.log(Level.INFO, JobAgent.NAME + " started");	
	}
	
	// destructor 
	protected void takeDown() {
		AgentHelper.unregister(this);
	}
	
	/*private class ReceiveRampInfosBehaviour extends CyclicBehaviour {
		MessageTemplate mt = MessageTemplate.MatchPerformative(MessageType.RETRIEVE_RAMP_INFO);
		
		@Override
		public void action() {
			ACLMessage msg = myAgent.receive(mt);
			if (msg != null) {
				if(Debugging.showInfoMessages) {
					logger.log(Level.INFO, myAgent.getLocalName() + " <- RETRIEVE_RAMP_INFO");
					logger.log(Level.INFO, "Content: " + msg.getUserDefinedParameter("rampType"));	
				}
			}
			else {
				block();
			}
		}
		
	}*/
	
	private class RequestRampInfosBehaviour extends OneShotBehaviour {
		public void action() {
			// send message
			ACLMessage msg = new ACLMessage(MessageType.REQUEST_RAMP_INFO);
			AgentHelper.addReceivers(((JobAgent)myAgent).getSzenarioID(), myAgent, msg);
			
			if(Debugging.showInfoMessages)
				logger.log(Level.INFO, myAgent.getLocalName() + " -> REQUEST_RAMP_INFO");
			
			send(msg);
		}
	}
	
	private class ReceiveRampInfosBehaviour extends TimeoutReceiverBehaviour {
		public ReceiveRampInfosBehaviour(Agent myAgent, int timeoutMS, MessageTemplate mt) {
			super(myAgent, timeoutMS, mt);
		}

		public void onMessage(ACLMessage msg) {
			if(Debugging.showInfoMessages) {
				logger.log(Level.INFO, myAgent.getLocalName() + " <- RETRIEVE_RAMP_INFO");
				logger.log(Level.INFO, "Content: " + msg.getUserDefinedParameter("rampType"));	
			}
			
			AID senderAID = msg.getSender();
			int rampType = Integer.parseInt(msg.getUserDefinedParameter("rampType"));			
			
			if (rampType == ConveyorRamp.RAMP_ENTRANCE) {
				((JobAgent)myAgent).getRampListIncoming().add(senderAID);	
			}
			else if (rampType == ConveyorRamp.RAMP_EXIT) {
				((JobAgent)myAgent).getRampListOutgoing().add(senderAID);	
			}
		}

		public void onTimeout() {
			// get job
			// read dstID
			// use right list
			// ask ramps for available space
			// retrieve info
			// choose "best" ramp or random
			// send packageid to chosen ramp
			// delete job from joblist(?)
		}
	}
}
