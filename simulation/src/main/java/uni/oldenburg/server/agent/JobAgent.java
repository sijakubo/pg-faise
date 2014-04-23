package uni.oldenburg.server.agent;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.apache.log4j.Level;
import org.apache.log4j.Logger;

import uni.oldenburg.Debugging;
import uni.oldenburg.server.agent.Behaviour.TimeoutReceiverBehaviour;
import uni.oldenburg.server.agent.helper.AgentHelper;
import uni.oldenburg.server.agent.message.MessageType;
import uni.oldenburg.shared.model.ConveyorRamp;
import uni.oldenburg.shared.model.Job;
import jade.core.AID;
import jade.core.Agent;
import jade.core.behaviours.CyclicBehaviour;
import jade.core.behaviours.OneShotBehaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;

@SuppressWarnings("serial")
public class JobAgent extends Agent {
	public final static String NAME = "JobAgent";
	
	private int szenarioID = 0;
	
	private Logger logger = Logger.getLogger(JobAgent.class);
	
	private List<AID> lstRampIncoming = new ArrayList<AID>();
	private List<AID> lstRampOutgoing = new ArrayList<AID>();
	
	// to test job distribution	
	private Job currentJob = null;
	
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
		addBehaviour(new ReceiveRampInfosBehaviour(this, 1000, MessageTemplate.MatchPerformative(MessageType.SEND_RAMP_INFO)));
		addBehaviour(new DistributeJobBehaviour());
		addBehaviour(new AssignDestinationRampBehaviour());
		
		AgentHelper.registerAgent(szenarioID, this, JobAgent.NAME);
		
		if(Debugging.showAgentStartupMessages)
			logger.log(Level.INFO, JobAgent.NAME + " started");
	}
	
	// destructor 
	protected void takeDown() {
		AgentHelper.unregister(this);
	}
	
	/**
     * @author Matthias
     */
	private class RequestRampInfosBehaviour extends OneShotBehaviour {
		public void action() {
			// send ramp info request
			ACLMessage msg = new ACLMessage(MessageType.REQUEST_RAMP_INFO);
			AgentHelper.addReceivers(((JobAgent)myAgent).getSzenarioID(), myAgent, msg);
			
			if(Debugging.showInfoMessages)
				logger.log(Level.INFO, myAgent.getLocalName() + " -> REQUEST_RAMP_INFO");
			
			send(msg);
		}
	}
	
	/**
     * @author Matthias
     */
	private class ReceiveRampInfosBehaviour extends TimeoutReceiverBehaviour {
		public ReceiveRampInfosBehaviour(Agent myAgent, int timeoutMS, MessageTemplate mt) {
			super(myAgent, timeoutMS, mt);
		}

		public void onMessage(ACLMessage msg) {
			JobAgent currentAgent = (JobAgent)myAgent;
			
			// get ramp infos
			if(Debugging.showInfoMessages)
				logger.log(Level.INFO, myAgent.getLocalName() + " <- SEND_RAMP_INFO");
			
			AID senderAID = msg.getSender();
			int rampType = Integer.parseInt(msg.getUserDefinedParameter("rampType"));
			
			if (rampType == ConveyorRamp.RAMP_ENTRANCE) {
				currentAgent.getRampListIncoming().add(senderAID);	
			}
			else if (rampType == ConveyorRamp.RAMP_EXIT) {
				currentAgent.getRampListOutgoing().add(senderAID);	
			}
		}

		public void onTimeout() {
			if(Debugging.showInfoMessages) {
				logger.log(Level.INFO, "Incoming Ramp Count: " + ((JobAgent)myAgent).getRampListIncoming().size());
				logger.log(Level.INFO, "Outgoing Ramp Count: " + ((JobAgent)myAgent).getRampListOutgoing().size());	
			}
			
			// retrieve job
			Job myJob = new Job(100, 0);
			
			ACLMessage msg = new ACLMessage(MessageType.SEND_JOB);
			msg.addReceiver(myAgent.getAID());
			
			try {
				msg.setContentObject(myJob);
			} catch (IOException e) {
				e.printStackTrace();
			}
			
			if(Debugging.showInfoMessages)
				logger.log(Level.INFO, myAgent.getLocalName() + " -> SEND_JOB");		
			
			send(msg);
		}
	}
	
	/**
     * @author Matthias
     */
	private class DistributeJobBehaviour extends CyclicBehaviour {
		public void action() {
			// get job
			MessageTemplate mt = MessageTemplate.MatchPerformative(MessageType.SEND_JOB);
			ACLMessage msg = myAgent.receive(mt);
			
			if (msg != null) {
				if(Debugging.showInfoMessages)
					logger.log(Level.INFO, myAgent.getLocalName() + " <- SEND_JOB");
				
				try {
					currentJob = (Job)msg.getContentObject();
				} catch (UnreadableException e) {
					e.printStackTrace();
				}
				
				// send ramp space request
				ACLMessage msgInfo = new ACLMessage(MessageType.PACKAGE_SPACE_AVAILABLE);
				
				switch(currentJob.getType()) {
					case Job.INCOMING:
						AgentHelper.addReceivers(msgInfo, lstRampIncoming);
						break;
					case Job.OUTGOING:
						AgentHelper.addReceivers(msgInfo, lstRampOutgoing);
						break;
				}
				
				if(Debugging.showInfoMessages)
					logger.log(Level.INFO, myAgent.getLocalName() + " -> PACKAGE_SPACE_AVAILABLE");
				
				send(msgInfo);	
			}
			else {
				block();
			}
		}
	}
	
	/**
     * @author Matthias
     */
	private class AssignDestinationRampBehaviour extends CyclicBehaviour {
		int rampsResponded = 0;
		int rampCount = 0;
		
		List<AID> lstRampsWithSpace = new ArrayList<AID>();
		
		public void action() {
			MessageTemplate mt = MessageTemplate.MatchPerformative(MessageType.PACKAGE_SPACE_AVAILABLE);
			ACLMessage msg = myAgent.receive(mt);
			
			if (msg != null) {
				if (lstRampIncoming.contains(msg.getSender()))
					rampCount = lstRampIncoming.size();
				else
					rampCount = lstRampOutgoing.size();
				
				++rampsResponded;
				
				if (msg.getUserDefinedParameter("space_available") == "1") {
					lstRampsWithSpace.add(msg.getSender());
				}
				
				if(Debugging.showInfoMessages)
					logger.log(Level.INFO, "Ramps responsed: " + rampsResponded + "/" + rampCount);
				
				if (rampsResponded == rampCount) {
					if(Debugging.showInfoMessages)
						logger.log(Level.INFO, "Available Ramp Count: " + lstRampsWithSpace.size());
					
					
				}
			}
			else {
				block();
			}
		}
	}
}