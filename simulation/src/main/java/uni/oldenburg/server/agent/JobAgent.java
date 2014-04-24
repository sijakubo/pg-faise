package uni.oldenburg.server.agent;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.apache.log4j.Level;
import org.apache.log4j.Logger;

import uni.oldenburg.Debugging;
import uni.oldenburg.server.agent.behaviour01.CyclicReceiverBehaviour;
import uni.oldenburg.server.agent.behaviour01.TimeoutReceiverBehaviour;
import uni.oldenburg.server.agent.data.PackageData;
import uni.oldenburg.server.agent.helper.AgentHelper;
import uni.oldenburg.server.agent.message.MessageType;
import uni.oldenburg.shared.model.ConveyorRamp;
import uni.oldenburg.shared.model.Job;
import jade.core.AID;
import jade.core.Agent;
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
	//private PackageData currentPackage = null;
	
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
		addBehaviour(new DistributeJobBehaviour(MessageTemplate.MatchPerformative(MessageType.SEND_JOB)));
		addBehaviour(new AssignDestinationRampBehaviour(MessageTemplate.MatchPerformative(MessageType.PACKAGE_SPACE_AVAILABLE)));
		
		AgentHelper.registerAgent(szenarioID, this, JobAgent.NAME);
		
		if(Debugging.showAgentStartupMessages)
			logger.log(Level.INFO, JobAgent.NAME + " started");
	}
	
	// destructor 
	protected void takeDown() {
		AgentHelper.unregister(this);
	}
	
	/**
	 * request info about ramp types
	 * 
     * @author Matthias
     */
	private class RequestRampInfosBehaviour extends OneShotBehaviour {
		public void action() {
			// send ramp info request
			ACLMessage msg = new ACLMessage(MessageType.REQUEST_RAMP_INFO);
			AgentHelper.addReceivers(msg, myAgent, ((JobAgent)myAgent).getSzenarioID());
			
			if(Debugging.showInfoMessages)
				logger.log(Level.INFO, myAgent.getLocalName() + " -> REQUEST_RAMP_INFO");
			
			send(msg);
		}
	}
	
	/**
	 * - receive ramp info (f.e. incoming/outgoing/storage)
	 * - add a basic job to test job behaviour functionality
	 * 
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

		public void onTimeout() throws IOException {
			if(Debugging.showInfoMessages) {
				logger.log(Level.INFO, "Incoming Ramp Count: " + ((JobAgent)myAgent).getRampListIncoming().size());
				logger.log(Level.INFO, "Outgoing Ramp Count: " + ((JobAgent)myAgent).getRampListOutgoing().size());	
			}
			
			// store relevant job data in here
			PackageData currentPackage = new PackageData(1, 0);
			
			ACLMessage msg = new ACLMessage(MessageType.SEND_JOB);
			msg.addReceiver(myAgent.getAID());
			
			msg.setContentObject(currentPackage);
			
			if(Debugging.showInfoMessages)
				logger.log(Level.INFO, myAgent.getLocalName() + " -> SEND_JOB");		
			
			send(msg);
		}
	}
	
	/**
	 * - get incoming job data
	 * - ask incoming or outgoing ramps for available space
	 * 
     * @author Matthias
     */
	private class DistributeJobBehaviour extends CyclicReceiverBehaviour {
		protected DistributeJobBehaviour(MessageTemplate mt) {
			super(mt);
		}

		public void onMessage(ACLMessage msg) throws UnreadableException, IOException {
			if(Debugging.showInfoMessages)
				logger.log(Level.INFO, myAgent.getLocalName() + " <- SEND_JOB");
			
			PackageData currentPackage = (PackageData)msg.getContentObject();
			
			// send ramp space request
			ACLMessage msgInfo = new ACLMessage(MessageType.PACKAGE_SPACE_AVAILABLE);
			msgInfo.setContentObject(currentPackage);
			
			switch(currentPackage.getDestinationID()) {
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
	}
	
	/**
	 * - get info about ramps with available space
	 * - wait till everyone answered
	 * - reserve space on a random free one
	 * 
     * @author Matthias
     */
	private class AssignDestinationRampBehaviour extends CyclicReceiverBehaviour {
		int rampsResponded = 0;
		int rampCount = 0;
		List<AID> selectedList = null;
		List<AID> lstRampsWithSpace = new ArrayList<AID>();
		
		PackageData pendingPackage = null;
		
		protected AssignDestinationRampBehaviour(MessageTemplate mt) {
			super(mt);
		}

		public void onMessage(ACLMessage msg) throws UnreadableException, IOException {
			if (lstRampIncoming.contains(msg.getSender())) {
				rampCount = lstRampIncoming.size();	
				selectedList = lstRampIncoming;
			}
			else {
				rampCount = lstRampOutgoing.size();
				selectedList = lstRampOutgoing;
			}
			
			++rampsResponded;
			
			pendingPackage = (PackageData)msg.getContentObject();
			
			// remember those who have space
			if (msg.getUserDefinedParameter("space_available") == "1") {
				lstRampsWithSpace.add(msg.getSender());
			}
			
			if(Debugging.showInfoMessages)
				logger.log(Level.INFO, "Ramps responsed: " + rampsResponded + "/" + rampCount);
			
			// pick one of the available ramps with free space
			if (rampsResponded == rampCount) {
				if(Debugging.showInfoMessages)
					logger.log(Level.INFO, "Available Ramp Count: " + lstRampsWithSpace.size());
				
				int randomRamp = ((int)(Math.random() * 1000)) % lstRampsWithSpace.size();
				
				ACLMessage msgReply = new ACLMessage(MessageType.RESERVE_SPACE);
				
				String target = lstRampsWithSpace.get(randomRamp).toString();
				msgReply.addUserDefinedParameter("RampAID", target);
				msgReply.setContentObject(pendingPackage);
				
				AgentHelper.addReceivers(msgReply, selectedList);
				
				if(Debugging.showInfoMessages)
					logger.log(Level.INFO, myAgent.getLocalName() + " -> RESERVE_SPACE: " + target);
				
				send(msgReply);
			}
		}
	}
}