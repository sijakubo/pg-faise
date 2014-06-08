package uni.oldenburg.server.agent;

import java.io.IOException;

import org.apache.log4j.Level;
import org.apache.log4j.Logger;

import uni.oldenburg.Debugging;
import uni.oldenburg.server.agent.behaviour.CyclicReceiverBehaviour;
import uni.oldenburg.server.agent.data.PackageData;
import uni.oldenburg.server.agent.helper.AgentHelper;
import uni.oldenburg.server.agent.message.MessageType;
import uni.oldenburg.shared.model.ConveyorRamp;
import uni.oldenburg.shared.model.Job;
import jade.core.Agent;
import jade.core.behaviours.CyclicBehaviour;
import jade.core.behaviours.OneShotBehaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;
import uni.oldenburg.shared.model.Szenario;

@SuppressWarnings("serial")
public class RampPlattformAgent extends Agent {
	public final static String NAME = "RampPlattformAgent";
	
	private ConveyorRamp myConveyor;
	private Szenario mySzenario;

	private Logger logger = Logger.getLogger(RampPlattformAgent.class);
	
	/**
     * @author Matthias
     */	
	// init
	protected void setup() {
		Object[] args = getArguments();
		if (args != null) {
			mySzenario = (Szenario) args[0];
			myConveyor = (ConveyorRamp) args[1];
		}
		
		addBehaviour(new SendRampInfoBehaviour());
		addBehaviour(new IsPackageSpaceAvailableBehaviour());
		addBehaviour(new TransferPackageRelay(MessageType.TRANSFER_PACKAGE));
		
		String nickname = AgentHelper.getUniqueNickname(RampRoutingAgent.NAME, myConveyor.getID(), mySzenario.getId());
		AgentHelper.registerAgent(mySzenario.getId(), this, nickname);
		
		if(Debugging.showAgentStartupMessages)
			logger.log(Level.INFO, nickname + " started");
	}
	
	// destructor 
	protected void takeDown() {
		AgentHelper.unregister(this);
	}
	
	/**
	 * Got message:
	 * 		JobAgent::RequestRampInfosBehaviour
	 * Send message: 
	 * 		JobAgent::ReceiveRampInfosBehaviour
	 * 
	 * send information about the ramp type
	 * 
     * @author Matthias
     */
	private class SendRampInfoBehaviour extends OneShotBehaviour {	
		public void action() {
			// get ramp info request
			MessageTemplate mt = MessageTemplate.MatchPerformative(MessageType.REQUEST_RAMP_INFO);			
			ACLMessage msg = myAgent.blockingReceive(mt);
			
			if(Debugging.showJobInitMessages)
				logger.log(Level.INFO, myAgent.getLocalName() + " <- REQUEST_RAMP_INFO");
			
			// send ramp info data
			ACLMessage msgReply = new ACLMessage(MessageType.SEND_RAMP_INFO);
			msgReply.addUserDefinedParameter("rampType", "" + myConveyor.getRampType());
			msgReply.addReceiver(msg.getSender());
			
			if(Debugging.showJobInitMessages)
				logger.log(Level.INFO, myAgent.getLocalName() + " -> SEND_RAMP_INFO");
			
			send(msgReply);
		}
	}

	/**
	 * Got message:
	 * 		JobAgent::DistributeJobBehaviour
	 * 		PackageAgent::GetPackageCountBehaviour
	 * 		JobAgent::AssignDestinationRampBehaviour
	 * 			[after ramp was chosen]
	 * Send message:
	 * 		PackageAgent::GetPackageCountBehaviour
	 * 			[ask ramps for item count]
	 * 		JobAgent::AssignDestinationRampBehaviour
	 * 		PackageAgent::AddPackageBehaviour
	 * 			[only if current ramp got the job]
	 * 
	 * send message if space is available for a package on this ramp
	 * and wait for a response if a new space should be "reserved" or not
	 * 
     * @author Matthias
     */
	private class IsPackageSpaceAvailableBehaviour extends CyclicBehaviour {
		int step = 0;
		int packageCount = 0;
		
		public void action() {
			RampPlattformAgent currentAgent = (RampPlattformAgent)myAgent;

			if (step == 0) {
				// get ramp space request
				MessageTemplate mt = MessageTemplate.MatchPerformative(MessageType.PACKAGE_SPACE_AVAILABLE);
				ACLMessage msg = myAgent.receive(mt);
				
				if (msg != null) {
					if(Debugging.showJobInitMessages)
						logger.log(Level.INFO, myAgent.getLocalName() + " <- PACKAGE_SPACE_AVAILABLE");
					
					try {
						// get package count from packageagent
						// - request
						ACLMessage msgPackageReq = new ACLMessage(MessageType.GET_PACKAGE_COUNT);
						AgentHelper.addReceiver(msgPackageReq, myAgent, PackageAgent.NAME, myConveyor.getID(), mySzenario.getId());
						
						if (Debugging.showJobInitMessages)
							logger.log(Level.INFO, myAgent.getLocalName() + " -> GET_PACKAGE_COUNT");
						
						send(msgPackageReq);
						
						// - response
						mt = MessageTemplate.MatchPerformative(MessageType.GET_PACKAGE_COUNT);
						ACLMessage msgPackageRes = myAgent.blockingReceive(mt);
						
						if (Debugging.showJobInitMessages)
							logger.log(Level.INFO, myAgent.getLocalName() + " <- GET_PACKAGE_COUNT");
						
						packageCount = Integer.parseInt(msgPackageRes.getUserDefinedParameter("package_count"));
						int packageCountMax = myConveyor.getPackageCountMax();
						String isSpaceAvailable = packageCount < packageCountMax ? "1" : "0";
						
						// always space available for outgoing ramps
						if (myConveyor.getRampType() == ConveyorRamp.RAMP_EXIT)
							isSpaceAvailable = "1";
						
						// send ramp space info
						ACLMessage msgReply = new ACLMessage(MessageType.PACKAGE_SPACE_AVAILABLE);
						msgReply.addUserDefinedParameter("space_available", isSpaceAvailable);


						String enquiringRampConveyorId = msg.getUserDefinedParameter("enquiring_ramp_conveyor_id");
						if (enquiringRampConveyorId != null) {
							msgReply.addUserDefinedParameter("enquiring_ramp_conveyor_id", enquiringRampConveyorId);
						}

						//Job or PackageData
						msgReply.setContentObject(msg.getContentObject());
						msgReply.addReceiver(msg.getSender());
						
						if(Debugging.showJobInitMessages)
							logger.log(Level.INFO, myAgent.getLocalName() + " -> PACKAGE_SPACE_AVAILABLE");

						if (msg.getUserDefinedParameter("information_message_no_step") == null) {
							++step;
						}

						send(msgReply);
					} catch (UnreadableException e) {
						e.printStackTrace();
					} catch (IOException e) {
						e.printStackTrace();
					}
				}
				else {
					block();
				}
			}
			else if (step == 1) {
				MessageTemplate mt = MessageTemplate.MatchPerformative(MessageType.RESERVE_SPACE);
				ACLMessage msg = myAgent.receive(mt);
				
				if (msg != null) {
					if(Debugging.showJobInitMessages)
						logger.log(Level.INFO, myAgent.getLocalName() + " <- RESERVE_SPACE");
					
					String target = msg.getUserDefinedParameter("RampAID");
					
					// reserve/add space in ramp if target matches 
					if (target.compareTo(myAgent.getAID().toString()) == 0) {
						ACLMessage msgAddPackage = new ACLMessage(MessageType.ADD_PACKAGE);
						AgentHelper.addReceiver(msgAddPackage, currentAgent, PackageAgent.NAME, myConveyor.getID(), mySzenario.getId());
						
						++packageCount;
							
						try {
							Job assigningJob = (Job)msg.getContentObject();
							
							// create packagedata from job infos
							PackageData myPackageData = new PackageData(assigningJob.getPackageId(), assigningJob.getDestinationId());
							
							msgAddPackage.setContentObject(myPackageData);
							send(msgAddPackage);
						}
						catch (IOException e) {
							e.printStackTrace();
						}
						catch (UnreadableException e) {
							e.printStackTrace();
						}
						
						if(Debugging.showJobInitMessages)
							logger.log(Level.INFO, myAgent.getLocalName() + " - space reserved: " + packageCount + "/" + myConveyor.getPackageCountMax());
					}
					
					step = 0;
				}
				else {
					block();
				}
			}
		}
	}
	
	private class TransferPackageRelay extends CyclicReceiverBehaviour {
		protected TransferPackageRelay(int msgType) {
			super(MessageTemplate.MatchPerformative(msgType));
		}

		public void onMessage(ACLMessage msg) throws UnreadableException, IOException {
			// send transfer request to own package-agent
			ACLMessage msgTransportPackage = new ACLMessage(MessageType.TRANSFER_PACKAGE);
			msgTransportPackage.addUserDefinedParameter("dstConveyorID", msg.getUserDefinedParameter("dstConveyorID"));
			AgentHelper.addReceiver(msgTransportPackage, myAgent, PackageAgent.NAME, myConveyor.getID(), mySzenario.getId());
			send(msgTransportPackage);
			
			myAgent.blockingReceive(MessageTemplate.MatchPerformative(MessageType.TRANSFER_PACKAGE_COMPLETED));
			
			ACLMessage msgCompleted = new ACLMessage(MessageType.TRANSFER_PACKAGE_COMPLETED);
			msgCompleted.addReceiver(msg.getSender());
			send(msgCompleted);
		}
	}
}
