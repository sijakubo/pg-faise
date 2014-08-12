package uni.oldenburg.server.agent;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;

import jade.core.Agent;
import jade.core.behaviours.CyclicBehaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;

import org.apache.log4j.Level;
import org.apache.log4j.Logger;

import uni.oldenburg.Debugging;
import uni.oldenburg.server.agent.behaviour.CyclicReceiverBehaviour;
import uni.oldenburg.server.agent.helper.AgentHelper;
import uni.oldenburg.server.agent.message.MessageType;
import uni.oldenburg.shared.model.ConveyorRamp;
import uni.oldenburg.shared.model.Szenario;
import uni.oldenburg.shared.model.SzenarioInfo;


@SuppressWarnings("serial")
public class RampOrderAgent extends Agent {
	public final static String NAME = "RampOrderAgent";

	private ConveyorRamp myConveyor;
	private Szenario mySzenario;
	
	SzenarioInfo myInfo = null;
	
	private Logger logger = Logger.getLogger(RampOrderAgent.class);

	/**
	 * @author Matthias
	 */
	protected void setup() {
		Object[] args = getArguments();
		if (args != null) {
			mySzenario = (Szenario) args[0];
			myConveyor = (ConveyorRamp) args[1];
		}
				
		myInfo = AgentHelper.getSimulationConveyorCounts(mySzenario);
		
		addBehaviour(new EnableIncomingJobStatusRelay(MessageType.SET_PENDING_INCOMING_STATUS));
		
		// what ramp type am i?
		switch(myConveyor.getRampType()) {
			case ConveyorRamp.RAMP_ENTRANCE:
				addBehaviour(new SendEnquirePackageRequestRelay());
				break;
			case ConveyorRamp.RAMP_EXIT:
				addBehaviour(new SendEnquirePackageRequestRelay());
				addBehaviour(new GetEnquirePackageRequest(MessageTemplate.MatchPerformative(MessageType.ENQUIRE_RAMPS_WITHOUT_ENTRANCE)));
				break;
			case ConveyorRamp.RAMP_STOREAGE:
				addBehaviour(new GetEnquirePackageRequest(
						MessageTemplate.or(	MessageTemplate.MatchPerformative(MessageType.ENQUIRE_RAMPS_WITHOUT_ENTRANCE), 
											MessageTemplate.MatchPerformative(MessageType.ENQUIRE_RAMPS_STORAGE))));
				break;
		}
		
		String nickname = AgentHelper.getUniqueNickname(RampOrderAgent.NAME, myConveyor.getID(), mySzenario.getId());
		AgentHelper.registerAgent(mySzenario.getId(), this, nickname);

		if (Debugging.showAgentStartupMessages)
			logger.log(Level.INFO, nickname + " started");
	}

	// destructor
	protected void takeDown() {
		AgentHelper.unregister(this);
	}
	
	/**
	 * Got message:
	 * 		PackageAgent::SendEnquirePackageRequest
	 * 		RampOrderAgent::GetEnquirePackageRequest
	 * 		RampRoutingAgent::Auction
	 * Send message:
	 * 		RampOrderAgent::GetEnquirePackageRequest
	 * 		RampRoutingAgent::Auction
	 * 		RampOrderAgent::SetDestinationRelay	
	 * 
	 * enquires ramp(s) if they want/demand to get a package
	 * 
     * @author Matthias
     */
	private class SendEnquirePackageRequestRelay extends CyclicBehaviour {
		int step = 0;
		int rampsResponded = 0;
		int packageID = 0;
		
		long timeoutStart = new Date().getTime();
		
		List<Integer> lstConveyorRamps = new ArrayList<Integer>();
		
		public void action() {
			//logger.log(Level.INFO, "[SendEnquirePackageRequestRelay] CID: " + myConveyor.getID() + " step: " + step);
			
			if (step == 0) {
				ACLMessage msgEnquire = myAgent.receive(MessageTemplate.MatchPerformative(MessageType.ENQUIRE_RAMPS_RELAY));
				
				if (msgEnquire != null) {
					ACLMessage enquireRampsMsg = null;
					packageID = Integer.parseInt(msgEnquire.getUserDefinedParameter("packageID"));
					
					switch(myConveyor.getRampType()) {
						case ConveyorRamp.RAMP_ENTRANCE:
							enquireRampsMsg = new ACLMessage(MessageType.ENQUIRE_RAMPS_WITHOUT_ENTRANCE);
							//logger.log(Level.INFO, "[SendEnquirePackageRequestRelay] enquiring for package: " + packageID);
							break;
						case ConveyorRamp.RAMP_EXIT:
							enquireRampsMsg = new ACLMessage(MessageType.ENQUIRE_RAMPS_STORAGE);
							break;
						default:
							return;
					}
					
					enquireRampsMsg.addUserDefinedParameter("packageID", "" + packageID);
					enquireRampsMsg.addUserDefinedParameter("requestingConveyorID", "" + myConveyor.getID());
					enquireRampsMsg.addUserDefinedParameter("requestingRampType", "" + myConveyor.getRampType());
					
					AgentHelper.addReceivers(enquireRampsMsg, myAgent, mySzenario.getId());
					
					send(enquireRampsMsg);
					
					step = 1;	
					
					timeoutStart = new Date().getTime();
				}
				else {
					//logger.log(Level.INFO, "[SendEnquirePackageRequestRelay] CID: " + myConveyor.getID() + " block for now");
					
					block();
				}
			}
			else if (step == 1) {
				int maxRampToRespond = 0;
				
				switch(myConveyor.getRampType()) {
					case ConveyorRamp.RAMP_ENTRANCE:
						maxRampToRespond = myInfo.ExitRampCount + myInfo.StorageRampCount;
						break;
					case ConveyorRamp.RAMP_EXIT:
						maxRampToRespond = myInfo.StorageRampCount;
						break;
					default:
						step = 0;
						return;
				}
				
				if (rampsResponded < maxRampToRespond) {
					ACLMessage msgEnquireResponse = myAgent.receive(MessageTemplate.MatchPerformative(MessageType.ENQUIRE_RAMP_RESPONSE));
					
					if (msgEnquireResponse != null) {
						++rampsResponded;
						
						timeoutStart = new Date().getTime();
						
						//logger.log(Level.INFO, "[SendEnquirePackageRequestRelay] CID: " + myConveyor.getID() + " responded: " + rampsResponded + "/" + maxRampToRespond);
						
						if (msgEnquireResponse.getUserDefinedParameter("demand_package").equals("1")) {
							rampsResponded = maxRampToRespond;
							lstConveyorRamps.clear();
						}
						
						if (msgEnquireResponse.getUserDefinedParameter("space_available").equals("1")) {
							lstConveyorRamps.add(Integer.parseInt(msgEnquireResponse.getUserDefinedParameter("destinationID")));	
						}
					}
					else {
						if (new Date().getTime() - timeoutStart > 3000) {
							//logger.log(Level.INFO, "[SendEnquirePackageRequestRelay] CID: " + myConveyor.getID() + " step=1 -> TIMEOUT!!!");
							
							for (int id : lstConveyorRamps) {
								logger.log(Level.INFO, "Conveyor " + id + " answered!");
							}
							
							rampsResponded = maxRampToRespond;
						}
						
						block();
					}
				}
				else {
					//logger.log(Level.INFO, "Ramps found: " + lstConveyorRamps.size());
					
					if (lstConveyorRamps.size() > 0) {
						int randomIndex = (int)(Math.random() * 100) % lstConveyorRamps.size();
						
						int selectedConveyorID = lstConveyorRamps.get(randomIndex);
						
						setSelectedIdAndPendingStatus(myAgent, selectedConveyorID);
						
						// initialize auction
						ACLMessage msgAuctionStart = new ACLMessage(MessageType.AUCTION_START);
						if (myConveyor.getRampType() == ConveyorRamp.RAMP_ENTRANCE) {
							msgAuctionStart.addUserDefinedParameter("srcRampID", "" + myConveyor.getID());
							msgAuctionStart.addUserDefinedParameter("dstRampID", "" + selectedConveyorID);	
						}
						else {
							msgAuctionStart.addUserDefinedParameter("srcRampID", "" + selectedConveyorID);
							msgAuctionStart.addUserDefinedParameter("dstRampID", "" + myConveyor.getID());
						}
						AgentHelper.addReceiver(msgAuctionStart, myAgent, RampRoutingAgent.NAME, myConveyor.getID(), mySzenario.getId());
						send(msgAuctionStart);
						
						// wait till auction is finished
						myAgent.blockingReceive(MessageTemplate.MatchPerformative(MessageType.AUCTION_END));
					}
					else {
						//if (myConveyor.getRampType() == ConveyorRamp.RAMP_ENTRANCE)
							//logger.log(Level.INFO, "[SendEnquirePackageRequestRelay] CID: " + myConveyor.getID() + " - no free ramps found!");
						
						//if (myConveyor.getRampType() == ConveyorRamp.RAMP_EXIT)
							//logger.log(Level.INFO, "[SendEnquirePackageRequestRelay] CID: " + myConveyor.getID() + " - no package found in storage");
					}
					
					step = 0;
					rampsResponded = 0;
					lstConveyorRamps.clear();					
				}
			}	
		}
	}
	
	void setSelectedIdAndPendingStatus(Agent myAgent, int selectedConveyorID) {
		ACLMessage msgSetDST = new ACLMessage(MessageType.SET_DESTINATION);
		
		//logger.log(Level.INFO, "Selected ConveyorID: " + selectedConveyorID);
		
		if (myConveyor.getRampType() == ConveyorRamp.RAMP_ENTRANCE) {
			msgSetDST.addUserDefinedParameter("destinationID", "" + selectedConveyorID);
			AgentHelper.addReceiver(msgSetDST, myAgent, PackageAgent.NAME, myConveyor.getID(), mySzenario.getId());
			
			sendIncomingJobFlag(myAgent, selectedConveyorID);
			
		}
		else if (myConveyor.getRampType() == ConveyorRamp.RAMP_EXIT) {
			msgSetDST.addUserDefinedParameter("destinationID", "" + myConveyor.getID());
			AgentHelper.addReceiver(msgSetDST, myAgent, PackageAgent.NAME, selectedConveyorID, mySzenario.getId());
			
			sendIncomingJobFlag(myAgent, myConveyor.getID());
		}
		else
			return;
			
		send(msgSetDST);	
		
		myAgent.blockingReceive(MessageTemplate.MatchPerformative(MessageType.SET_DESTINATION_COMPLETED), 2000);
		
		//logger.log(Level.INFO, "ConveyorID: " + myConveyor.getID() + " -> selCID: " + selectedConveyorID);
	}
	
	private void sendIncomingJobFlag(Agent myAgent, int receiverID) {
		logger.log(Level.INFO, "[SendIncomingJobFlag] Receiver: " + receiverID);

		ACLMessage msgSetFlag = new ACLMessage(MessageType.SET_PENDING_INCOMING_STATUS);
		AgentHelper.addReceiver(msgSetFlag, myAgent, RampOrderAgent.NAME, receiverID, mySzenario.getId());
		send(msgSetFlag);

		myAgent.blockingReceive(MessageTemplate.MatchPerformative(MessageType.SET_JOB_FLAG_COMPLETED), 2000);
		
		//logger.log(Level.INFO, "Conveyor " + receiverID + ": " + "incoming flag set");
	}
	
	private class EnableIncomingJobStatusRelay extends CyclicReceiverBehaviour {
		protected EnableIncomingJobStatusRelay(int msgType) {
			super(MessageTemplate.MatchPerformative(msgType));
		}

		public void onMessage(ACLMessage msg) throws UnreadableException, IOException {
			//logger.log(Level.INFO, "[EnableIncomingJobStatusRelay] Conveyor: " + myConveyor.getID());
			
			ACLMessage msgSetFlag = new ACLMessage(MessageType.SET_PENDING_INCOMING_STATUS);			
			AgentHelper.addReceiver(msgSetFlag, myAgent, PackageAgent.NAME, myConveyor.getID(), mySzenario.getId());
			send(msgSetFlag);
			
			myAgent.blockingReceive(MessageTemplate.MatchPerformative(MessageType.SET_JOB_FLAG_COMPLETED), 2000);
			
			ACLMessage msgAnswer = new ACLMessage(MessageType.SET_JOB_FLAG_COMPLETED);
			msgAnswer.addReceiver(msg.getSender());
			send(msgAnswer);
		}
	}
	
	/**
	 * Got message:
	 * 		RamOrderAgent::SendEnquirePackageRequestRelay
	 * 		PackageAgent::Job_Available
	 * 		PackageAgent::GetPackageCountBehaviour
	 * Send message:
	 * 		PackageAgent::Job_Available
	 * 		PackageAgent::GetPackageCountBehaviour
	 * 		PackageAgent::PackageFoundInStorage
	 * 		RampOrderAgent::SendEnquirePackageRequestRelay
	 * 
	 * determines if ramp has space to take a package
	 * and demands to get it when ramp has a job for it
	 * 
     * @author Matthias
     */
	private class GetEnquirePackageRequest extends CyclicReceiverBehaviour {
		protected GetEnquirePackageRequest(MessageTemplate mt) {
			super(mt);
		}

		@Override
		public void onMessage(ACLMessage msg) throws UnreadableException,IOException {
			int packageID = Integer.parseInt(msg.getUserDefinedParameter("packageID"));
			int requestingRampType = Integer.parseInt(msg.getUserDefinedParameter("requestingRampType"));
			boolean isSpaceAvailable = false;
			boolean doDemandPackage = false;
			
			//logger.log(Level.INFO, "[GetEnquirePackageRequest] ping!");
			
			// entrance ramp sent enquire -> send to exit and storage ramps
			if (requestingRampType == ConveyorRamp.RAMP_ENTRANCE) {		
				// request job status
				ACLMessage msgGetJobStatus = new ACLMessage(MessageType.GET_INCOMING_JOB_STATUS);
				AgentHelper.addReceiver(msgGetJobStatus, myAgent, PackageAgent.NAME, myConveyor.getID(), mySzenario.getId());
				send(msgGetJobStatus);
				
				ACLMessage msgJobStatus = myAgent.blockingReceive(MessageTemplate.MatchPerformative(MessageType.GET_INCOMING_JOB_STATUS), 1000); 
				
				boolean hasIncomingJob = true;
				
				if (msgJobStatus != null)
					hasIncomingJob = msgJobStatus.getUserDefinedParameter("status").equals("1") ? true : false;
				else
					logger.log(Level.INFO, "[GetEnquirePackageRequest] TIMEOUT: hasIncomingJob");
				
				//logger.log(Level.INFO, "Conveyor " + myConveyor.getID() + " has incoming job: " + hasIncomingJob);
				
				if (!hasIncomingJob) {
					// send to exit ramps and ask, if it has an available job
					if (myConveyor.getRampType() == ConveyorRamp.RAMP_EXIT) {	
						// ask if job is available and force demanding if so
						ACLMessage msgDemandPackage = new ACLMessage(MessageType.DEMAND_PACKAGE);
						msgDemandPackage.addUserDefinedParameter("packageID", "" + packageID);
						AgentHelper.addReceiver(msgDemandPackage, myAgent, PackageAgent.NAME, myConveyor.getID(), mySzenario.getId());
						send(msgDemandPackage);
						
						ACLMessage msgResponseDemand = myAgent.blockingReceive(MessageTemplate.MatchPerformative(MessageType.DEMAND_PACKAGE), 1000);
						
						if (msgResponseDemand != null)
							doDemandPackage = msgResponseDemand.getUserDefinedParameter("demanding").equals("1") ? true : false;
						
						// only tell that you have space, when you want something
						if (doDemandPackage)
							isSpaceAvailable = true;
					
					} // send to storage ramps and ask if they have space to take a package
					else if (myConveyor.getRampType() == ConveyorRamp.RAMP_STOREAGE) {	
						ACLMessage msgSizeRequest = new ACLMessage(MessageType.GET_PACKAGE_COUNT);
						AgentHelper.addReceiver(msgSizeRequest, myAgent, PackageAgent.NAME, myConveyor.getID(), mySzenario.getId());
						send(msgSizeRequest);
						
						ACLMessage msgSizeResponse = myAgent.blockingReceive(MessageTemplate.MatchPerformative(MessageType.GET_PACKAGE_COUNT), 1000);
						int size = myConveyor.getPackageCountMax();
						
						if (msgSizeResponse != null)
							size = Integer.parseInt(msgSizeResponse.getUserDefinedParameter("package_count"));
						else
							logger.log(Level.INFO, "[GetEnquirePackageRequest] TIMEOUT: size");
						
						isSpaceAvailable = (myConveyor.getPackageCountMax() - size) > 0 ? true : false;
					}
					else { // ignore the rest
						return;
					}	
				}
			
			} // exit ramp sent enquire -> send to storage ramps 
			else if (requestingRampType == ConveyorRamp.RAMP_EXIT) {
				// request job status
				ACLMessage msgGetJobStatus = new ACLMessage(MessageType.GET_OUTGOING_JOB_STATUS);
				AgentHelper.addReceiver(msgGetJobStatus, myAgent, PackageAgent.NAME, myConveyor.getID(), mySzenario.getId());
				send(msgGetJobStatus);
				
				ACLMessage msgJobStatus = myAgent.blockingReceive(MessageTemplate.MatchPerformative(MessageType.GET_OUTGOING_JOB_STATUS)); 
				
				boolean hasOutgoingJob = true;
				
				if (msgJobStatus != null)
					hasOutgoingJob = msgJobStatus.getUserDefinedParameter("status").equals("1") ? true : false;
				else
					logger.log(Level.INFO, "[GetEnquirePackageRequest] TIMEOUT: hasOutgoingJob");
				
				if (!hasOutgoingJob) {
					ACLMessage msgDemandPackage = new ACLMessage(MessageType.FIND_PACKAGE_IN_STORAGE);
					msgDemandPackage.addUserDefinedParameter("packageID", "" + packageID);
					AgentHelper.addReceiver(msgDemandPackage, myAgent, PackageAgent.NAME, myConveyor.getID(), mySzenario.getId());
					send(msgDemandPackage);
					
					ACLMessage msgResponseDemand = myAgent.blockingReceive(MessageTemplate.MatchPerformative(MessageType.FIND_PACKAGE_IN_STORAGE), 1000);
					
					if (msgResponseDemand != null)
						doDemandPackage = msgResponseDemand.getUserDefinedParameter("demanding").equals("1") ? true : false;
					
					if (doDemandPackage)
						isSpaceAvailable = true;
				}

			} // if someone/something else asks, ignore it
			else {
				return;
			}
			
			if (Debugging.showEnquirePackageMessages) {
				logger.log(Level.INFO, myAgent.getLocalName() + " - send enquire ["
								+ ((requestingRampType == ConveyorRamp.RAMP_ENTRANCE) ? "Entrance" : "Exit") + " -> " +
								((myConveyor.getRampType() == ConveyorRamp.RAMP_EXIT) ? "Exit" : "Storage") +
								
								"]: space: " + isSpaceAvailable + " (for entrance only) - demand package: " + doDemandPackage + " - packageID: " + packageID);	
			}
			
			ACLMessage msgEnquireResponse = new ACLMessage(MessageType.ENQUIRE_RAMP_RESPONSE);
			// only has to be processed by entrance ramps when 
			msgEnquireResponse.addUserDefinedParameter("space_available", isSpaceAvailable == true ? "1" : "0");
			// needed for entrance and exit ramps in case a package with a specific id was found
			msgEnquireResponse.addUserDefinedParameter("demand_package", doDemandPackage == true ? "1" : "0");
			//msgEnquireResponse.addUserDefinedParameter("requestingConveyorID", "" + requestingConveyorID);
			msgEnquireResponse.addUserDefinedParameter("destinationID", "" + myConveyor.getID());
			
			msgEnquireResponse.addReceiver(msg.getSender());
			send(msgEnquireResponse);
		}
	}
}
