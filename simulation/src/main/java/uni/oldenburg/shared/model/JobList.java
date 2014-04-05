package uni.oldenburg.shared.model;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * @author Christopher
 */

@SuppressWarnings("serial")
public class JobList implements Serializable {
	private static int idCounterPacket = 0;
	
	private String name;
	private List<Job> jobList;
	
	public JobList() {
		this.name = "undefined";
		this.jobList = new ArrayList<Job>();		
	}
	
	public JobList(String name) {
		this.name = name;
		this.jobList = new ArrayList<Job>();
	}
	
	public void addJob(Job job) {
			
		int index = 0;
		
		//sort by timestamp
		while(index < jobList.size() && jobList.get(index).getTimestamp() < job.getTimestamp()) {
			index++;
		}
		
		if(index == jobList.size()) {
			jobList.add(job);
		} else {
			jobList.add(index, job);
		}
	}
	
	public void addRandomJobs(int numberOfJobs) {
		for(int i = 0; i < numberOfJobs; i++) {
			int destinationId = (Math.random() < 0.5) ? 0 : -1;
			int type = destinationId == 0 ? Job.INCOMING : Job.OUTGOING;
			int packageId = 1;
			
			if (type == Job.INCOMING)
				packageId = ++idCounterPacket;
			else {
				packageId = (int)((Math.random() * 10000) % idCounterPacket);
				
				if (Math.random() < 0.1 || idCounterPacket < 5)
					packageId += 5+ (packageId / 10);
			}
				
			int expectedValue = 500;
			int standardDeviation = 100;
			long timestamp = (long) (expectedValue + (standardDeviation * new Random().nextGaussian())) % 1000;
			
			addJob(new Job(type, timestamp, packageId, destinationId));
		}
	}
	
	public String getName() {
		return name;
	}
	
	public List<Job> getJobList() {
		return jobList;
	}
	
	public int size() {
		return jobList.size();
	}
	
	public List<Job> subList(int start, int end) {
		return jobList.subList(start, end);
	}
	
	public void removeJob(Job job) {
		jobList.remove(job);
	}
}
