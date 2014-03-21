package uni.oldenburg.shared.model;

import java.util.ArrayList;
import java.util.List;

public class JobList {
	
	private String name;
	private List<Job> jobList;
	
	public JobList(String name) {
		this.name = name;
		jobList = new ArrayList<Job>();
	}
	
	public void addJob(Job job) {
		int index = 0;
		//sort by timestamp
		while(jobList.get(index) != null && jobList.get(index).getTimestamp() < job.getTimestamp()) {
			index++;
		}
		jobList.add(index, job);
	}
	
	public void addRandomJobs(int numberOfJobs) {
		for(int i = 0; i < numberOfJobs; i++) {
			
			String jobName = "Job #" + i;
			int type = (Math.random() < 0.5) ? Job.INCOMING : Job.OUTGOING;
			long timestamp = (long)(Math.random() * 10000);

			addJob(new Job(i, jobName, type, timestamp));
		}
	}
	
	public String getName() {
		return name;
	}
	
	public List<Job> getJobs() {
		return jobList;
	}
}
