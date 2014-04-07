package uni.oldenburg.client.service;

import java.util.ArrayList;

import uni.oldenburg.shared.model.JobList;
import uni.oldenburg.shared.model.Szenario;

import com.google.gwt.user.client.rpc.AsyncCallback;

public interface SimulationServiceAsync extends ServiceAsync {
	void loadSzenario(String name, AsyncCallback<Szenario> asyncCallback);
	void getScenarioTitles (AsyncCallback<ArrayList<String>> asyncCallback);
	void saveSzenario(Szenario szenario, AsyncCallback<Void> asyncCallback);
	void checkIfTitleExists(String title, AsyncCallback<Boolean> asyncCallback);
	void getUserName( AsyncCallback<String> asyncCallback);	
	void loadJoblist(String name, AsyncCallback<JobList> asyncCallback);
	void getJoblistTitles(AsyncCallback<ArrayList<String>> asyncCallback);
	void saveJoblist(JobList list,AsyncCallback<Void> asyncCallback);
	void checkIfJobListExists(String title, AsyncCallback<Boolean> asyncCallback);
}

