package uni.oldenburg.client.service;

import java.util.ArrayList;

import uni.oldenburg.shared.model.JobList;
import uni.oldenburg.shared.model.Szenario;
import uni.oldenburg.shared.model.SzenarioInfo;

import com.google.gwt.user.client.rpc.AsyncCallback;

public interface SimulationServiceAsync extends ServiceAsync {
	void loadSzenario(String name, AsyncCallback<Szenario> asyncCallback);
	void getScenarioInfos (AsyncCallback<ArrayList<SzenarioInfo>> asyncCallback);
	void saveSzenario(Szenario szenario, AsyncCallback<Void> asyncCallback);
	void checkIfTitleExists(String title, AsyncCallback<Boolean> asyncCallback);
	void loadJoblist(String name, AsyncCallback<JobList> asyncCallback);
	void getJoblistTitles(AsyncCallback<ArrayList<String>> asyncCallback);
	void saveJoblist(JobList list,AsyncCallback<Void> asyncCallback);
	void checkIfJobListExists(String title, AsyncCallback<Boolean> asyncCallback);
}

