package uni.oldenburg.client.service;

import com.google.gwt.user.client.rpc.AsyncCallback;

import uni.oldenburg.shared.model.SimulationUser;

public interface RegistrationAndLoginServiceAsync extends ServiceAsync {
    void registerUser(SimulationUser newUser, AsyncCallback<Boolean> async);
    void loginUser(String username, String password, AsyncCallback<Boolean> async);
}
