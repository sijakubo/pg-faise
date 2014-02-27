package uni.oldenburg.client.service;

import com.google.gwt.user.client.rpc.AsyncCallback;
import uni.oldenburg.shared.model.SimulationUser;

public interface RegistrationServiceAsync {
    void registerUser(SimulationUser newUser, AsyncCallback<Boolean> async);
}
