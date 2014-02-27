package uni.oldenburg.client.service;

import com.google.gwt.user.client.rpc.RemoteService;
import com.google.gwt.user.client.rpc.RemoteServiceRelativePath;
import uni.oldenburg.shared.model.SimulationUser;

@RemoteServiceRelativePath("registrationService")
public interface RegistrationAndLoginService extends RemoteService {
    boolean registerUser(SimulationUser newUser);
    boolean loginUser(String username, String password);
}


