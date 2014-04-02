package uni.oldenburg.client.service;

import com.google.gwt.user.client.rpc.RemoteService;
import com.google.gwt.user.client.rpc.RemoteServiceRelativePath;

@RemoteServiceRelativePath("sessionInformationService")
/**
 * @author sijakubo
 */
public interface SessionInformationService extends RemoteService {
    boolean isUserLoggedInInSession();
}
