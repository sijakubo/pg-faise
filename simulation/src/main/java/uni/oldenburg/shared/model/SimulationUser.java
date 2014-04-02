package uni.oldenburg.shared.model;

import java.io.Serializable;

@SuppressWarnings("serial")
/**
 * @author sijakubo
 */
public class SimulationUser implements Serializable {
    public static final String TABLE_NAME = "simulationuser";

    private Long id;
    private String email;
    private String name;
    private String password;

    public SimulationUser() {
    }

    public SimulationUser(String email, String name, String password) {
        this.email = email;
        this.name = name;
        this.password = password;
    }

    public SimulationUser(Long id, String email, String name, String password) {
        this.id = id;
        this.email = email;
        this.name = name;
        this.password = password;
    }

    public Long getId() {
        return id;
    }

    public void setId(Long id) {
        this.id = id;
    }

    public String getEmail() {
        return email;
    }

    public void setEmail(String email) {
        this.email = email;
    }

    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }

    public String getPassword() {
        return password;
    }

    public void setPassword(String password) {
        this.password = password;
    }
}
