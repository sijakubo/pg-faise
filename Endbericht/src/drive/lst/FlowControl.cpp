// Foerderbandsteuerung zum Aufladen von Paketen
void TankSteering::flowControl(const ros::TimerEvent& event) //Funktionsaufruf durch Timer
{

	int lightValue;
	lightValue = tankSettings.epos[flow]->getLightSensorsValue();

	int sensorBack = 0x4000; //Wert, wenn hintere Lichtschranke ausgeloest ist
	int sensorNo = 0x0000; //Wert, wenn keine Lichtschranke ausgeloest ist

	std::string loadStatus;

	if (tankSettings.flowControl == 1.0 && tankSettings.hubControl == 0.0){ //Abfrage ob Flow-Flag gesetzt und Hub an Zielposition ist
		if (lightValue == sensorBack && loadStatus != "PackageLoaded"){ // Motorstop wenn hintere Lichtschranke ausloest 

			sleep(2);
			tankSettings.targetVelocityMPS[flow] = 0.0; //Foederbandgeschwindigkeit auf Null gesetzt

			loadStatus = "PackageLoaded";
			tankSettings.flowPub.publish(loadStatus); //Sende Ladestatus an simple_navigation_goals

			tankSettings.flowControl = 0.0; //Flow-Flag zurueckgesetzt
		}
		else if (lightValue != sensorBack){ //Wenn hintere Lichtschranke nicht ausgeloest ist

			tankSettings.targetVelocityMPS[flow] = 35.0; //Setzen der Foerderbandgeschwindigkeit
		}
	}
}