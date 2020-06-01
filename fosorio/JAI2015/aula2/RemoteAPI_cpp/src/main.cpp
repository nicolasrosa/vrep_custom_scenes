/*
    Client of V-REP simulation server (remoteApi)
    Copyright (C) 2015  Rafael Alceste Berri rafaelberri@gmail.com

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// Habilite o server antes na simulação V-REP com o comando lua:
// simExtRemoteApiStart(portNumber) -- inicia servidor remoteAPI do V-REP

extern "C" {
  #include "../remoteApi/extApi.h"
}

#include <iostream>
#include <string>

using namespace std;

class Actuator{
public:
  int handle = 0;
  float vel = 0;
};

class UltraSensors{
public:
  string sensorNome[16];
  int sensorHandle[16];
  
  // Ultrasonic and Pioneer's actuation variables
  float noDetectionDist=0.5;
  float maxDetectionDist=0.2;
  float detect[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
};

class Pioneer{
  public:
    Actuator leftMotor;
    Actuator rightMotor;

    UltraSensors usensors;
  private:
    int _test = 0;
};


int main(int argc, char **argv) 
{
  // Server Configuration Variables
  string serverIP = "127.0.0.1";
  int serverPort = 19999;
  
  // Robot Object
  Pioneer pioneer;

  float braitenbergL[16]={-0.2,-0.4,-0.6,-0.8,-1,-1.2,-1.4,-1.6, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
  float braitenbergR[16]={-1.6,-1.4,-1.2,-1,-0.8,-0.6,-0.4,-0.2, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
  float v0=2;
  
  // Client Connection
  int clientID=simxStart((simxChar*)serverIP.c_str(),serverPort,true,true,2000,5);
  
  if (clientID!=-1)
  {
    cout << "Servidor conectado!" << std::endl;
    
    // Inicialização dos motores (remoteApi)
    if(simxGetObjectHandle(clientID,(const simxChar*) "Pioneer_p3dx_leftMotor",(simxInt *) &pioneer.leftMotor.handle, (simxInt) simx_opmode_oneshot_wait) != simx_return_ok)
      cout << "Handle do motor esquerdo não encontrado!" << std::endl;  
    else
      cout << "Conectado ao motor esquerdo!" << std::endl;
    
    if(simxGetObjectHandle(clientID,(const simxChar*) "Pioneer_p3dx_rightMotor",(simxInt *) &pioneer.rightMotor.handle, (simxInt) simx_opmode_oneshot_wait) != simx_return_ok)
      cout << "Handle do motor direito não encontrado!" << std::endl;  
    else
      cout << "Conectado ao motor direito!" << std::endl;
    
    // Inicialização dos sensores (remoteApi)
    for(int i = 0; i < 16; i++)
    {
      pioneer.usensors.sensorNome[i] = "Pioneer_p3dx_ultrasonicSensor" + to_string(i + 1);
      
      if(simxGetObjectHandle(clientID,(const simxChar*) pioneer.usensors.sensorNome[i].c_str(),(simxInt *) &pioneer.usensors.sensorHandle[i], (simxInt) simx_opmode_oneshot_wait) != simx_return_ok)
	      cout << "Handle do sensor " << pioneer.usensors.sensorNome[i] << " não encontrado!" << std::endl;
      else
      {
        cout << "Conectado ao sensor " << pioneer.usensors.sensorNome[i] << "!" << std::endl;
	      simxReadProximitySensor(clientID,pioneer.usensors.sensorHandle[i],NULL,NULL,NULL,NULL,simx_opmode_streaming); // Mandatory First Read
      }
    }
    
    // Desvio e velocidade do robô
    while(simxGetConnectionId(clientID)!=-1) // enquanto a simulação estiver ativa
    {
      for(int i = 0; i < 16; i++)
      {
	      simxUChar state;
	      simxFloat coord[3];
	
	    if(simxReadProximitySensor(clientID,pioneer.usensors.sensorHandle[i],&state,coord,NULL,NULL,simx_opmode_buffer)==simx_return_ok)
	    { 
	      float dist = coord[2];
	      if(state > 0 && (dist<pioneer.usensors.noDetectionDist))
	      {
	        if(dist<pioneer.usensors.maxDetectionDist)
	        {
	          dist=pioneer.usensors.maxDetectionDist;
	        }
	    
	      pioneer.usensors.detect[i]=1-((dist-pioneer.usensors.maxDetectionDist)/(pioneer.usensors.noDetectionDist-pioneer.usensors.maxDetectionDist));
	      }
      else
        pioneer.usensors.detect[i] = 0;
	    }
	    else
	      pioneer.usensors.detect[i] = 0;
      }
      
      pioneer.leftMotor.vel = v0;
      pioneer.rightMotor.vel = v0;
      
      for(int i = 0; i < 16; i++)
      {
	      pioneer.leftMotor.vel = pioneer.leftMotor.vel + braitenbergL[i]*pioneer.usensors.detect[i];
        pioneer.rightMotor.vel = pioneer.rightMotor.vel + braitenbergR[i]*pioneer.usensors.detect[i];
      }
      
      // Atualiza velocidades dos motores
      simxSetJointTargetVelocity(clientID, pioneer.leftMotor.handle, (simxFloat) pioneer.leftMotor.vel, simx_opmode_streaming);
      simxSetJointTargetVelocity(clientID, pioneer.rightMotor.handle, (simxFloat) pioneer.rightMotor.vel, simx_opmode_streaming);
      
      // Espera um pouco antes de reiniciar a leitura dos sensores
      extApi_sleepMs(5);
    }
      
    simxFinish(clientID); // fechando conexao com o servidor
    cout << "Conexão fechada!" << std::endl;
  }
  else
    cout << "Problemas para conectar o servidor!" << std::endl;
  
  cout << "Done." << endl;

  return 0;
}