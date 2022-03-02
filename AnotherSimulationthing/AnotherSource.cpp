#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>
#include "Particle.h"
#include "RigidBody.h"


using namespace std;

int FixCenterOfMass(Particle Particles[50], int NumberOfParticles);
void FindMomentOfInertia(double Moment[3][3]);
double DotProduct(double[], double[]);
int GetNumParticles();

int main(void)
{

	Particle Particles[50];
	int Num_Particles = GetNumParticles();
	cout << Num_Particles << endl;
	double MomentOfInertia[3][3] = { 0 };
	FixCenterOfMass(Particles, Num_Particles);
	FindMomentOfInertia(MomentOfInertia);
	double T;

	RigidBody Body = RigidBody(Particles, MomentOfInertia);
	
		/*
		cout << "For how much time would you like to simulate?" << endl;
		cin >> T;

		cout << endl << "Choose your dt " << endl;
		double dt;
		cin >> dt;
		double time = 0;
		ofstream StateVectorOutput;
		ofstream ParticlePositionsOutput;
		ParticlePositionsOutput.open("ParticlePositions.txt");
		StateVectorOutput.open("SimulationOutput.txt");
		ParticlePositionsOutput << Num_Particles << endl;
		StateVectorOutput << setw(15) << "Time";
		for (int i = 0; i < 3; i++)
		{
			StateVectorOutput << setw(14) << "COM_" << i;
		}
		for (int j = 0; j < 4; j++)
		{
			StateVectorOutput << setw(14) << "QUAT_" << j;
		}
		StateVectorOutput << endl;
		while (time < T)
		{
			// Describing State Vector
			for (int i = 0; i < 3; i++)
			{
				StateVectorOutput << setw(15) << Body.GetCenterOfMass(i);
			}
			for (int j = 0; j < 4; j++)
			{
				StateVectorOutput << setw(15) << Body.GetQuaternion(j);
			}
			StateVectorOutput << endl;

			// Giving Position Of all the Particles

			int index = 0;
			Body.Timestep(dt);
			time += dt;

			int i = 0;
			while(!Body.GetParticleMass(i)==0)
			{
				ParticlePositionsOutput << setw(6) << " ";
				for (int j = 0; j < 3; j++)
				{
					ParticlePositionsOutput << setw(12) << Body.GetCurrentParticlePosition(i, j) + Body.GetCenterOfMass(j);
				}
				i++;
			}
			ParticlePositionsOutput << endl;
		}
		StateVectorOutput.close();
		ParticlePositionsOutput.close();
		*/
		

	
	
	Body.TravelRoute();
	//Body.TravelTestRoute();

	// Eraser Length: 17.5, 7, 2.8
	// Body.EraserSimulationOne();
	
	system("pause");
	return 0;
}



//// Fix Center Of Mass
//


int FixCenterOfMass(Particle P[50], int NumberOfParticles)
{
	cout << "Determining Initial center of Mass " << endl;
	// Opening File With Given data on Rigid Body
	ifstream RigidInputFile;
	RigidInputFile.open("RigidBodyRoughData.txt");
	if (!RigidInputFile)
	{
		cout << " Error Couldn't open the dude" << endl;
	}

	// Iterating Through File to determin initial center of Mass
	double TotalMass = 0;
	double inputMass;
	double InputPosition[3];
	double WeightedPosition[3] = { 0 };
	double CenteredPosition[3];
	double WeightedCenterOfMass[3];
	int counter = -1;


	while (!RigidInputFile.eof())
	{
		RigidInputFile >> inputMass >> InputPosition[0] >> InputPosition[1] >> InputPosition[2];
		for (int i = 0; i < 3; i++)
		{
			WeightedPosition[i] += InputPosition[i] * inputMass;

		}
		TotalMass += inputMass;
		counter++;
		P[counter].SetValues(inputMass, InputPosition);
	}

	double Array[3] = { 0 };
	P[counter].SetValues(0, Array);



	cout << " Total Mass of System " << setw(10) << TotalMass << endl;
	for (int i = 0; i < 3; i++)
	{
		WeightedCenterOfMass[i] = WeightedPosition[i] / TotalMass;
	}


	RigidInputFile.close();
	for (int i = 0; i < NumberOfParticles; i++)
	{
		double NewLocation[3] = { 0,0,0 };
		for (int j = 0; j < 3; j++)
		{
			NewLocation[j] = P[i].GetPosition(j) - WeightedCenterOfMass[j];

			
		}
		P[i].SetPosition(NewLocation);

	}

	
	// Translating inputted Rigid body so that Center of Mass is at origin and outputting the resultant rigid body to another text file, called "RigidCenterData"

	RigidInputFile.open("RigidBodyRoughData.txt");
	ofstream RigidCenterData;
	RigidCenterData.open("CenteredRigidInitial.txt");
	if (!RigidCenterData.is_open())
	{
		cout << "Error!" << endl;
	}
	while (!RigidInputFile.eof())
	{
		RigidInputFile >> inputMass >> InputPosition[0] >> InputPosition[1] >> InputPosition[2];
		for (int i = 0; i < 3; i++)
		{
			CenteredPosition[i] = InputPosition[i] - WeightedCenterOfMass[i];

		}
		RigidCenterData << inputMass << " " << CenteredPosition[0] << " " << CenteredPosition[1] << " " << CenteredPosition[2] << endl;
	}
	RigidInputFile.close();
	RigidCenterData.close();

	return counter;

}

//
// Find Moment Of Inertia
//


void FindMomentOfInertia(double Moment[3][3])
{

	double StandardBasis[3][3] = { {1,0,0},{0,1,0},{0,0,1} };
	double CurrentPosition[3];
	double CurrentMass, CurrentNormSquared;


	cout << "Determining Moment of Inertia" << endl;
	// Opening File with data on centered Rigid Body
	ifstream RigidInput;
	RigidInput.open("CenteredRigidInitial.txt");
	// 
	// Determining Matrix representation of Moment of Inertia by determining the images of the elements of the standard basis under the operator as defined in the notes. 
	//

	while (!RigidInput.eof())
	{
		RigidInput >> CurrentMass >> CurrentPosition[0] >> CurrentPosition[1] >> CurrentPosition[2];
		CurrentNormSquared = DotProduct(CurrentPosition, CurrentPosition);

		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				Moment[i][j] += CurrentMass * (CurrentNormSquared*StandardBasis[i][j] - DotProduct(CurrentPosition, StandardBasis[j])*CurrentPosition[i]);
			}
		}


	}

	//
	// Printing Matrix Representation of Moment of Inertia to the screen and a text file. 
	//

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			cout << setw(9) << Moment[i][j];
		}
		cout << endl << endl << endl;
	}

}
double DotProduct(double Vec1[], double Vec2[])
{
	double Product = 0;
	for (int i = 0; i < 3; i++)
	{
		Product += Vec1[i] * Vec2[i];
	}
	return Product;
}
int GetNumParticles()
{
	int count = -4;
	ifstream TempInBoi;
		TempInBoi.open("RigidBodyRoughData.txt");
		string dude;
		while (!TempInBoi.eof())
		{
			count++;
			getline(TempInBoi, dude);
		}
		return count;
		TempInBoi.close();
}




