#include <fstream>
#include <iostream>
#include <iomanip>
#include "RigidBody.h"
#include  <cmath> 

using namespace std;

// Constructors and such

RigidBody::RigidBody()
{

}
RigidBody::RigidBody(Particle Collection[50], double Moment[3][3])
{
	for (int i = 0; i < 4; i++)
	{
		ForceFromRotors[i] = 0;
	}
	BodyMass = 0;
	for (int i = 0; i < 50; i++)
	{
		BodyMass += Collection[i].GetMass();

	}
	cout << "The Body's Total Mass is " << BodyMass << endl;
	for (int i = 0; i < 50; i++)
	{
		Particles[i] = Collection[i];
	}

	for (int i = 0; i < 50; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			FixedPositions[i][j] = Collection[i].GetPosition(j);
		}
	}

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			MomentOfInertia[i][j] = Moment[i][j];
		}
		/*
		cout << "Please Enter The Desired ";
		if (i == 0)
		{
			cout << "X ";
		}
		else if (i == 1)
		{
			cout << "Y ";
		}
		else
		{
			cout << "Z ";
		}



		cout << "Coordinate of the Body's Initial " << endl;
		cout << "Center Of Mass, Velocity, and Body Rate, in that order, separated by a space." << endl;
		cin >> CenterOfMass[i] >> Velocity[i] >> BodyRate[i];

		*/

		CenterOfMass[i] = 0;
		Velocity[i] = 0;
		BodyRate[i] = 0;
		
		// Temp Code



		//Temp Code End
		BodyTorque[i] = 0;
		TotalForce[i] = 0;
		Quaternion[i] = 0;


	}
	ExternalForce[0] = 0;
	ExternalForce[1] = 2;
	ExternalForce[2] = -9.8*BodyMass;

	InvertMatrix(MomentOfInertia, MomentOfInertiaInverse);
	Quaternion[3] = 0;
	Quaternion[0] = 1;
	VScalar(1 / BodyMass, TotalForce, CenterAcceleration);
	MixProduct(Quaternion, BodyRate, QuaternionVelocity);
	QScalar(.5, QuaternionVelocity);
	double NewDude[3];
	double OtherNewDude[3];
	MatrixOperation(MomentOfInertia, BodyRate, NewDude);
	CrossProduct(BodyRate, NewDude, OtherNewDude);
	VScalar(-1, OtherNewDude);
	VSum(BodyTorque, OtherNewDude);
	MatrixOperation(MomentOfInertiaInverse, OtherNewDude, BodyRateDerivative);
	MaxThrust = 100;
	MaxTorque = 2;

}
ofstream ControlOutput;

// Retrievers and Such

double RigidBody::GetVelocity(int i)
{
	return Velocity[i];
}
double RigidBody::GetQuaternion(int i)
{
	return Quaternion[i];
}
double RigidBody::GetCenterOfMass(int i)
{
	return CenterOfMass[i];
}
double RigidBody::GetBodyRate(int i)
{
	return BodyRate[i];
}
double RigidBody::GetBodyTorque(int i)
{
	return BodyTorque[i];
}
double RigidBody::GetQuaternionVelocity(int i)
{
	return QuaternionVelocity[i];
}
double RigidBody::GetMass()
{
	return BodyMass;
}
double RigidBody::GetParticleMass(int i)
{
	return Particles[i].GetMass();
}
double RigidBody::GetMomentOfInertia(int i, int j)
{
	return MomentOfInertia[i][j];
}
double RigidBody::GetParticlePosition(int i, int j)
{
	return Particles[i].GetPosition(j);
}
double RigidBody::GetCurrentParticlePosition(int i, int j)
{
	return CurrentParticles[i].GetPosition(j);
}
double RigidBody::GetCommandedPosition(int j)
{
	return CommandedPosition[j];
}
double RigidBody::GetCommandedVelocity(int j)
{
	return CommandedVelocity[j];
}
double RigidBody::GetCommandedAcceleration(int j)
{
	return CommandedAcceleration[j];
}
double RigidBody::GetCommandedQuaternion(int j)
{
	return CommandedQuaternion[j];
}
double RigidBody::GetCommandedQuaternionDerivative(int j)
{
	return CommandedQuaternionDerivative[j];
}
double RigidBody::GetCommandedBodyRate(int j)
{
	return CommandedBodyRate[j];
}
double RigidBody::GetCommandedBodyRateDerivative(int j)
{
	return CommandedBodyRateDerivative[j];
}
double RigidBody::GetCommandedTorque(int j)
{
	return CommandedTorque[j];
}
double RigidBody::GetCommandedThrust()
{
	return CommandedThrust;
}
double RigidBody::GetMaxTorque()
{
	return MaxTorque;
}
double RigidBody::GetMaxThrust()
{
	return MaxThrust;
}
double RigidBody::GetExternalForce(int i)
{
	return ExternalForce[i];

}
double RigidBody::GetThrust()
{
	return Thrust;
}
double RigidBody::GetTorque(int i)
{
	return BodyTorque[i];
}
double RigidBody::GetBodyRateDerivative(int i)
{
	return BodyRateDerivative[i];
}
double RigidBody::GetTotalForce(int i)
{
	return TotalForce[i];
}

// Editors

void RigidBody::SetVelocity(double NewVelocity[3], double OldVelocity[3])
{
	for (int i = 0; i < 3; i++)
	{
		OldVelocity[i] = NewVelocity[i];
	}
}
void RigidBody::SetQuaternion(double NewQuaternion[4], double OldQuaternion[4])
{
	for (int i = 0; i < 4; i++)
	{
		OldQuaternion[i] = NewQuaternion[i];
	}
}
void RigidBody::SetCenterOfMass(double NewCenterOfMass[3], double OldCenterOfMass[3])
{
	for (int i = 0; i < 3; i++)
	{
		OldCenterOfMass[i] = NewCenterOfMass[i];
	}
}
void RigidBody::SetBodyRate(double NewBodyRate[3], double OldBodyRate[3])
{
	for (int i = 0; i < 3; i++)
	{
		OldBodyRate[i] = NewBodyRate[i];
	}
}
void RigidBody::SetBodyTorque(double NewBodyTorque[3])
{
	for (int i = 0; i < 3; i++)
	{
		BodyTorque[i] = NewBodyTorque[i];
	}
}
void RigidBody::SetQuaternionVelocity(double NewQVelocity[4], double OldQVelocity[4])
{
	for (int i = 0; i < 4; i++)
	{
		OldQVelocity[i] = NewQVelocity[i];
	}
}
void RigidBody::SetCommandedPosition(double K[3])
{
	for (int i = 0; i < 3; i++)
	{
		CommandedPosition[i] = K[i];
	}
}
void RigidBody::SetCommandedVelocity(double K[3])
{
	for (int i = 0; i < 3; i++)
	{
		CommandedVelocity[i] = K[i];
	}
}
void RigidBody::SetCommandedAcceleration(double K[3])
{
	for (int i = 0; i < 3; i++)
	{
		CommandedAcceleration[i] = K[i];
	}
}
void RigidBody::SetCommandedQuaternion(double K[4])
{
	for (int i = 0; i < 4; i++)
	{
		CommandedQuaternion[i] = K[i];
	}
}
void RigidBody::SetCommandedQuaternionDerivative(double K[4])
{
	for (int i = 0; i < 4; i++)
	{
		CommandedQuaternionDerivative[i] = K[i];
	}
}
void RigidBody::SetCommandedBodyRate(double K[3])
{
	for (int i = 0; i < 3; i++)
	{
		CommandedBodyRate[i] = K[i];
	}
}
void RigidBody::SetCommandedBodyRateDerivative(double K[3])
{
	for (int i = 0; i < 3; i++)
	{
		CommandedBodyRateDerivative[i] = K[i];
	}
}
void RigidBody::SetCommandedTorque(double K[3])
{
	for (int i = 0; i < 3; i++)
	{
		CommandedTorque[i] = K[i];
	}
}
void RigidBody::SetCommandedThrust(double K)
{
	CommandedThrust = K;
}
void RigidBody::SetMaxTorque(double K)
{
	MaxTorque = K;

}
void RigidBody::SetMaxThrust(double K)
{
	MaxThrust = K;
}
void RigidBody::SetExternalForce(double K[3])
{
	for (int i = 0; i < 3; i++)
	{
		ExternalForce[i] = K[3];
	}
}
void RigidBody::SetForce(double K[3])
{
	for (int i = 0; i < 3; i++)
	{
		TotalForce[i] = K[i];
	}
}
void RigidBody::SetThrust(double K)
{
	Thrust = K;
}

//
//
// Basic Mathematical Operations
//
//

// Sum Of Quaterniions and Vectors
void RigidBody::QSum(double Q1[4], double Q2[4], double ReturnQ[4])
{
	for (int i = 0; i < 4; i++)
	{
		ReturnQ[i] = Q1[i] + Q2[i];

	}
}
void RigidBody::QSum(double Q1[4], double Q2[4])
{
	for (int i = 0; i < 4; i++)
	{
		Q1[i] += Q2[i];
	}
}
void RigidBody::VSum(double V1[3], double V2[3], double ReturnV[3])
{
	for (int i = 0; i < 3; i++)
	{
		ReturnV[i] = V1[i] + V2[i];
	}
}
void RigidBody::VSum(double V1[3], double V2[3])
{
	for (int i = 0; i < 3; i++)
	{
		V1[i] += V2[i];
	}
}
void RigidBody::MixSum(double Q[4], double V[3])
{
	for (int i = 0; i < 3; i++)
	{
		Q[i + 1] += V[i];
	}
}
void RigidBody::MixSum(double Q[4], double V[3], double OutQ[4])
{
	for (int i = 0; i < 3; i++)
	{
		OutQ[i + 1] = Q[i + 1] + V[i];
	}
	OutQ[0] = Q[0];
}

// Scalaing Vectors, Operators, and Quaternions
void RigidBody::QScalar(double a, double Q[4], double OutQ[4])
{
	for (int i = 0; i < 4; i++)
	{
		OutQ[i] = a * Q[i];
	}
}
void RigidBody::VScalar(double a, double V[3], double OutV[3])
{
	for (int i = 0; i < 3; i++)
	{
		OutV[i] = a * V[i];
	}
}
void RigidBody::VScalar(double a, double V[3])
{
	for (int i = 0; i < 3; i++)
	{
		V[i] *= a;
	}
}
void RigidBody::QScalar(double a, double Q[4])
{
	for (int i = 0; i < 4; i++)
	{
		Q[i] *= a;
	}
}
void RigidBody::MScalar(double a, double M[3][3])
{
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			M[i][j] *= a;
		}
	}
}
void RigidBody::MScalar(double a, double M[3][3], double OutM[3][3])
{
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			OutM[i][j] = M[i][j] * a;
		}
	}
}
void RigidBody::VNormalize(double Vector[3])
{
	double a = InnerProduct(Vector, Vector);
	double b = sqrt(a);
	VScalar(1 / b, Vector);
}
void RigidBody::QNormalize(double Q[4])
{
	double F[3];
	QuaternionToVector(Q, F);
	double a = Q[0] * Q[0] + InnerProduct(F, F);
	double b = sqrt(a);
	QScalar(1 / b, Q);
}
double RigidBody::QNorm(double Q[4])
{
	double F[3];
	QuaternionToVector(Q, F);
	double a = Q[0] * Q[0] + InnerProduct(F, F);
	double b = sqrt(a);
	return b;
}

// Swapping between Vectors and Quaternions
void RigidBody::VectorToQuaternion(double Vector[3], double Quaternion[4])
{
	Quaternion[0] = 0;
	for (int i = 0; i < 3; i++)
	{
		Quaternion[i + 1] = Vector[i];

	}
}
void RigidBody::QuaternionToVector(double Quaternion[4], double Vector[3])
{
	for (int i = 0; i < 3; i++)
	{
		Vector[i] = Quaternion[i + 1];
	}
}
int RigidBody::sgn(double a)
{
	if (a > 0)
	{
		return 1;
	}
	else if (a < 0)
	{
		return -1;
	}
	else
	{
		return 0;
	}
}
double RigidBody::abs(double a)
{
	if (a < 0)
	{
		return -1.0*a;
	}
	else
	{
		return a;
	}
}


// Products Of Various Types

// Saves Answer as OutputQ
void RigidBody::QProduct(double Q1[4], double Q2[4], double OutputQ[4]) 
{
	double V1[3] = { Q1[1], Q1[2], Q1[3] };
	double V2[3] = { Q2[1], Q2[2], Q2[3] };
	double Cross[3];
	CrossProduct(V1, V2, Cross);
	double a = InnerProduct(V1, V2);
	VScalar(Q1[0], V2);
	VScalar(Q2[0], V1);


	for (int i = 0; i < 4; i++)
	{
		OutputQ[i] = 0;
	}
	OutputQ[0] = Q1[0] * Q2[0] - a;
	MixSum(OutputQ, V1);
	MixSum(OutputQ, V2);
	MixSum(OutputQ, Cross);
}
//Saves Answer As Q1
void RigidBody::QProduct(double Q1[4], double Q2[4]) 
{
	double LocalDude[4];
	QProduct(Q1, Q2, LocalDude);
	for (int i = 0; i < 4; i++)
	{
		Q1[i] = LocalDude[i];

	}
}
// Saves Answer As Q
void RigidBody::MixProduct(double Q[4], double V[3])
{
	double LocalVariable[4];
	VectorToQuaternion(V, LocalVariable);
	QProduct(Q, LocalVariable);
	for (int i = 0; i < 4; i++)
	{
		Q[i] = LocalVariable[i];
	}

}
void RigidBody::MixProduct(double Q[4], double V[3], double OutQ[4])
{
	double LocalVariable[4] = { 0, V[0], V[1], V[2] };
	QProduct(Q, LocalVariable, OutQ);
}
double RigidBody::InnerProduct(double Vec1[3], double Vec2[3])
{
	double a = 0;
	for (int i = 0; i < 3; i++)
	{
		a += Vec1[i] * Vec2[i];

	}
	return a;
}
void RigidBody::CrossProduct(double Vec1[3], double Vec2[3], double ReturnVec[3])
{
	ReturnVec[0] = Vec1[1] * Vec2[2] - Vec1[2] * Vec2[1];
	ReturnVec[1] = Vec1[2] * Vec2[0] - Vec1[0] * Vec2[2];
	ReturnVec[2] = Vec1[0] * Vec2[1] - Vec1[1] * Vec2[0];
}
void RigidBody::MatrixOperation(double M[3][3], double V[3], double OutV[3])
{
	for (int i = 0; i < 3; i++)
	{
		OutV[i] = 0;
		for (int j = 0; j < 3; j++)
		{
			OutV[i] += M[i][j] * V[j];
		}
	}
}
void RigidBody::MatrixOperation(double M[3][3], double V[3])
{
	double LocalVariable[3];
	MatrixOperation(M, V, LocalVariable);
	for (int i = 0; i < 3; i++)
	{
		V[i] = LocalVariable[i];
	}
}
void RigidBody::OperatorComposition(double M1[3][3], double M2[3][3], double OutM[3][3])
{
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			OutM[i][j] = 0;
			for (int k = 0; k < 3; k++)
			{
				OutM[i][j] += M1[i][k] * M2[k][j];
			}

		}
	}
}
void RigidBody::OperatorComposition(double M1[3][3], double M2[3][3])
{
	double LocalVariable[3][3];
	OperatorComposition(M1, M2, LocalVariable);
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			M1[i][j] = LocalVariable[i][j];
		}
	}
}
void RigidBody::InvertMatrix(double M1[3][3], double  M2[3][3])
{
	double DummyMatrix[3][3];
	AdjugateMatrix(M1, DummyMatrix);
	MScalar(1 / Det(M1), DummyMatrix, M2);

}
void RigidBody::AdjugateMatrix(double M1[3][3], double M2[3][3])
{
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			M2[j][i] = CoFactor(M1, i, j);
		}
	}
}
double RigidBody::CoFactor(double M[3][3], int i, int j)
{
	int a = 1;
	for (int k = 0; k < i + j; k++)
	{
		a *= -1;
	}

	double NewMatrix[2][2];
	for (int m = 0; m < 2; m++)
	{
		for (int n = 0; n < 2; n++)
		{
			if (m < i&&n < j)
			{
				NewMatrix[m][n] = M[m][n];
			}
			else if (m < i)
			{
				NewMatrix[m][n] = M[m][n + 1];
			}
			else if (n < j)
			{
				NewMatrix[m][n] = M[m + 1][n];
			}
			else
			{
				NewMatrix[m][n] = M[m + 1][n + 1];
			}
		}
	}
	double b = NewMatrix[0][0] * NewMatrix[1][1] - NewMatrix[0][1] * NewMatrix[1][0];
	double c = a * b;
	return c;

}
double RigidBody::Det(double M[3][3])
{
	double a = M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) - M[0][1] * (M[1][0] * M[2][2] - M[2][0] * M[1][2]) + M[0][2] * (M[1][0] * M[2][1] - M[2][0] * M[1][1]);

	return a;
}

//
// Physicsey Methods
//

void RigidBody::CalcRFromQ(double Q[4], double R[3][3])
{
	R[0][0] = Q[0] * Q[0] + Q[1] * Q[1] - Q[2] * Q[2] - Q[3] * Q[3];
	R[0][1] = 2 * (Q[1] * Q[2] - Q[0] * Q[3]);
	R[0][2] = 2 * (Q[1] * Q[3] + Q[0] * Q[2]);
	R[1][0] = 2 * (Q[1] * Q[2] + Q[0] * Q[3]);
	R[1][1] = Q[0] * Q[0] - Q[1] * Q[1] + Q[2] * Q[2] - Q[3] * Q[3];
	R[1][2] = 2 * (Q[2] * Q[3] - Q[0] * Q[1]);
	R[2][0] = 2 * (Q[1] * Q[3] - Q[0] * Q[2]);
	R[2][1] = 2 * (Q[2] * Q[3] + Q[0] * Q[1]);
	R[2][2] = Q[0] * Q[0] - Q[1] * Q[1] - Q[2] * Q[2] + Q[3] * Q[3];
}
void RigidBody::CalcRStar(double R[3][3], double RS[3][3])
{
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			RS[i][j] = R[j][i];
		}
	}
}
void RigidBody::CalculateTorqueFromRotorForces()
{
	double TauPrime[3] = { 0, 0, 0 };
	double FRTorque[3];
	double BRTorque[3];
	double FLTorque[3];
	double BLTorque[3];




	double FRForce[3] = { 0, 0, ForceFromRotors[0] };
	double BRForce[3] = { 0, 0, ForceFromRotors[1] };
	double FLForce[3] = { 0, 0, ForceFromRotors[2] };
	double BLForce[3] = { 0, 0, ForceFromRotors[3] };

	double CurrentQ[4];

	for (int i = 0; i < 4; i++)
	{
		CurrentQ[i] = GetQuaternion(i);
	}

	

	double R[3][3];

	CalcRFromQ(CurrentQ, R);


	MatrixOperation(R, FRForce);
	MatrixOperation(R, BRForce);
	MatrixOperation(R, FLForce);
	MatrixOperation(R, BLForce);




	double FRPosition[3];
	double BRPosition[3];
	double FLPosition[3];
	double BLPosition[3];

	

	for (int i = 0; i < 3; i++)
	{
		FRPosition[i] = FixedPositions[1][i];
		BRPosition[i] = FixedPositions[3][i];
		FLPosition[i] = FixedPositions[4][i];
		BLPosition[i] = FixedPositions[6][i];

	}


	CrossProduct(FRPosition, FRForce, FRTorque);
	CrossProduct(BRPosition, BRForce, BRTorque);
	CrossProduct(FLPosition, FLForce, FLTorque);
	CrossProduct(BLPosition, BLForce, BLTorque);


	for (int i = 0; i < 3; i++)
	{
		TauPrime[i] += FRTorque[i] + BRTorque[i] + FLTorque[i] + BLTorque[i];

	}

	MatrixOperation(R, TauPrime);


	SetBodyTorque(TauPrime);



}


// Simulators
void RigidBody::Timestep(double dt)
{
	Timestep(dt, CenterOfMass, Velocity, CenterAcceleration, Quaternion, QuaternionVelocity, BodyRate, BodyRateDerivative, BodyTorque, MomentOfInertia, TotalForce, BodyMass, MomentOfInertiaInverse, RotationMatrix);
}
void RigidBody::Timestep(double dt, double COM[3], double V[3], double VDot[3], double Q[4], double QDot[4], double BR[3], double BRDot[3], double BT[3], double MOI[3][3], double Force[3], double m, double MoiInverse[3][3], double Rotation[3][3])
{



	
	
	
	// Calculating Derivative Of New State Vector (Except velocity, which is part of State Vector)
	//VDot
	VScalar(1 / m, Force, VDot);

	//QDot

	double dummy[4] = { 0, BR[0], BR[1], BR[2] };
	QProduct(Q, dummy, QDot);
	QScalar(.5, QDot);
	
	
	// BRDot
	double NewDude[3]; // Will Hold BodyRate
	MatrixOperation(MOI, BR, NewDude);
	double OtherNewDude[3];
	CrossProduct(BR, NewDude, OtherNewDude);
	VScalar(-1, OtherNewDude);
	double Torqeydude[3];
	for (int i = 0; i < 3; i++)
	{
		Torqeydude[i] = GetTorque(i);
	}
	VSum(OtherNewDude, Torqeydude);
	MatrixOperation(MoiInverse, OtherNewDude, BRDot);

	



	// Position
	double dr[3];
	VScalar(dt, V, dr);
	VSum(COM, dr);


	//Velocity Return to this one
	double dv[3];
	VScalar(dt, VDot, dv);
	VSum(V, dv);


	// Quaternion
	double dq[4];
	QScalar(dt, QDot, dq);
	QSum(Q, dq);
	QNormalize(Q);


	 //Body Rate
	double dw[3];
	VScalar(dt, BRDot, dw);
	VSum(BR, dw);
	CalcRFromQ(Q, Rotation);




	//
	//
	// And output Their Updated values Here
	//
	//
}

//
//
// Controllers
//
//



void RigidBody::OutputSystemState()
{

	int i = 0;
	while (!GetParticleMass(i) == 0)
	{
		for (int j = 0; j < 3; j++)
		{
			ControlOutput << setw(12) << GetCurrentParticlePosition(i, j) + GetCenterOfMass(j);
		}
		i++;
	}

	double QuaternionThing[4];
	for (int i = 0; i < 4; i++)
	{
		QuaternionThing[i] = GetQuaternion(i);
	}
	double DummyR[3][3];
	CalcRFromQ(QuaternionThing, DummyR);
	ControlOutput << setw(15);
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			ControlOutput << setw(12) << DummyR[i][j];
			//cout << setw(12) << DummyR[i][j];
		}
		//cout << endl;
	}
	for (int i = 0; i < 3; i++)
	{
		ControlOutput << setw(12) << GetCenterOfMass(i);
	}

	ControlOutput << setw(12) << GetCommandedPosition(0) << setw(12) << GetCommandedPosition(1) << setw(12) << GetCommandedPosition(2);
	ControlOutput << endl;
}

// 
// Basic Commanders
//
void RigidBody::UpdateCommandedPosition(double NewPosition[3])
{
	SetCommandedPosition(NewPosition);
}
// Updates Commanded velocity to be consistent with commanded Position
void RigidBody::UpdateCommandedVelocity()
{




	// MakingTemporary Variables to contain commanded position, the difference, and the current velocity
	
	double dummy[3];
	double TempCommandedP[3];
	double difference[3];
	double TempV[3];
	double FuturePosition[3];
	for (int i = 0; i < 3; i++)
	{
		TempCommandedP[i] = GetCommandedPosition(i);
		FuturePosition[i] = CenterOfMass[i];

	}
	FuturePosition[0] += .45*sgn(Velocity[0])*Velocity[0] * Velocity[0];
	FuturePosition[1] += .45*sgn(Velocity[1])*Velocity[1] * Velocity[1];
	FuturePosition[2] += .15*sgn(Velocity[2])*Velocity[2] * Velocity[2];
	for(int i=0;i<3;i++)
	{
		difference[i] = TempCommandedP[i] -FuturePosition[i];
		TempV[i] = GetVelocity(i);
	}
	double a = InnerProduct(difference, TempV);
	double b = InnerProduct(difference, difference);
	double ProjectionOfVelocityOntoDifference[3];
	if (b <.000001)
	{
		double dankMank[3] = { 0, 0, 0 };
		SetCommandedVelocity(dankMank);
	}
	else 
	{
		for (int i = 0; i < 3; i++)
		{
			ProjectionOfVelocityOntoDifference[i] = difference[i] * a / b;
		}

		double PerpindicularComponent[3];
		for (int i = 0; i < 3; i++)		
		{
			PerpindicularComponent[i] = TempV[i] - ProjectionOfVelocityOntoDifference[i];
		}

		if (a > 0)
		{
			for (int i = 0; i < 3; i++)
			{
				dummy[i] = .7*(abs(GetCommandedPosition(i) - GetCenterOfMass(i)) *sgn(GetCommandedPosition(i) - GetCenterOfMass(i)) - 4 * PerpindicularComponent[i]);
			}
		}
		else
		{
			for (int i = 0; i < 3; i++)
			{
				dummy[i] = (sqrt(abs(GetCommandedPosition(i) - GetCenterOfMass(i)))*sgn(GetCommandedPosition(i) - GetCenterOfMass(i)) - 4 * PerpindicularComponent[i]);
			}
		}


		dummy[2] = sqrt(abs(GetCommandedPosition(2) - GetCenterOfMass(2) - .2*GetVelocity(2)))*sgn(GetCommandedPosition(2) - GetCenterOfMass(2) - .2*GetVelocity(2));

		SetCommandedVelocity(dummy);

		
	}
	
}
// Updates Commanded Acceleration to be consistent with Commanded velocity
void RigidBody::UpdateCommandedAcceleration()
{
	
	
	double TempDifference[3];
	for (int i = 0; i < 3; i++)
	{
		TempDifference[i] = GetCommandedVelocity(i) - GetVelocity(i);
	}
	double b = InnerProduct(TempDifference, TempDifference);
	if (b < .00000001)
	{
		double dnak[3] = { 0, 0, 0 };
		SetCommandedAcceleration(dnak);
	}
	else
	{
		double dummy[3];
		double TempCommandedV[3];
		for (int i = 0; i < 3; i++)
		{
			TempCommandedV[i] = GetCommandedVelocity(i);
		}
		double difference[3];
		for (int i = 0; i < 3; i++)
		{
			difference[i] = TempCommandedV[i] - GetVelocity(i);
		}
		double Tempa[3];
		for (int i = 0; i < 3; i++)
		{
			Tempa[i] = CenterAcceleration[i];
		}
		double a = InnerProduct(Tempa, difference);
		if (a > 0)
		{
			for (int i = 0; i < 3; i++)
			{
				dummy[i] = sqrt(abs(GetCommandedVelocity(i) - GetVelocity(i))) *sgn(GetCommandedVelocity(i) - GetVelocity(i));
			}
		}
		else
		{
			for (int i = 0; i < 3; i++)
			{
				dummy[i] = 5 *sqrt( sqrt(sqrt(abs(GetCommandedVelocity(i) - GetVelocity(i)))))*sgn(GetCommandedVelocity(i) - GetVelocity(i));
			}
			dummy[0] *=1.1;
			dummy[1] *=1.1;
		}
		dummy[0] *= 1.1;
		dummy[1] *= 1.1;


		SetCommandedAcceleration(dummy);
		}
	
}
void RigidBody::DisplaceDrone(double displacement[3])
{
	GoToPosition(displacement[0]+GetCenterOfMass(0), displacement[1]+GetCenterOfMass(1), displacement[2]+GetCenterOfMass(2), 30);
}
void RigidBody::UpdateCommandedThrust()
{
	double DesiredForceProduction[3];
		DesiredForceProduction[0] = (GetCommandedAcceleration(0)*BodyMass-ExternalForce[0]);
		DesiredForceProduction[1] = (GetCommandedAcceleration(1)*BodyMass-ExternalForce[1]);
		DesiredForceProduction[2] = ( GetCommandedAcceleration(2)*BodyMass - ExternalForce[2]);
		double a = InnerProduct(DesiredForceProduction,DesiredForceProduction);
		double b = sqrt(a);

		

		SetCommandedThrust(b);
}
void RigidBody::UpdateCommandedQuaternion()
{


	double dummy[3];// Gives X y and Z coordinates which we're gonna solve for
	dummy[0] = (BodyMass*GetCommandedAcceleration(0) - GetExternalForce(0)) / (2*GetThrust());
	dummy[1] = (BodyMass*GetCommandedAcceleration(1) - GetExternalForce(1)) / (2 * GetThrust());
	dummy[2] = (BodyMass*GetCommandedAcceleration(2) - GetExternalForce(2)) / GetThrust();

	double q0squared = (dummy[2] + sqrt(dummy[2] * dummy[2] + 4 * (dummy[1] * dummy[1] + dummy[0] * dummy[0]))) / 2;
	double TempCommandQ[4];
	TempCommandQ[0] = sqrt(q0squared);
	TempCommandQ[3] = 0;
	TempCommandQ[1] = -1.0*dummy[1] / TempCommandQ[0];
	TempCommandQ[2] = dummy[0] / TempCommandQ[0];
	QNormalize(TempCommandQ);

	SetCommandedQuaternion(TempCommandQ);

}
void RigidBody::UpdateCommandedQuaternionDerivative()
{
	
	double Difference[4];
	for (int i = 0; i < 4; i++)
	{
		Difference[i] = GetCommandedQuaternion(i) - GetQuaternion(i);
	}
	double GoalNorm = sqrt(InnerProduct(Difference, Difference));
	
	
	if (GoalNorm<.0000001)
	{
		double dummy[4] = { 0, 0, 0, 0 };
		SetCommandedQuaternionDerivative(dummy);
	}
	else 
	{
		// Finding component of difference parallel to Current quaternion
		double dummy[4];
		for (int i = 0; i < 4; i++)
		{
			dummy[i] = GetQuaternion(i);
		}
		double RelativeSize;
		RelativeSize = InnerProduct(Difference, dummy);
		VScalar(-1.0*RelativeSize, dummy);
		VSum(Difference, dummy);
		
		VNormalize(Difference);
		VScalar(GoalNorm, Difference);
		

		// Finding Projection of Quaternion Derivative Along Difference

		 
		double a = 0;
		double b = 0;

		for (int i = 0; i < 4; i++)
		{
			a += GetQuaternionVelocity(i)*Difference[i];
			b += Difference[i] * Difference[i];
		}


		// Finding Component of Quaterniion Derivative Orthogonal To Difference
		// Subtracting that Component From difference To Prevent Massive Rotations

		double PerpVelocity[4];
		for (int i = 0; i < 4; i++)
		{
			PerpVelocity[i] = GetQuaternionVelocity(i) - Difference[i] * a / b;
			Difference[i] = Difference[i] - PerpVelocity[i];
		}
		VNormalize(Difference);
		VScalar(GoalNorm, Difference);

		for (int i = 0; i < 4; i++)
		{
			Difference[i] = 2.0*Difference[i];
		}
		




		SetCommandedQuaternionDerivative(Difference);
	}


	

	
}
void RigidBody::UpdateCommandedBodyRate()
{
	double Q[4];
	double QDot[4]; // Since QDot=(1/2)Q W, we have 2 QStar Q=WCommanded
	for (int i = 0; i<4; i++)
	{
		Q[i] = -2.0*GetQuaternion(i);
		QDot[i] = GetCommandedQuaternionDerivative(i);
	}
	Q[0] = 2*GetQuaternion(0);
	double dummy[4];
	QProduct(Q, QDot, dummy);
	double dood[3];
	for (int i = 0; i < 3; i++)
	{
		dood[i] =  dummy[i + 1];
	}

	SetCommandedBodyRate(dood);


}
void RigidBody::UpdateCommandedBodyRateDerivative()
{
	double dummy[3];
	for (int i = 0; i < 3; i++)
	{
		dummy[i] =  sqrt(abs(GetCommandedBodyRate(i) - GetBodyRate(i)))*sgn(GetCommandedBodyRate(i) - GetBodyRate(i));
	}
	SetCommandedBodyRateDerivative(dummy);
}
void RigidBody::UpdateCommandedTorque()
{
	double CBR[3];
	double CBRDot[3];
	for (int i = 0; i < 3; i++)
	{
		CBR[i] = GetCommandedBodyRate(i);
		CBRDot[i] = GetCommandedBodyRateDerivative(i);
	}
		double ICBR[3];
		double cross[3];
		MatrixOperation(MomentOfInertia, CBR, ICBR);
		CrossProduct(CBR, ICBR, cross);
		double ICBRDot[3];
		MatrixOperation(MomentOfInertia, CBRDot, ICBRDot);
		double dood[3];
		VSum(ICBRDot, cross, dood);
		SetCommandedTorque(dood);


}

//
// Assigns values directly based off of commanderss
//

void RigidBody::UpdateTorque()
{


	double TempCommandBodyTorque[3];
	for (int i = 0; i < 3; i++)
	{
		TempCommandBodyTorque[i] =3*GetCommandedTorque(i);
	}
	double DesiredNorm = sqrt(InnerProduct(TempCommandBodyTorque, TempCommandBodyTorque));
	SetBodyTorque(TempCommandBodyTorque);



		

}
void RigidBody::UpdateThrust()
{
	double dood[3] = { 0, 0, 1 };
	double TempQ[4];
	for (int i = 0; i < 4; i++)
	{
		TempQ[i] = GetQuaternion(i);
	}
	double rotator[3][3];
	CalcRFromQ(TempQ, rotator);
	MatrixOperation(rotator, dood);
	double CurrentWantedAcceleration[3];
	CurrentWantedAcceleration[0] = (BodyMass*GetCommandedAcceleration(0) - ExternalForce[0]) / 2;
	CurrentWantedAcceleration[1] = (BodyMass*GetCommandedAcceleration(1) - ExternalForce[1]) / 2;
	CurrentWantedAcceleration[2] = BodyMass * GetCommandedAcceleration(2) - ExternalForce[2];
	VNormalize(CurrentWantedAcceleration);
	double scaling;
	double a = InnerProduct(dood, CurrentWantedAcceleration);
	if (a < .1)
	{
		scaling = .1;
	}
	else
	{
		scaling = a;
	}
	scaling *= GetCommandedThrust();
	SetThrust(scaling);
		
		
}
void RigidBody::UpdatePositions()
{
	for (int i = 0; i < 50; i++)
	{
		double DummyVariable[3];
		for (int j = 0; j < 3; j++)
		{
			DummyVariable[j] = Particles[i].GetPosition(j);
		}
		double AnotherDummyVariable[3];
		MatrixOperation(RotationMatrix, DummyVariable, AnotherDummyVariable);
		for (int j = 0; j < 3; j++)
		{
			CurrentParticles[i].SetPosition(AnotherDummyVariable);
		}


	}
}



//
// Actual High Level Control Algorithms
//


void RigidBody::GoToPosition(double x, double y, double z, double K)
{
	double dummy[4];
	dummy[0] = x;
	dummy[1] = y;
	dummy[2] = z;
	dummy[3] = K;
	GoToPosition(dummy);
}
void RigidBody::GoToPosition(double K[4])
{
	cout << "Going" << endl;
	int ConsecutiveInTheres = 0;
	double NeededConsecutiveInTheres = K[3];
	double d[3];
	for (int i = 0; i < 3; i++)
	{
		d[i] = K[i];
	}
	SetCommandedPosition(d);
	double margin = .05;
	double CheckCenter[3];
	double gamma;
	int count = 0;
	int othercount = 0;
	int HasBeenThere = 0;
	int kappa = 0;

	while (ConsecutiveInTheres < NeededConsecutiveInTheres)
	{
		UpdateCommandedVelocity();
		UpdateCommandedAcceleration();
		UpdateCommandedThrust();
		UpdateThrust();
		UpdateCommandedQuaternion();
		UpdateCommandedQuaternionDerivative();
		UpdateCommandedBodyRate();
		UpdateCommandedBodyRateDerivative();
		UpdateCommandedTorque();
		UpdateTorque();
		UpdateForce();
		Timestep(.002);
		UpdatePositions();


		// Check Distance From Center of mass to goal position

		for (int i = 0; i < 3; i++)
		{
			CheckCenter[i] = GetCommandedPosition(i) - GetCenterOfMass(i);
		}

		gamma = abs(InnerProduct(CheckCenter, CheckCenter));

		if (gamma < margin)
		{
			ConsecutiveInTheres++;

		}
		else
		{
			ConsecutiveInTheres = 0;

		}
		OutputSystemState();
	}
	
}
void RigidBody::GoToSmoothly( double K[4])
{	
	double MidPoint[4];
	MidPoint[3] = 300;
	MidPoint[0] = GetCenterOfMass(0);
	MidPoint[1] = K[1];
	MidPoint[2] = K[2] - GetCenterOfMass(2);
	GoToPosition(MidPoint);
	GoToPosition(K);
}
void RigidBody::UpdateForce()
{
	double Exterior[3];
	double GeneratedThrust[3];

	for (int i = 0; i < 3; i++)
	{
		Exterior[i] = GetExternalForce(i);
		GeneratedThrust[i] = 0;
	}
	GeneratedThrust[2] = GetThrust();
	double AngledThrust[3];
	double Dank[3][3];
	double qdood[4];
	for (int i = 0; i < 4; i++)
	{
		qdood[i] = GetQuaternion(i);
	}
	CalcRFromQ(qdood, Dank);
	MatrixOperation(Dank, GeneratedThrust, AngledThrust);
	VSum(AngledThrust, Exterior);
	SetForce(AngledThrust);


}
void RigidBody::TravelRoute()
{
	ControlOutput.open("ControlOutput.txt");

	cout << "How Many Set Points would you like to travel To?" << endl;
	int NumDestinations;
	cin >> NumDestinations;

	double Destinations[50][4];
	for (int i = 0; i < NumDestinations; i++)
	{
		cout << "Please Output the X, Y, and Z  Coordinate of destination number " << i+1 << "?" << endl;
		cin >> Destinations[i][0] >> Destinations[i][1] >> Destinations[i][2];
	}
	for (int i = 0; i < NumDestinations; i++)
	{
		cout << "For how many cycles should the center of mass linger at destination number " << i + 1 << "?" << endl;
		cin >> Destinations[i][3];
	}


	if (!ControlOutput)
	{
		cout << "Thats Wak" << endl;
		system("pause");
	}

	for (int i = 0; i < NumDestinations; i++)
	{
		double dummy[4];
		for (int j = 0; j < 3; j++)
		{
			dummy[j] = Destinations[i][j];
		}
		cout << "Heading to particle: " << i << endl;
		GoToSmoothly(dummy);
		
	}
	ControlOutput.close();
}
void RigidBody::TravelTestRoute()
{
	ControlOutput.open("ControlOutput.txt");
	double Destination[4] = {0,0, 0, 2000 };
	double Destination1[4] = { 0, 2, 0 ,2000 };
	double Destination2[4] = { 2,0,0,2000 };
	double Destination3[4] = { 0,0,2,2000 };


	for (int i = 0; i < 2; i++)
	{
		MakeAStar();
	}
	MakeACircle();
	ControlOutput.close();
	cout << "Success" << endl;
	system("pause");

	
}
void RigidBody::MakeAStar()
{
	double a=8;
	double b=-20;
	cout << "Making A Star" << endl;
	cout << "First Point" << endl;
	GoToPosition(0, b, a, 500);
	GoToPosition(.25*a, b, .25*a, 500);
	cout << "Second Point " << endl;
	GoToPosition(a, b, 0, 500);
	GoToPosition(.25*a, b, -.25*a, 500);
	cout << "Third Point " << endl;
	GoToPosition(0, b, -a, 500);
	GoToPosition(-0.25*a, b, -0.25*a, 500);
	cout << "Fourth Point " << endl;
	GoToPosition( -a, b, 0, 500);
	GoToPosition(-0.25*a, b, .25*a, 500);
	cout << "ReturnToStart" << endl;
	GoToPosition(0, b, a, 500);

	
}
void RigidBody::MakeACircle()
{
	double r = 10;
	double b =-20;
	GoToPosition(0, b, 0, 1000);
	for (int i = 0; i < 12; i++)
	{
		GoToPosition(r*sin(2*3.14*i/12.0), b, r*cos(2*3.14*i/12.0), 10);
	}



}
void RigidBody::PrintSystemState()
{
	cout << endl << endl << "---------------------------------------------------------------------------------------------------------------------------------------------------------------------" << endl;
	cout << setw(15) << "Q" << setw(15) << "QDOt" << setw(15) << "CQ " << setw(15) << "CQDot" << endl;
	for (int i = 0; i < 4; i++)
	{
		cout << setw(15) << GetQuaternion(i) << setw(15) << GetQuaternionVelocity(i) << setw(15) << GetCommandedQuaternion(i) << setw(15) << GetCommandedQuaternionDerivative(i) << endl;
	}

	cout << endl << endl;

	cout << setw(15) << "COM " << setw(15) << "ComPosit" << setw(15) << "Velocity" << setw(15) << "CVelocity" << setw(15) << "CAccel"  << setw(15) << "BR" << setw(15) <<  "CBR" << setw(15) << "BRDot" << setw(15) << "CBRDOT" << setw(15) << "CTorque" << setw(15) << "Torque" << setw(15) << "TotalForce" << setw(15) << "External Force" <<endl;
	for (int i = 0; i < 3; i++)
	{
		cout << setw(15) << GetCenterOfMass(i) << setw(15) << GetCommandedPosition(i) << setw(15) << GetVelocity(i) << setw(15) << GetCommandedVelocity(i) << setw(15) << GetCommandedAcceleration(i) << setw(15) << GetBodyRate(i) << setw(15) << GetCommandedBodyRate(i) << setw(15) << GetBodyRateDerivative(i) << setw(15) << GetCommandedBodyRateDerivative(i) << setw(15) << GetCommandedTorque(i) << setw(15) << GetTorque(i) << setw(15) << GetTotalForce(i)<< endl;
	}

	cout << "Commanded Thrust:" << GetCommandedThrust() << endl;
	cout << "Thrust : " << GetThrust() << endl;
	system("pause");
}
void RigidBody::EraserSimulationOne()
{
	ControlOutput.open("EraserSimulationOne.txt");
	double dt = .001;
	ExternalForce[2] = 0;
	for (int i = 0; i < 2000; i++)
	{
		
		Timestep(dt);
		UpdatePositions(); 
		OutputSystemState();

	}

	// For First Simulation, force2=20, force1=1, Apply for 200 cycles, 

	ForceFromRotors[0] =20;
	ForceFromRotors[1] =20;
	CalculateTorqueFromRotorForces();



	for (int i = 0; i < 200; i++)
	{
		Timestep(dt);
		UpdatePositions();
		OutputSystemState();
	}

	ForceFromRotors[0] = 0;
	ForceFromRotors[1] = 0;
	CalculateTorqueFromRotorForces();
	for (int i = 0; i < 30000;i++)
	{


		Timestep(dt);
		UpdatePositions();
		OutputSystemState();
	}

	ControlOutput.close();
}
