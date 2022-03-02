#include <iostream>
#include "Particle.h"

using namespace std;

// Constructors and such

Particle::Particle()
{
	ParticleMass = 0;
	for (int i = 0; i < 3; i++)
	{
		ParticlePosition[i] = 0;
		ParticleForce[i] = 0;
	}

}

Particle::Particle(double Mass, double FirstPosition[3])
{
	ParticleMass = Mass;
	for (int i = 0; i < 3; i++)
	{
		ParticlePosition[i] = FirstPosition[i];
		ParticleForce[i] = 0;
	}
}


// Retrievers

double Particle::GetMass()
{
	return ParticleMass;
}
double Particle::GetForce(int Component)
{
	return ParticleForce[Component];
}
double Particle::GetPosition(int Component)
{
	return ParticlePosition[Component];
}



// Value Setters

void Particle::SetForce(double NewForce[3])
{
	for (int i = 0; i < 3; i++)
	{
		ParticleForce[i] = NewForce[i];
	}
}
void Particle::SetPosition(double NewPosition[3])
{
	for (int i = 0; i < 3; i++)
	{
		ParticlePosition[i] = NewPosition[i];
	}
}
void Particle::SetValues(double Mass, double Position[3])
{
	ParticleMass = Mass;
	for (int i = 0; i < 3; i++)
	{
		ParticlePosition[i] = Position[i];
	}
}