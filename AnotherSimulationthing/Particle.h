#pragma once
class Particle
{
public:
	// Constructors

	Particle();
	Particle(double Mass, double FirstPosition[3]);


	// Information Retrievers
	double GetMass();
	double GetForce(int Component);
	double GetPosition(int Component);



	// Value Setters
	void SetForce(double newForce[3]);
	void SetPosition(double NewPosition[3]);
	void SetValues(double Mass, double Position[3]);


private:
	double ParticleMass;
	double ParticlePosition[3];
	double ParticleForce[3];
};
