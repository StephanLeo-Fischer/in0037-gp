#ifndef DIFFUSIONSIMULATOR_h
#define DIFFUSIONSIMULATOR_h

#include "Simulator.h"
#include "vectorbase.h"
#include "pcgsolver.h"

class Grid {
	// to be implemented
public:
	// Construtors
	Grid(int nx, int ny, int nz);

	// Functions
	void setT(std::vector<Real>& x);
	void setValue(int i, int j, int k, Real value);
	Real getValue(int i, int j, int k);

private:
	// Attributes
	int nx, ny, nz;
	Real *T;
};



class DiffusionSimulator:public Simulator{
public:
	// Construtors
	DiffusionSimulator();

	// Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void simulateTimestep(float timeStep);
	void externalForcesCalculations(float timeElapsed) {};
	void onClick(int x, int y);
	void onMouse(int x, int y);
	// Specific Functions
	void drawObjects();

	// Feel free to change the signature of these functions, add arguments, etc.
	void diffuseTemperatureExplicit(int nx, int ny, int nz, Grid T, double alpha, double timestep);
	void diffuseTemperatureImplicit(int nx, int ny, int nz, Grid T, double alpha, double timestep);

private:
	// Attributes
	Vec3  m_vfMovableObjectPos;
	Vec3  m_vfMovableObjectFinalPos;
	Vec3  m_vfRotate;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	int nx, ny, nz;
	float alpha;
	float timestep;
	Grid T;

	// Helper Functions
	void setupA(int nx, int ny, int nz, SparseMatrix<Real>& A, double factor);
	void setupB(int nx, int ny, int nz, Grid T, std::vector<Real>& b);
};

#endif