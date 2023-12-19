#ifndef DIFFUSIONSIMULATOR_h
#define DIFFUSIONSIMULATOR_h

#include "Simulator.h"
#include "vectorbase.h"


#define EXPLICIT_SOLVER_2D 0
#define IMPLICIT_SOLVER_2D 1


class Grid {
public:
	vector<vector<float>> grid;

	Grid() {
		
	}

	Grid::Grid(int width, int height) {
		for (int i = 0; i < width; i++) {
			for (int j = 0; i < height; i++)
				grid.at(i).push_back(0);  // j mal 
		}
	}

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
	void diffuseTemperatureExplicit(float timeStep);
	void diffuseTemperatureImplicit();

	void setup2DDiffusion();

private:
	// Attributes
	Vec3  m_vfMovableObjectPos;
	Vec3  m_vfMovableObjectFinalPos;
	Vec3  m_vfRotate;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	Grid grid;


	float alpha;
	int gridWidth;
	int gridHeight;
	int deltaX;
	int deltaY;

};

#endif