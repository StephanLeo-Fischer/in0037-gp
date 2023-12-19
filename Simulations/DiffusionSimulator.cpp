#include "DiffusionSimulator.h"
#include "pcgsolver.h"
using namespace std;


DiffusionSimulator::DiffusionSimulator()
{
	m_iTestCase = 0;
	m_vfMovableObjectPos = Vec3();
	m_vfMovableObjectFinalPos = Vec3();
	m_vfRotate = Vec3();
	// rest to be implemented
}

const char * DiffusionSimulator::getTestCasesStr(){
	return "Explicit_solver, Implicit_solver";
}

void DiffusionSimulator::reset(){
		m_mouse.x = m_mouse.y = 0;
		m_trackmouse.x = m_trackmouse.y = 0;
		m_oldtrackmouse.x = m_oldtrackmouse.y = 0;

}

void DiffusionSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
	switch (m_iTestCase)
	{
	case EXPLICIT_SOLVER_2D:  // fall through, because same parameters
	case IMPLICIT_SOLVER_2D:
		TwAddVarRW(DUC->g_pTweakBar, "Thermal diffusivity (alpha)", TW_TYPE_FLOAT, &alpha, "min=0 step=0.01");
		break;
	}
}

void DiffusionSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	m_vfMovableObjectPos = Vec3(0, 0, 0);
	m_vfRotate = Vec3(0, 0, 0);
	//
	// to be implemented
	//
	switch (m_iTestCase)
	{
	case EXPLICIT_SOLVER_2D:
		cout << "Explicit solver!\n";
		setup2DDiffusion();
		break;
	case IMPLICIT_SOLVER_2D:
		cout << "Implicit solver!\n";
		setup2DDiffusion();
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

void DiffusionSimulator::setup2DDiffusion() {
	gridWidth, gridHeight = 4;

	grid = Grid(gridWidth, gridHeight);
}

void DiffusionSimulator::diffuseTemperatureExplicit(float timeStep) {
	// forward finite diff
	// T^{t+1}_{ij} = alpha(..bigThing..) * delta t + T^t_{ij} 
	for (int i = 0; i < gridWidth; i++) {
		for (int j = 0; j < gridHeight; i++) {
			float bigThing = (grid.grid.at(i+2).at(j) - 2 * grid.grid.at(i + 1).at(j) - grid.grid.at(i).at(j)) / (deltaX * deltaX)  
				+ (grid.grid.at(i).at(j + 2) - 2 * grid.grid.at(i).at(j + 1) - grid.grid.at(i).at(j)) / (deltaY * deltaY);
			float nextValue = alpha * bigThing * timeStep + grid.grid.at(i).at(j);
			grid.grid.at(i).at(j) = nextValue;  // after delta t
		}
	}
}


void DiffusionSimulator::diffuseTemperatureImplicit() {
	// solve A T = b

	// This is just an example to show how to work with the PCG solver,
	const int nx = 5;
	const int ny = 5;
	const int nz = 5;
	const int N = nx * ny * nz;

	SparseMatrix<Real> A(N);
	std::vector<Real> b(N);

	// This is the part where you have to assemble the system matrix A and the right-hand side b!

	// perform solve
	Real pcg_target_residual = 1e-05;
	Real pcg_max_iterations = 1000;
	Real ret_pcg_residual = 1e10;
	int  ret_pcg_iterations = -1;

	SparsePCGSolver<Real> solver;
	solver.set_solver_parameters(pcg_target_residual, pcg_max_iterations, 0.97, 0.25);

	std::vector<Real> x(N);
	for (int j = 0; j < N; ++j) { x[j] = 0.; }

	// preconditioners: 0 off, 1 diagonal, 2 incomplete cholesky
	solver.solve(A, b, x, ret_pcg_residual, ret_pcg_iterations, 0);

	// Final step is to extract the grid temperatures from the solution vector x
	// to be implemented
}



void DiffusionSimulator::simulateTimestep(float timeStep)
{
	// update current setup for each frame
	switch (m_iTestCase)
	{
	case 0:
		// feel free to change the signature of this function
		diffuseTemperatureExplicit(timeStep);
		break;
	case 1:
		// feel free to change the signature of this function
		diffuseTemperatureImplicit();
		break;
	}
}

void DiffusionSimulator::drawObjects()
{
	// to be implemented
	//visualization
}


void DiffusionSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	drawObjects();
}

void DiffusionSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void DiffusionSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}
