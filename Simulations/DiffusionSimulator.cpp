#include "DiffusionSimulator.h"
#include "pcgsolver.h"
using namespace std;


DiffusionSimulator::DiffusionSimulator()
{
	m_iTestCase = 0;
	m_vfMovableObjectPos = Vec3();
	m_vfMovableObjectFinalPos = Vec3();
	m_vfRotate = Vec3();

	m_fAlpha = 0.2;					// Thermal diffusivity
	m_fGridScale = 0.1;
	m_iGridRows = m_iGridCols = 100;
	m_fBoundaryTemperature = 0;

	T.resize(m_iGridRows, m_iGridCols, m_fGridScale);
	nextT.resize(m_iGridRows, m_iGridCols, m_fGridScale);
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
	case EXPLICIT_SOLVER:
	case IMPLICIT_SOLVER:
		TwAddVarRW(DUC->g_pTweakBar, "Grid rows", TW_TYPE_INT32, &m_iGridRows, "min=1");
		TwAddVarRW(DUC->g_pTweakBar, "Grid cols", TW_TYPE_INT32, &m_iGridCols, "min=1");
		TwAddVarRW(DUC->g_pTweakBar, "Grid scale", TW_TYPE_FLOAT, &m_fGridScale, "min=0.005 max=0.1 step=0.001");

		TwAddVarRW(DUC->g_pTweakBar, "Thermal diffusivity (alpha)", TW_TYPE_FLOAT, &m_fAlpha, "min=0 max=2.5 step=0.01");
		TwAddVarRW(DUC->g_pTweakBar, "Boundary temperature", TW_TYPE_FLOAT, &m_fBoundaryTemperature, "min=-100 max=1000 step=1");
		break;

	default:
		break;
	}
}

void DiffusionSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	m_vfMovableObjectPos = Vec3(0, 0, 0);
	m_vfRotate = Vec3(0, 0, 0);

	switch (m_iTestCase)
	{
	case EXPLICIT_SOLVER:
		cout << "Explicit solver!\n";
		setupDemo1();
		break;

	case IMPLICIT_SOLVER:
		cout << "Implicit solver!\n";
		setupDemo1();
		break;

	default:
		cout << "Empty Test!\n";
		break;
	}
}

void DiffusionSimulator::diffuseTemperatureExplicit(float timestep) {
	double r = m_fAlpha * timestep / (T.getSize() * T.getSize());

	for (int i = 0; i < T.getRows(); i++) {
		for (int j = 0; j < T.getCols(); j++) {
			// Get the value of T, and use the m_fBoundaryTemperature
			// when the indices are outside from the grid:
			float tmp = T.get(i + 1, j, m_fBoundaryTemperature)
				+ T.get(i - 1, j, m_fBoundaryTemperature)
				+ T.get(i, j + 1, m_fBoundaryTemperature)
				+ T.get(i, j - 1, m_fBoundaryTemperature)
				- 4 * T.get(i, j);

			nextT.set(i, j, T.get(i, j) + r * tmp);
		}
	}

	// Update the grid with the new values:
	T.swapValues(&nextT);
}


void DiffusionSimulator::diffuseTemperatureImplicit(float timestep) {
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

void DiffusionSimulator::setupDemo1()
{
	T.resize(m_iGridRows, m_iGridCols, m_fGridScale);
	nextT.resize(m_iGridRows, m_iGridCols, m_fGridScale);

	int rowMin = 4 * T.getRows() / 10;
	int rowMax = 6 * T.getRows() / 10;
	int colMin = 4 * T.getCols() / 10;
	int colMax = 6 * T.getCols() / 10;

	for(int row = 0; row < T.getRows(); row++) {
		for (int col = 0; col < T.getCols(); col++) {
			if ((row < rowMin || row > rowMax) && colMin <= col && col <= colMax)
				T.set(row, col, 500);
			else
				T.set(row, col, 0);
		}
	}
}



void DiffusionSimulator::simulateTimestep(float timestep)
{
	// update current setup for each frame
	switch (m_iTestCase)
	{
	case EXPLICIT_SOLVER:
		// feel free to change the signature of this function
		diffuseTemperatureExplicit(timestep);
		break;
	case IMPLICIT_SOLVER:
		// feel free to change the signature of this function
		diffuseTemperatureImplicit(timestep);
		break;
	}
}

void DiffusionSimulator::drawObjects()
{
	T.draw(DUC);
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
