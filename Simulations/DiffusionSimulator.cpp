#include "DiffusionSimulator.h"
#include "pcgsolver.h"
using namespace std;

extern Simulator* g_pSimulator;

DiffusionSimulator::DiffusionSimulator()
{
	m_iTestCase = 0;
	m_vfMovableObjectPos = Vec3();
	m_vfMovableObjectFinalPos = Vec3();
	m_vfRotate = Vec3();

	m_fAlpha = 0.2;					// Thermal diffusivity
	m_fGridScale = 0.2;
	m_iGridSizeX = m_iGridSizeY = 16;
	m_iGridSizeZ = 1;

	m_bShowAllSlices = false;
	m_iSliceIndex = 0;

	m_fTopBoundaryTemperature = 0;
	m_fBottomBoundaryTemperature = 0;
	m_fLeftBoundaryTemperature = 0;
	m_fRightBoundaryTemperature = 0;
	m_fForwardBoundaryTemperature = 0;
	m_fBackwardBoundaryTemperature = 0;

	T.resize(m_iGridSizeX, m_iGridSizeY, m_iGridSizeZ, m_fGridScale);
	nextT.resize(m_iGridSizeX, m_iGridSizeY, m_iGridSizeZ, m_fGridScale);
}

const char * DiffusionSimulator::getTestCasesStr(){
	return "Explicit_solver,Implicit_solver,Explicit_solver_3D,Implicit_solver_3D";
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
	case EXPLICIT_SOLVER_2D:
	case IMPLICIT_SOLVER_2D:
		TwAddVarRW(DUC->g_pTweakBar, "Grid size X", TW_TYPE_INT32, &m_iGridSizeX, "min=2");
		TwAddVarRW(DUC->g_pTweakBar, "Grid size Y", TW_TYPE_INT32, &m_iGridSizeY, "min=2");
		TwAddVarRW(DUC->g_pTweakBar, "Grid scale", TW_TYPE_FLOAT, &m_fGridScale, "min=0.005 max=0.5 step=0.001");

		TwAddVarRW(DUC->g_pTweakBar, "Thermal diffusivity (alpha)", TW_TYPE_FLOAT, &m_fAlpha, "min=0 step=0.01");
		TwAddVarRW(DUC->g_pTweakBar, "Left temperature", TW_TYPE_FLOAT, &m_fLeftBoundaryTemperature, "min=-100 max=1000 step=100");
		TwAddVarRW(DUC->g_pTweakBar, "Right temperature", TW_TYPE_FLOAT, &m_fRightBoundaryTemperature, "min=-100 max=1000 step=100");
		TwAddVarRW(DUC->g_pTweakBar, "Forward temperature", TW_TYPE_FLOAT, &m_fForwardBoundaryTemperature, "min=-100 max=1000 step=100");
		TwAddVarRW(DUC->g_pTweakBar, "Backward temperature", TW_TYPE_FLOAT, &m_fBackwardBoundaryTemperature, "min=-100 max=1000 step=100");
		break;

	case EXPLICIT_SOLVER_3D:
	case IMPLICIT_SOLVER_3D:
		TwAddVarRW(DUC->g_pTweakBar, "Grid size X", TW_TYPE_INT32, &m_iGridSizeX, "min=2");
		TwAddVarRW(DUC->g_pTweakBar, "Grid size Y", TW_TYPE_INT32, &m_iGridSizeY, "min=2");
		TwAddVarRW(DUC->g_pTweakBar, "Grid size Z", TW_TYPE_INT32, &m_iGridSizeZ, "min=2");
		TwAddVarRW(DUC->g_pTweakBar, "Grid scale", TW_TYPE_FLOAT, &m_fGridScale, "min=0.005 max=0.5 step=0.001");

		TwAddVarRW(DUC->g_pTweakBar, "Thermal diffusivity (alpha)", TW_TYPE_FLOAT, &m_fAlpha, "min=0 step=0.01");

		TwAddVarRW(DUC->g_pTweakBar, "Show all", TW_TYPE_BOOLCPP, &m_bShowAllSlices, "");
		TwAddButton(DUC->g_pTweakBar, "Next slice", [](void* s) { ((DiffusionSimulator*)g_pSimulator)->nextSlice(); }, nullptr, "");
		
		TwAddVarRW(DUC->g_pTweakBar, "Left temperature", TW_TYPE_FLOAT, &m_fLeftBoundaryTemperature, "min=-100 max=1000 step=100");
		TwAddVarRW(DUC->g_pTweakBar, "Right temperature", TW_TYPE_FLOAT, &m_fRightBoundaryTemperature, "min=-100 max=1000 step=100");
		TwAddVarRW(DUC->g_pTweakBar, "Forward temperature", TW_TYPE_FLOAT, &m_fForwardBoundaryTemperature, "min=-100 max=1000 step=100");
		TwAddVarRW(DUC->g_pTweakBar, "Backward temperature", TW_TYPE_FLOAT, &m_fBackwardBoundaryTemperature, "min=-100 max=1000 step=100");
		TwAddVarRW(DUC->g_pTweakBar, "Top temperature", TW_TYPE_FLOAT, &m_fTopBoundaryTemperature, "min=-100 max=1000 step=100");
		TwAddVarRW(DUC->g_pTweakBar, "Bottom temperature", TW_TYPE_FLOAT, &m_fBottomBoundaryTemperature, "min=-100 max=1000 step=100");
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

	m_iSliceIndex = 0;

	switch (m_iTestCase)
	{
	case EXPLICIT_SOLVER_2D:
		cout << "Explicit solver 2D!\n";
		m_iGridSizeZ = 1;
		setupDemo1();
		break;

	case IMPLICIT_SOLVER_2D:
		cout << "Implicit solver 2D!\n";
		m_iGridSizeZ = 1;
		setupDemo1();
		break;

	case EXPLICIT_SOLVER_3D:
		cout << "Explicit solver 3D!\n";
		m_iGridSizeZ = max(2, m_iGridSizeZ);	// In 3D, m_iGridSizeZ should be at least 2
		setupDemo1();
		break;

	case IMPLICIT_SOLVER_3D:
		cout << "Implicit solver 3D!\n";
		m_iGridSizeZ = max(2, m_iGridSizeZ);	// In 3D, m_iGridSizeZ should be at least 2
		setupDemo1();
		break;

	default:
		cout << "Empty Test!\n";
		break;
	}
}

void DiffusionSimulator::diffuseTemperatureExplicit2D(float timestep) {
	const double r = m_fAlpha * timestep / (T.getStep() * T.getStep());

	for (int x = 0; x < nextT.getSizeX(); x++) {
		for (int y = 0; y < nextT.getSizeY(); y++) {

			// Define boundary temperatures:
			if (nextT.isBoundaryCell(x, y, 0))
				nextT.set(x, y, 0, getBoundaryTemperature(x, y, 0));
			
			// Use the heat equation to compute the other parameters:
			else {
				float tmp = T.get(x + 1, y, 0)
					+ T.get(x - 1, y, 0)
					+ T.get(x, y + 1, 0)
					+ T.get(x, y - 1, 0)
					- 4 * T.get(x, y, 0);

				nextT.set(x, y, 0, T.get(x, y, 0) + r * tmp);
			}			
		}
	}

	// Update the grid with the new values:
	T.swapValues(&nextT);
}

void DiffusionSimulator::diffuseTemperatureExplicit3D(float timestep) {
	const double r = m_fAlpha * timestep / (T.getStep() * T.getStep());

	for (int x = 0; x < nextT.getSizeX(); x++) {
		for (int y = 0; y < nextT.getSizeY(); y++) {
			for (int z = 0; z < nextT.getSizeZ(); z++) {

				// Define boundary temperatures:
				if (nextT.isBoundaryCell(x, y, z))
					nextT.set(x, y, z, getBoundaryTemperature(x, y, z));

				// Use the heat equation to compute the other parameters:
				else {
					float tmp = T.get(x+1, y, z)
						+ T.get(x-1, y, z)
						+ T.get(x, y+1, z)
						+ T.get(x, y-1, z)
						+ T.get(x, y, z+1)
						+ T.get(x, y, z-1)
						- 6 * T.get(x, y, z);

					nextT.set(x, y, z, T.get(x, y, z) + r * tmp);
				}
			}
		}
	}

	// Update the grid with the new values:
	T.swapValues(&nextT);
}

void DiffusionSimulator::diffuseTemperatureImplicit(float timestep) {
	// If the grid is in 3D, the laplacian is computed differently:
	const bool grid3D = T.getSizeZ() > 1;

	const int N = T.getDataSize();
	const double r = m_fAlpha * timestep / (T.getStep() * T.getStep());

	SparseMatrix<Real> A(N);
	std::vector<Real> b(N);

	// Set each row of the matrix A:
	int x, y, z;
	for (int i = 0; i < N; i++) {
		T.indexToPosition(i, &x, &y, &z);

		if (T.isBoundaryCell(x, y, z)) {
			A.set_element(i, T.positionToIndex(x, y, z), 1);
			T.set(x, y, z, getBoundaryTemperature(x, y, z));
		}
		else {
			if (grid3D) {
				A.set_element(i, T.positionToIndex(x, y, z), 1 + 6 * r);

				A.set_element(i, T.positionToIndex(x, y, z + 1), -r);
				A.set_element(i, T.positionToIndex(x, y, z - 1), -r);
			}
			else {
				A.set_element(i, T.positionToIndex(x, y, z), 1 + 4 * r);
			}
			
			A.set_element(i, T.positionToIndex(x+1, y, z), -r);
			A.set_element(i, T.positionToIndex(x-1, y, z), -r);
			A.set_element(i, T.positionToIndex(x, y+1, z), -r);
			A.set_element(i, T.positionToIndex(x, y-1, z), -r);
		}
	}

	// Set b:
	for (int i = 0; i < N; i++)
		b[i] = T.get(i);

	// perform solve
	Real pcg_target_residual = 1e-05;
	Real pcg_max_iterations = 1000;
	Real ret_pcg_residual = 1e10;
	int  ret_pcg_iterations = -1;

	SparsePCGSolver<Real> solver;
	solver.set_solver_parameters(pcg_target_residual, pcg_max_iterations, 0.97, 0.25);

	std::vector<Real> X(N);
	for (int j = 0; j < N; ++j) { X[j] = 0.; }

	// preconditioners: 0 off, 1 diagonal, 2 incomplete cholesky
	solver.solve(A, b, X, ret_pcg_residual, ret_pcg_iterations, 0);

	// Final step is to extract the grid temperatures from the solution vector x
	T.set(X);
}

void DiffusionSimulator::setupDemo1()
{
	T.resize(m_iGridSizeX, m_iGridSizeY, m_iGridSizeZ, m_fGridScale);
	nextT.resize(m_iGridSizeX, m_iGridSizeY, m_iGridSizeZ, m_fGridScale);

	// Warning, be sure to initialize all the grid !!!
	T.set(0);

	int xMin = 4 * T.getSizeX() / 10;
	int xMax = 6 * T.getSizeX() / 10;
	int yMin = 4 * T.getSizeY() / 10;
	int yMax = 6 * T.getSizeY() / 10;

	for(int x = 0; x < T.getSizeX(); x++) {
		for (int y = 0; y < T.getSizeY(); y++) {
			if ((x < xMin || x > xMax) && yMin <= y && y <= yMax)
				T.set(x, y, 0, 1000);
			else
				T.set(x, y, 0, 0);
		}
	}
}

void DiffusionSimulator::simulateTimestep(float timestep)
{
	// update current setup for each frame
	switch (m_iTestCase)
	{
	case EXPLICIT_SOLVER_2D:
		diffuseTemperatureExplicit2D(timestep);
		break;
	case IMPLICIT_SOLVER_2D:
		diffuseTemperatureImplicit(timestep);
		break;
	case EXPLICIT_SOLVER_3D:
		diffuseTemperatureExplicit3D(timestep);
		break;
	case IMPLICIT_SOLVER_3D:
		diffuseTemperatureImplicit(timestep);
		break;
	default:
		break;
	}
}

void DiffusionSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	if (m_bShowAllSlices)
		T.drawAll(DUC);
	else
		T.drawSlice(DUC, m_iSliceIndex);

	Sleep(1);
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

void DiffusionSimulator::nextSlice() {
	m_iSliceIndex = (m_iSliceIndex + 1) % T.getSizeZ();
}

float DiffusionSimulator::getBoundaryTemperature(int x, int y, int z) {

	if (x == 0)
		return m_fLeftBoundaryTemperature;
	if (x == T.getSizeX()-1)
		return m_fRightBoundaryTemperature;
	if (y == 0)
		return m_fForwardBoundaryTemperature;
	if (y == T.getSizeY() - 1)
		return m_fBackwardBoundaryTemperature;
	if (z == 0)
		return m_fBottomBoundaryTemperature;

	return m_fTopBoundaryTemperature;
}