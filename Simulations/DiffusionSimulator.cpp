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
	gridWidth = 4;
	gridHeight = 4;

	grid = Grid(gridWidth, gridHeight);
}

void DiffusionSimulator::diffuseTemperatureExplicit(float timeStep) {
	// forward finite diff  not working, because you need 2 ahead
	// T^{t+1}_{ij} = alpha(..bigThing..) * delta t + T^t_{ij} 
	// temperature = 0 in boundary cells, thats why from 1 to width-1
	//for (int i = 1; i < gridWidth - 1; i++) {
	//	for (int j = 1; j < gridHeight - 1; i++) {
	//		float bigThing = (grid.grid.at(i+2).at(j) - 2 * grid.grid.at(i + 1).at(j) + grid.grid.at(i).at(j)) / (deltaX * deltaX)  
	//			+ (grid.grid.at(i).at(j + 2) - 2 * grid.grid.at(i).at(j + 1) + grid.grid.at(i).at(j)) / (deltaY * deltaY);
	//		float nextValue = alpha * bigThing * timeStep + grid.grid.at(i).at(j);
	//		grid.grid.at(i).at(j) = nextValue;  // after delta t
	//	}
	//}

	// central diff
	for (int i = 2; i < gridWidth - 2; i++) {
		for (int j = 2; j < gridHeight - 2; i++) {
			float bigThing = (grid.grid.at(i + 1).at(j) - 2 * grid.grid.at(i).at(j) + grid.grid.at(i -1).at(j)) / (deltaX * deltaX)
				+ (grid.grid.at(i).at(j + 1) - 2 * grid.grid.at(i).at(j) + grid.grid.at(i).at(j - 1)) / (deltaY * deltaY);
			float nextValue = alpha * bigThing * 2 * timeStep + grid.grid.at(i).at(j);
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
	//cout << "height" << gridHeight << "width" << gridWidth << endl;
	//visualization
	for (int idx_x = 0; idx_x < gridWidth; idx_x++) {
		for (int idx_y = 0; idx_y < gridHeight; idx_y++) {
			
			/* For visualization, you could use spheres to represent every point position and use the color of the
				sphere to represent the temperature of this point position (red for negative value and white for
				positive value). Then, we can see how the equation is solved by observing color change.
				*/
			DUC->setUpLighting(
				abs(grid.grid.at(idx_x).at(idx_y)) * Vec3(            // emmisive color															
					1,									            // red
					grid.grid.at(idx_x).at(idx_y) > 0 ? 1 : 0,	// green
					grid.grid.at(idx_x).at(idx_y) > 0 ? 1 : 0),   // blue
				0.4 * Vec3(1, 1, 1),									// specular color
				10,														// specular power
				abs(grid.grid.at(idx_x).at(idx_y)) * Vec3(1, 1, 1));	// diffuse color
			DUC->drawSphere(
				0.15 * Vec3(idx_x - int(gridWidth / 2), idx_y - int(gridHeight / 2), 0),	// position
				0.1 * Vec3(1, 1, 1)															// scale
			);
		}
	}
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
