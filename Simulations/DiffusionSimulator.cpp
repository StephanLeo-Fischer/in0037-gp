#pragma once
#include "DiffusionSimulator.h"
#include "pcgsolver.h"
using namespace std;

//##################################################################################
//	Grid class
//##################################################################################

Grid::Grid(int nx = 5, int ny = 5, int nz = 5) {
	this->nx = nx;
	this->ny = ny;
	this->nz = nz;
	T = (Real*)malloc(nx * ny * nz * sizeof(Real));
}

void Grid::setT(std::vector<Real>& x) {
	for (int idx_x = 0; idx_x < nx; idx_x++) {
		for (int idx_j = 0; idx_j < ny; idx_j++) {
			for (int idx_z = 0; idx_z < nz; idx_z++) {
				if (
					idx_x == 0 || 
					idx_x == nx - 1 || 
					idx_j == 0 || 
					idx_j == ny - 1 || 
					(nz > 1 && (idx_z == 0 || idx_z == nz - 1)))
					setValue(idx_x, idx_j, idx_z, 0.0); //set boundary condition
				else {
					setValue(idx_x, idx_j, idx_z, x.at(idx_x * ny * nz + idx_j * nz + idx_z));	
				}
			}
		}
	}
}


void Grid::setValue(int i, int j, int k, Real value) 
{
	T[i * ny * nz + j * nz + k] = value;
}

Real Grid::getValue(int i, int j, int k) 
{
	return T[i * ny * nz + j * nz + k];
}

//##################################################################################
//##################################################################################
//	DiffusionSimulator
//##################################################################################
//##################################################################################


//##################################################################################
//	Constructor/Destructor
//##################################################################################


DiffusionSimulator::DiffusionSimulator()
{
	m_iTestCase = 0;
	m_vfMovableObjectPos = Vec3();
	m_vfMovableObjectFinalPos = Vec3();
	m_vfRotate = Vec3();
	// rest to be implemented
	nx = ny = 16; // squared
	nz = 1;
	alpha = 0.5;
	timestep = 0.01;

	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}


//##################################################################################
//	Functions
//##################################################################################

const char * DiffusionSimulator::getTestCasesStr(){
	return "Explicit_solver, Implicit_solver";
}

void DiffusionSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
	// to be implemented
	TwAddVarRW(DUC->g_pTweakBar, "nx", TW_TYPE_INT32, &nx, "min=1");
	TwAddVarRW(DUC->g_pTweakBar, "ny", TW_TYPE_INT32, &ny, "min=1");
	TwAddVarRW(DUC->g_pTweakBar, "nz", TW_TYPE_INT32, &nz, "min=1");
	TwAddVarRW(DUC->g_pTweakBar, "Alpha", TW_TYPE_FLOAT, &alpha, "min=0.0");
}

void DiffusionSimulator::reset(){
		m_mouse.x = m_mouse.y = 0;
		m_trackmouse.x = m_trackmouse.y = 0;
		m_oldtrackmouse.x = m_oldtrackmouse.y = 0;

}

void DiffusionSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	drawObjects();
}

void DiffusionSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	m_vfMovableObjectPos = Vec3(0, 0, 0);
	m_vfRotate = Vec3(0, 0, 0);
	//
	// to be implemented
	//
	T = Grid(nx, ny, nz);
	for (int i = 0; i < nx; i++) {
		for (int j = 0; j < ny; j++) {
			for (int k = 0; k < nz; k++) {
				T.setValue(i, j, k, Real(2 * (rand() % 100 / (double)101) - 1));
			}
		}
	}
	switch (m_iTestCase)
	{
	case 0:
		cout << "Explicit solver!\n";
		break;
	case 1:
		cout << "Implicit solver!\n";
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

void DiffusionSimulator::simulateTimestep(float timeStep)
{
	// update current setup for each frame
	switch (m_iTestCase)
	{
	case 0:
		// feel free to change the signature of this function
		diffuseTemperatureExplicit(nx, ny, nz, T, alpha, timestep);
		break;
	case 1:
		// feel free to change the signature of this function
		diffuseTemperatureImplicit(nx, ny, nz, T, alpha, timestep);
		break;
	}
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

// ##############################################################
//	Specific Functions
// ##############################################################

void DiffusionSimulator::drawObjects()
{
	// to be implemented
	//visualization
	int startindex = nz > 2 ? 1 : 0;
	int offset = nz > 2 ? -1 : 0;
	for (int idx_x = startindex; idx_x < nx + offset; idx_x++) {
		for (int idx_y = startindex; idx_y < ny + offset; idx_y++) {
			for (int idx_z = startindex; idx_z < nz + offset; idx_z++) {
				/* For visualization, you could use spheres to represent every point position and use the color of the
					sphere to represent the temperature of this point position (red for negative value and white for
					positive value). Then, we can see how the equation is solved by observing color change.
					*/
				DUC->setUpLighting(
					abs(T.getValue(idx_x, idx_y, idx_z)) * Vec3(            // emmisive color															
						1,									            // red
						T.getValue(idx_x, idx_y, idx_z) > 0 ? 1 : 0,	// green
						T.getValue(idx_x, idx_y, idx_z) > 0 ? 1 : 0),   // blue
					0.4 * Vec3(1, 1, 1),									// specular color
					10,														// specular power
					abs(T.getValue(idx_x, idx_y, idx_z)) * Vec3(1, 1, 1));	// diffuse color
				DUC->drawSphere(
					0.1 * Vec3(idx_x - int(nx / 2), idx_y - int(ny / 2), idx_z - int(nz / 2)),	// position
					0.1 * Vec3(1, 1, 1)															// scale
				);
			}
		}
	}
}

// ##############################################################
//	Diffuse Temperature
// ##############################################################

void DiffusionSimulator::diffuseTemperatureExplicit(int nx, int ny, int nz, Grid T, double alpha,double timestep) {
// to be implemented
	int calcfactor;
	T = Grid(nx, ny, nz);
	for (int idx_x = 0; idx_x < nx; idx_x++) {
		for (int idx_y = 0; idx_y < ny; idx_y++) {
			for (int idx_z = 0; idx_z < nz; idx_z++) {
				/* Solve the discretized heat equation with an explicit Euler. Implement the method
					diffuseTemperatureExplicit(…) in DiffusionSimulator.cpp. Call this function where all the magic
					happens and ensure the temperature is always zero in boundary cells.
					*/
				if (idx_x > 0 && 
					idx_x < nx - 1 && 
					idx_y > 0 && 
					idx_y < ny - 1 && 
					(nz > 1 ? (idx_z > 0 && idx_z < nz - 1) : 1)) 
				{
					T.setValue(idx_x, idx_y, idx_z, 
						alpha * timestep * (
							T.getValue(idx_x - 1, idx_y, idx_z) +	// 2d neigbour
							T.getValue(idx_x + 1, idx_y, idx_z) +	// 2d neigbour
							T.getValue(idx_x, idx_y - 1, idx_z) +	// 2d neigbour
							T.getValue(idx_x, idx_y + 1, idx_z) +	// 2d neigbour
							(nz > 1 ? T.getValue(idx_x, idx_y, idx_z - 1) + T.getValue(idx_x, idx_y, idx_z + 1) : 0) -
							(nz > 1 ? 6 : 4) * T.getValue(idx_x, idx_y, idx_z)
							) + T.getValue(idx_x, idx_y, idx_z));
				}
				else
					// boundary condition
				{
					T.setValue(idx_x, idx_y, idx_z, 0);
				}
			}
		}
	}
}

void DiffusionSimulator::diffuseTemperatureImplicit(int nx, int ny, int nz, Grid T, double alpha, double timestep) {
	// solve A T = b

	// This is just an example to show how to work with the PCG solver,
	/*const int nx = 5;
	const int ny = 5;
	const int nz = 5;*/
	const int N = nx * ny * nz;

	SparseMatrix<Real> A(N);
	std::vector<Real> b(N);

	// This is the part where you have to assemble the system matrix A and the right-hand side b!
	setupA(nx, ny, nz, A, alpha * timestep);
	setupB(nx, ny, nz, T, b);

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
	T.setT(x);
}

// ##############################################################
//	Helper Functions
// ##############################################################

void DiffusionSimulator::setupA(int nx, int ny, int nz, SparseMatrix<Real>& A, double factor) {
	for (int x_idx = 0; x_idx < nx; x_idx++) 
	{
		for (int y_idx = 0; y_idx < ny; y_idx++) 
		{
			for (int z_idx = 0; z_idx < nz; z_idx++) 
			{
				int idx = x_idx*ny*nz + y_idx * nz + z_idx;
				if (
					x_idx == 0 || 
					x_idx == nx - 1 || 
					y_idx == 0 || 
					y_idx == ny - 1 || 
					(nz > 1 && (z_idx == 0 || z_idx == nz - 1))) 
				{
					A.set_element(idx, idx, 1.0);	//set boundary condition
				}
				else 
				{
					A.set_element(idx, idx, 1 + (nz > 1 ? 6 : 4) * factor);
					int mxIdx = nx * ny * nz;
					if (idx - 1 >= 0)
						A.set_element(idx - 1, idx, -factor);		//3d: k-1; 2d: j+1	
					if (idx + 1 <= mxIdx - 1)
						A.set_element(idx + 1, idx, -factor);		//3d: k+1; 2d: j+1
					if (idx - ny*nz >= 0)
						A.set_element(idx - nz * ny, idx, -factor);	//i-1
					if (idx + ny*nz <= mxIdx - 1)
						A.set_element(idx + nz * ny, idx, -factor);	//j+1
					// 3d only
					if (nz > 1 && idx - nz >= 0)
						A.set_element(idx - nz, idx, -factor);
					if (nz > 1 && idx + nz <= mxIdx - 1)
						A.set_element(idx + nz, idx, -factor);
				}
			}
		}
	}
}

void DiffusionSimulator::setupB(int nx, int ny, int nz, Grid T, std::vector<Real>& b) {
	for (int i = 0; i < nx; i++) {
		for (int j = 0; j < ny; j++) {
			for (int k = 0; k < nz; k++) {
				b.at(i * ny * nz + j * nz + k) = T.getValue(i, j, k);
			}
		}
	}
}