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

	T.initValues();
	diffusionConst = 1.0;
	T.size = 5;
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
	// to be implemented

	TwAddVarRW(DUC->g_pTweakBar, "Grid Size", TW_TYPE_FLOAT, &T.size, "min=1 step=0.1");
	TwAddVarRW(DUC->g_pTweakBar, "Grid Scale n", TW_TYPE_INT32, &T.newN, "min=3 step=1");
	TwAddVarRW(DUC->g_pTweakBar, "Grid Scale m", TW_TYPE_INT32, &T.newM, "min=3 step=1");
	TwAddVarRW(DUC->g_pTweakBar, "Diffusion Constant", TW_TYPE_FLOAT, &diffusionConst, "min=0.1 step=0.1 max=10.0");
}

void DiffusionSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	//
	// to be implemented
	//

	T.initValues();

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

void DiffusionSimulator::diffuseTemperatureExplicit(float timeStep) {
	T.initTemp();
	for (int i = 0; i < T.n; i++) {
		for (int j = 0; j < T.m; j++) {
			if (!T.valIsInRange(i, j)) {
				cout << " VALUE NOT IN RANGE AT i=" << i << ", j=" << j << endl;
				break;
			}
			if (!T.tempIsInRange(i, j)) {
				cout << " TEMP NOT IN RANGE AT i=" << i << ", j=" << j << endl;
				break;
			}

			if(T.isBoundary(i, j)) {
				T.setTemp(i, j, 0);
			} else {

				if (!T.valIsInRange(i+1, j)) {
					cout << " EXTENDED VALUE NOT IN RANGE AT i=" << i+1 << ", j=" << j << endl;
					break;
				}
				if (!T.valIsInRange(i-1, j)) {
					cout << " EXTENDED VALUE NOT IN RANGE AT i=" << i-1 << ", j=" << j << endl;
					break;
				}
				if (!T.valIsInRange(i, j+1)) {
					cout << " EXTENDED VALUE NOT IN RANGE AT i=" << i << ", j=" << j+1 << endl;
					break;
				}
				if (!T.valIsInRange(i, j-1)) {
					cout << " EXTENDED VALUE NOT IN RANGE AT i=" << i << ", j=" << j-1 << endl;
					break;
				}



				float value = (T.getValue(i + 1, j) - 2.0 * T.getValue(i, j) + T.getValue(i - 1, j))/(T.xStep() * T.xStep());
				value += (T.getValue(i, j + 1) - 2.0 * T.getValue(i, j) + T.getValue(i, j - 1))/(T.yStep() * T.yStep());
				value *= diffusionConst * timeStep;

				T.setTemp(i, j, T.getTemp(i, j) + value);
			}
		}
	}

	for (int i = 0; i < T.n; i++) {
		for (int j = 0; j < T.m; j++) {
			T.setValue(i, j, T.getTemp(i, j));
		}
	}
}


void DiffusionSimulator::diffuseTemperatureImplicit(float timeStep) {
	// solve A T = b

	SparseMatrix<Real> A(T.n * T.m);
	std::vector<Real> b(T.n * T.m);

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
	if (T.newN != T.n || T.newM != T.m) {
		T.setSize(T.newN, T.newM);
	}
	// update current setup for each frame
	switch (m_iTestCase)
	{
	case 0:
		// feel free to change the signature of this function
		diffuseTemperatureExplicit(timeStep);
		break;
	case 1:
		// feel free to change the signature of this function
		diffuseTemperatureImplicit(timeStep);
		break;
	}
}

void DiffusionSimulator::drawObjects()
{
	// to be implemented
	//visualization

	for (int i = 0; i < T.n; i++) {
		for (int j = 0; j < T.m; j++) {

			if (!T.valIsInRange(i, j)) {
				cout << " VALUE NOT IN RANGE AT i=" << i << ", j=" << j << endl;
				break;
			}
			DUC->drawSphere(Vec3(i*T.xStep() - T.size/2, T.getValue(i, j), j*T.yStep() - T.size/2).toDirectXVector(), Vec3(0.1).toDirectXVector());
		}
	}
}


void DiffusionSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	DUC->setUpLighting(Vec3(), Vec3(0.5), 0.5, Vec3(0.5));
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

void Grid::setSize(int n, int m)
{
	values.resize(n);
	for (vector<float>& v : values) {
		v.resize(m, 0);
	}

	this->n = n;
	this->m = m;
}

bool Grid::isBoundary(int i, int j)
{
	return i == 0 || i == n-1 || j == 0 || j == m-1;
}

float Grid::getValue(int i, int j)
{
	return values.at(i).at(j);
}

void Grid::setValue(int i, int j, float value)
{
	values.at(i).at(j) = value;
}

float Grid::getTemp(int i, int j)
{
	return temp.at(i).at(j);
}

void Grid::setTemp(int i, int j, float value)
{
	temp.at(i).at(j) = value;
}

void Grid::initValues()
{
	values = vector<vector<float>>(16);
	for (auto& v : values) {
		v = vector<float>(16);
	}
	newN = newM = n = m = 16;

	for (int i = 0; i < 8; i++) {
		for (int j = 0; j < 8; j++) {
			setValue(i, j, 5.0);
		}
	}
}

void Grid::initTemp()
{
	temp = values;
}

bool Grid::valIsInRange(int i, int j)
{
	return i >= 0 && i < values.size() && j >= 0 && j < values.at(i).size();
}

bool Grid::tempIsInRange(int i, int j)
{
	return i >= 0 && i < temp.size() && j >= 0 && j < temp.at(i).size();
}

float Grid::xStep()
{
	return size/(float)n;
}

float Grid::yStep()
{
	return size/(float)m;
}
