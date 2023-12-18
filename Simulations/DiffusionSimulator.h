#ifndef DIFFUSIONSIMULATOR_h
#define DIFFUSIONSIMULATOR_h

#include "Simulator.h"
#include "vectorbase.h"

#define EXPLICIT_SOLVER_2D 0
#define IMPLICIT_SOLVER_2D 1
#define EXPLICIT_SOLVER_3D 2
#define IMPLICIT_SOLVER_3D 3


class Grid {
public:
	Grid() : m_sizeX(0), m_sizeY(0), m_sizeZ(0), m_dataSize(0), m_step(0), m_data(NULL) {
		// Empty constructor. The object shouldn't be used in this case
	}

	Grid(int sizeX, int sizeY, int sizeZ, float step) :
		m_sizeX(sizeX), m_sizeY(sizeY), m_sizeZ(sizeZ), 
		m_dataSize(sizeX * sizeY * sizeZ),
		m_step(step), 
		m_data(new float[m_dataSize]) {}

	void drawSlice(DrawingUtilitiesClass* DUC, int slice) {
		if (slice >= 0 && slice < m_sizeZ) {
			DUC->beginLine();
			drawGrid2D(DUC, slice, m_sizeZ == 1);	// Draw a heightMap only if we are in 2D (m_sizeZ == 1)
			DUC->endLine();
		}
	}

	// Draw all slices of the grid:
	void drawAll(DrawingUtilitiesClass* DUC) {
		DUC->beginLine();

		for (int z = 0; z < m_sizeZ; z++) {
			drawGrid2D(DUC, z, m_sizeZ == 1);		// Draw a heightMap only if we are in 2D (m_sizeZ == 1)
		}

		DUC->endLine();
	}

	inline float getStep() const {
		return m_step;
	}

	inline int getSizeX() const {
		return m_sizeX;
	}

	inline int getSizeY() const {
		return m_sizeY;
	}

	inline int getSizeZ() const {
		return m_sizeZ;
	}

	inline int getDataSize() {
		return m_dataSize;
	}

	void resize(int sizeX, int sizeY, int sizeZ, float step) {
		int newDataLength = sizeX * sizeY * sizeZ;

		if (newDataLength != m_dataSize) {
			delete[] m_data;
			m_data = new float[newDataLength];
		}
		
		m_sizeX = sizeX;
		m_sizeY = sizeY;
		m_sizeZ = sizeZ;
		m_dataSize = newDataLength;
		m_step = step;
	}

	void swapValues(Grid* other) {
		if (other->m_sizeX != m_sizeX || other->m_sizeY != m_sizeY || other->m_sizeZ != m_sizeZ)
			throw std::runtime_error("Cannot swap grid with different dimensions");

		float* tmp = other->m_data;
		other->m_data = this->m_data;
		this->m_data = tmp;
	}

	void set(float value) {
		for (int i = 0; i < m_dataSize; i++)
			m_data[i] = value;
	}

	void set(std::vector<Real> values) {
		if (values.size() != m_dataSize)
			throw std::runtime_error("Cannot set this vector to this grid: the vector has the wrong size !");

		for (int i = 0; i < m_dataSize; i++)
			m_data[i] = values[i];
	}

	void set(int x, int y, int z, float value) {
		if (x < 0 || x >= m_sizeX || y < 0 || y >= m_sizeY || z < 0 || z >= m_sizeZ)
			throw std::out_of_range("Grid index is out of range");

		m_data[positionToIndex(x, y, z)] = value;
	}

	float get(int index) const {
		if(index < 0 || index >= m_dataSize)
			throw std::out_of_range("Grid index is out of range");

		return m_data[index];
	}

	float get(int x, int y, int z) const {
		if (x < 0 || x >= m_sizeX || y < 0 || y >= m_sizeY || z < 0 || z >= m_sizeZ)
			throw std::out_of_range("Grid index is out of range");

		return m_data[positionToIndex(x, y, z)];
	}

	// Map a position in the grid, to a unique index in the array m_data:
	inline int positionToIndex(int x, int y, int z) const {
		return m_sizeX * (m_sizeY * z + y) + x;
	}

	// Map an index the array m_data to a unique position in the grid:
	void indexToPosition(int index, int * x, int * y, int * z) {
		*z = index / (m_sizeX * m_sizeY);

		index = index % (m_sizeX * m_sizeY);
		*y = index / m_sizeX;
		*x = index % m_sizeX;
	}

	// If the grid is a 2D grid (m_sizeZ == 1), then the boundary is just the squareBoundary.
	// Else, we also check if z is a boundary
	inline bool isBoundaryCell(int x, int y, int z) {
		bool squareBoundary = x == 0 || x + 1 == m_sizeX || y == 0 || y + 1 == m_sizeY;
		
		return squareBoundary || (m_sizeZ != 1 && (z == 0 || z + 1 == m_sizeZ));
	}

	inline ~Grid() {
		delete[] m_data;
	}

private:
	float m_step;			// Space between the points of the grid

	int m_sizeX, m_sizeY, m_sizeZ;

	int m_dataSize;	// m_sizeX * m_sizeY * m_sizeZ
	float* m_data;

	const float minValue = -100;
	const float maxValue = 1000;
	const Vec3 minColor = Vec3(1, 0, 0);	// The color of a point with the minimum value
	const Vec3 maxColor = Vec3(1, 1, 1);	// The color of a point with the maximum value
	const float minHeight = 0;				// The height of a point with the minimum value (for 2D grid)
	const float maxHeight = 10;				// The height of a point with the maximum value (for 2D grid)

	void drawGrid2D(DrawingUtilitiesClass* DUC, int z, bool heightMap) {
		Vec3 prevColor, currentColor;
		Vec3 prevPos, currentPos;

		// X-axis lines:
		for (int y = 0; y < m_sizeY; y++) {
			getPosition(0, y, z, &prevPos, heightMap);
			getColor(0, y, z, &prevColor);

			for (int x = 1; x < m_sizeX; x++) {
				getPosition(x, y, z, &currentPos, heightMap);
				getColor(x, y, z, &currentColor);

				DUC->drawLine(prevPos, prevColor, currentPos, currentColor);
				prevPos = currentPos;
				prevColor = currentColor;
			}
		}

		// Y-axis lines:
		for (int x = 0; x < m_sizeX; x++) {
			getPosition(x, 0, z, &prevPos, heightMap);
			getColor(x, 0, z, &prevColor);

			for (int y = 1; y < m_sizeY; y++) {
				getPosition(x, y, z, &currentPos, heightMap);
				getColor(x, y, z, &currentColor);

				DUC->drawLine(prevPos, prevColor, currentPos, currentColor);
				prevPos = currentPos;
				prevColor = currentColor;
			}
		}
	}

	// Compute the linear interpolation between colorA and colorB, using t in [0, 1]
	// and store the result in dest:
	void lerpColor(const Vec3* colorA, const Vec3* colorB, float t, Vec3* dest) {
		dest->x = colorA->x + t * (colorB->x - colorA->x);
		dest->y = colorA->y + t * (colorB->y - colorA->y);
		dest->z = colorA->z + t * (colorB->z - colorA->z);
	}

	void getColor(int x, int y, int z, Vec3* dest) {
		float normalized = (get(x, y, z) - minValue) / (maxValue - minValue);
		
		lerpColor(&minColor, &maxColor, normalized, dest);
	}

	void getPosition(int x, int y, int z, Vec3* dest, bool heightMap) {
		dest->x = (x - m_sizeX / 2) * m_step;
		dest->y = z * m_step;
		dest->z = (y - m_sizeY / 2) * m_step;

		if (heightMap) {
			// Compute a height for the point, depending on it's value:
			float height = map(get(x, y, z), minValue, maxValue, minHeight, maxHeight);

			dest->y += height;
		}
	}

	// Lineary map the value from [min, max] to [newMin, newMax]: 
	inline float map(float value, float min, float max, float newMin, float newMax) {
		return newMin + value * (newMax - newMin) / (max - min);
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
	void simulateTimestep(float timestep);
	void externalForcesCalculations(float timeElapsed) {};
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// Feel free to change the signature of these functions, add arguments, etc.
	void diffuseTemperatureImplicit(float timestep);
	void diffuseTemperatureExplicit2D(float timestep);
	void diffuseTemperatureExplicit3D(float timestep);

private:
	// Attributes
	Vec3  m_vfMovableObjectPos;
	Vec3  m_vfMovableObjectFinalPos;
	Vec3  m_vfRotate;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;

	Grid T;			// Current state of the temperature
	Grid nextT;		// Next state of the temperature (used when computing the timestep)
	
	float m_fAlpha;					// Thermal diffusivity
	float m_fGridScale;
	int m_iGridSizeX, m_iGridSizeY, m_iGridSizeZ;	// Dimensions of the grid

	bool m_bShowAllSlices;		// If we draw all slices in 3D mode
	int m_iSliceIndex;			// The slice we are currently drawing, if m_bShowAllSlices = false
	
	// Define the boundary temperatures:
	float m_fTopBoundaryTemperature;
	float m_fBottomBoundaryTemperature;
	float m_fLeftBoundaryTemperature;
	float m_fRightBoundaryTemperature;
	float m_fForwardBoundaryTemperature;
	float m_fBackwardBoundaryTemperature;


	void setupDemo1();
	void nextSlice();

	// Assuming that the given point is at the boundary, return it's temperature:
	float getBoundaryTemperature(int x, int y, int z);
};

#endif