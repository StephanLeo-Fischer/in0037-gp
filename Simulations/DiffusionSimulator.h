#ifndef DIFFUSIONSIMULATOR_h
#define DIFFUSIONSIMULATOR_h

#include "Simulator.h"
#include "vectorbase.h"

#define EXPLICIT_SOLVER 0
#define IMPLICIT_SOLVER 1

class Grid {
public:
	Grid() : m_rows(0), m_cols(0), m_size(0), m_data(NULL) {
		// Empty constructor. The object shouldn't be used in this case
	}

	Grid(int rows, int cols, float size) :
		m_rows(rows), m_cols(cols),
		m_size(size), m_data(new float[rows * cols]) {}

	void draw(DrawingUtilitiesClass* DUC) {
		Vec3 prevColor, currentColor;
		Vec3 prevPos, currentPos;

		DUC->beginLine();

		// Horizontal lines:
		for (int row = 0; row < m_rows; row++) {
			getPosition(row, 0, &prevPos);
			getColor(row, 0, &prevColor);

			for (int col = 1; col < m_cols; col++) {
				getPosition(row, col, &currentPos);
				getColor(row, col, &currentColor);

				DUC->drawLine(prevPos, prevColor, currentPos, currentColor);
				prevPos = currentPos;
				prevColor = currentColor;
			}
		}

		// Vertical lines:
		for (int col = 0; col < m_cols; col++) {
			getPosition(0, col, &prevPos);
			getColor(0, col, &prevColor);

			for (int row = 1; row < m_rows; row++) {
				getPosition(row, col, &currentPos);
				getColor(row, col, &currentColor);

				DUC->drawLine(prevPos, prevColor, currentPos, currentColor);
				prevPos = currentPos;
				prevColor = currentColor;
			}
		}
		DUC->endLine();
	}

	inline float getSize() const {
		return m_size;
	}

	inline int getRows() const {
		return m_rows;
	}

	inline int getCols() const {
		return m_cols;
	}

	void resize(int rows, int cols, float size) {
		if (m_rows * m_cols != rows * cols) {
			delete[] m_data;
			m_data = new float[rows * cols];
		}
		
		m_rows = rows;
		m_cols = cols;
		m_size = size;
	}

	void swapValues(Grid* other) {
		if (other->m_rows != m_rows || other->m_cols != m_cols)
			throw std::runtime_error("Cannot swap grid with different dimensions");

		float* tmp = other->m_data;
		other->m_data = this->m_data;
		this->m_data = tmp;
	}

	// Set all the values of the grid to a unique value:
	void set(float value) {
		for (int row = 0; row < m_rows; row++)
			for (int col = 0; col < m_cols; col++)
				m_data[positionToIndex(row, col)] = value;
	}

	void set(std::vector<Real> values) {
		if (values.size() != m_rows * m_cols)
			throw std::runtime_error("Cannot set this vector to this grid: the vector has the wrong size !");

		for (int i = 0; i < values.size(); i++)
			m_data[i] = values[i];
	}

	inline void set(int row, int col, float value) {
		if (row < 0 || row >= m_rows || col < 0 || col >= m_cols)
			throw std::out_of_range("Grid index is out of range");

		m_data[positionToIndex(row, col)] = value;
	}

	inline float get(int row, int col) const {
		return m_data[positionToIndex(row, col)];
	}

	// Map a position in the 2D grid, to a unique index in the array m_data:
	inline int positionToIndex(int row, int col) const {
		return m_cols * row + col;
	}

	// Map an index the array m_data to a unique position in the 2D grid:
	inline void indexToPosition(int index, int * row, int * col) {
		*row = index / m_cols;
		*col = index % m_cols;
	}

	boolean isBoundaryCell(int row, int col) {
		return row == 0 || row + 1 == m_rows || col == 0 || col + 1 == m_cols;
	}

	inline ~Grid() {
		delete[] m_data;
	}

private:
	float m_size;			// Size between the points of the grid

	int m_rows, m_cols;
	float* m_data;

	const float minValue = -100;
	const float maxValue = 1000;
	const Vec3 minColor = Vec3(1, 0, 0);	// The color of a point with the minimum value
	const Vec3 maxColor = Vec3(1, 1, 1);	// The color of a point with the maximum value
	const float minHeight = 0;				// The height of a point with the minimum value
	const float maxHeight = 10;				// The height of a point with the maximum value

	// Compute the linear interpolation between colorA and colorB, using t in [0, 1]
	// and store the result in dest:
	void lerpColor(const Vec3* colorA, const Vec3* colorB, float t, Vec3* dest) {
		dest->x = colorA->x + t * (colorB->x - colorA->x);
		dest->y = colorA->y + t * (colorB->y - colorA->y);
		dest->z = colorA->z + t * (colorB->z - colorA->z);
	}

	void getColor(int row, int col, Vec3* dest) {
		float normalized = (get(row, col) - minValue) / (maxValue - minValue);
		
		lerpColor(&minColor, &maxColor, normalized, dest);
	}

	void getPosition(int row, int col, Vec3* dest) {
		float height = map(get(row, col), minValue, maxValue, minHeight, maxHeight);
		
		dest->x = (row - m_rows / 2) * m_size;
		dest->y = height;
		dest->z = (col - m_cols / 2) * m_size;
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
	// Specific Functions
	void drawObjects();

	// Feel free to change the signature of these functions, add arguments, etc.
	void diffuseTemperatureExplicit(float timestep);
	void diffuseTemperatureImplicit(float timestep);

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
	int m_iGridRows, m_iGridCols;
	float m_fBoundaryTemperature;

	void setupDemo1();
};

#endif