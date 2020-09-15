/*
 * VerticalPlotterKinematics.h
 *
 *  Created on: 01 Sept 2020
 *      Author: Stefan
 */

#ifndef SRC_MOVEMENT_KINEMATICS_VERTICALPLOTTERKINEMATICS_H_
#define SRC_MOVEMENT_KINEMATICS_VERTICALPLOTTERKINEMATICS_H_

#include "Kinematics.h"

class VerticalPlotterKinematics : public Kinematics
{
public:
	VerticalPlotterKinematics() noexcept;

	// Overridden base class functions. See Kinematics.h for descriptions.
	const char *GetName(bool forStatusReport) const noexcept override;
	bool Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply, bool& error) THROWS(GCodeException) override;
	bool CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const noexcept override;
	void MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const noexcept override;
	void GetAssumedInitialPosition(size_t numAxes, float positions[]) const noexcept override;
	size_t NumHomingButtons(size_t numVisibleAxes) const noexcept override { return 0; }
	const char* HomingButtonNames() const noexcept override { return "ABT"; }
	HomingMode GetHomingMode() const noexcept override { return HomingMode::homeIndividualMotors; }
	AxesBitmap AxesAssumedHomed(AxesBitmap g92Axes) const noexcept override;
	AxesBitmap MustBeHomedAxes(AxesBitmap axesMoving, bool disallowMovesBeforeHoming) const noexcept override;
	AxesBitmap GetHomingFileName(AxesBitmap toBeHomed, AxesBitmap alreadyHomed, size_t numVisibleAxes, const StringRef& filename) const noexcept override;
	bool QueryTerminateHomingMove(size_t axis) const noexcept override;
	void OnHomingSwitchTriggered(size_t axis, bool highEnd, const float stepsPerMm[], DDA& dda) const noexcept override;
	void LimitSpeedAndAcceleration(DDA& dda, const float *normalisedDirectionVector, size_t numVisibleAxes, bool continuousRotationShortcut) const noexcept override;
	AxesBitmap GetLinearAxes() const noexcept override;

protected:
	DECLARE_OBJECT_MODEL
	OBJECT_MODEL_ARRAY(anchorA)
	OBJECT_MODEL_ARRAY(anchorB)


private:
	static constexpr float DefaultSegmentsPerSecond = 100.0;
	static constexpr float DefaultMinSegmentSize = 0.2;

	// Basic facts about movement system
	static constexpr size_t VERTICALPLOTTER_AXES = 2;
	static constexpr size_t A_AXIS = 0;
	static constexpr size_t B_AXIS = 1;

	void Init() noexcept;
	void Recalc() noexcept;
	float LineLengthSquared(const float machinePos[3], const float anchor[2]) const noexcept;		// Calculate the square of the line length from a spool from a Cartesian coordinate
	void InverseTransform(float La, float Lb, float machinePos[3]) const noexcept;

	float anchorA[2], anchorB[2];				// XY coordinates of the anchors

	// Derived parameters
	float Xa, Xb, Ya, Yb;
	float Xa2, Xb2, Ya2, Yb2;

	bool calibrationMode;
};

#endif /* SRC_MOVEMENT_KINEMATICS_POLARKINEMATICS_H_ */
