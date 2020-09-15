/*
 * VerticalPlotterKinematics.cpp
 *
 *  Created on: 01 Sept 2020
 *      Author: Stefan
 */

#include "VerticalPlotterKinematics.h"

#include "RepRap.h"
#include "Platform.h"
#include "Storage/MassStorage.h"
#include "GCodes/GCodeBuffer/GCodeBuffer.h"
#include "Movement/DDA.h"

constexpr float DefaultAnchorA[2] = {-1000.0,  2000.0};
constexpr float DefaultAnchorB[2] = { 3000.0,  2000.0};
constexpr float DefaultCalibrationMode = false;

#if SUPPORT_OBJECT_MODEL

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(VerticalPlotterKinematics, __VA_ARGS__)

constexpr ObjectModelArrayDescriptor VerticalPlotterKinematics::anchorAArrayDescriptor =
{
	nullptr,					// no lock needed
	[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return 2; },
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue { return ExpressionValue(((const VerticalPlotterKinematics *)self)->anchorA[context.GetLastIndex()], 2); }
};

constexpr ObjectModelArrayDescriptor VerticalPlotterKinematics::anchorBArrayDescriptor =
{
	nullptr,					// no lock needed
	[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return 2; },
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue { return ExpressionValue(((const VerticalPlotterKinematics *)self)->anchorB[context.GetLastIndex()], 2); }
};

constexpr ObjectModelTableEntry VerticalPlotterKinematics::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. kinematics members
		{ "anchorA",	OBJECT_MODEL_FUNC_NOSELF(&anchorAArrayDescriptor), 	ObjectModelEntryFlags::none },
		{ "anchorB",	OBJECT_MODEL_FUNC_NOSELF(&anchorBArrayDescriptor), 	ObjectModelEntryFlags::none },
		{ "calibrationMode",	OBJECT_MODEL_FUNC(self->calibrationMode, 1), ObjectModelEntryFlags::none },
		{ "name",	OBJECT_MODEL_FUNC(self->GetName(true)), 	ObjectModelEntryFlags::none },
};

constexpr uint8_t VerticalPlotterKinematics::objectModelTableDescriptor[] = { 1, 4 };

DEFINE_GET_OBJECT_MODEL_TABLE(VerticalPlotterKinematics)

#endif

// Constructor
VerticalPlotterKinematics::VerticalPlotterKinematics() noexcept
	: Kinematics(KinematicsType::verticalplotter, DefaultSegmentsPerSecond, DefaultMinSegmentSize, true)

{
	Init();
}

void VerticalPlotterKinematics::Init() noexcept
{
	ARRAY_INIT(anchorA, DefaultAnchorA);
	ARRAY_INIT(anchorB, DefaultAnchorB);
	calibrationMode = DefaultCalibrationMode;

	Recalc();
}

// Recalculate the derived parameters
void VerticalPlotterKinematics::Recalc() noexcept
{
	Xa	= anchorA[0];
	Xb	= anchorB[0];
	Ya	= anchorA[1];
	Yb	= anchorB[1];
	Xa2 = fsquare(Xa);
	Xb2 = fsquare(Xb);
	Ya2 = fsquare(Ya);
	Yb2 = fsquare(Yb);
}

// Return the name of the current kinematics.
// If 'forStatusReport' is true then the string must be the one for that kinematics expected by DuetWebControl and PanelDue.
// Otherwise it should be in a format suitable for printing.
// For any new kinematics, the same string can be returned regardless of the parameter.
const char *VerticalPlotterKinematics::GetName(bool forStatusReport) const noexcept
{
	return "VerticalPlotter";
}

// Set or report the parameters from a M665, M666 or M669 command
// If 'mCode' is an M-code used to set parameters for the current kinematics (which should only ever be 665, 666, 667 or 669)
// then search for parameters used to configure the current kinematics. If any are found, perform appropriate actions,
// and return true if the changes affect the geometry.
// If errors were discovered while processing parameters, put an appropriate error message in 'reply' and set 'error' to true.
// If no relevant parameters are found, print the existing ones to 'reply' and return false.
// If 'mCode' does not apply to this kinematics, call the base class version of this function, which will print a suitable error message.
bool VerticalPlotterKinematics::Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply, bool& error) THROWS(GCodeException)
{
	if (mCode == 669)
	{
		bool seen = false;
		bool seenNonGeometry = false;
		gb.TryGetFValue('S', segmentsPerSecond, seenNonGeometry);
		gb.TryGetFValue('T', minSegmentLength, seenNonGeometry);

		if (gb.Seen('C'))
		{
			float calibrationParameter = 0;
			gb.TryGetFValue('C', calibrationParameter, seen);
			if (calibrationParameter == 0)
			{
				calibrationMode = false;
			}
			else if (calibrationParameter == 1)
			{
				calibrationMode = true;
			}
			else
			{
				reply.printf("C parameter invalid");
				error = true;
				return true;
			}
		}

		if (gb.TryGetFloatArray('A', 3, anchorA, reply, seen))
		{
			if (calibrationMode == false)
			{
				error = true;
				return true;
			}
		}
		if (gb.TryGetFloatArray('B', 3, anchorB, reply, seen))
		{
			if (calibrationMode == false)
			{
				error = true;
				return true;
			}
		}

		if (seen || seenNonGeometry)
		{
			Recalc();
		}
		else if (!gb.Seen('K'))
		{
			reply.printf("Kinematics is VerticalPlotter with AB anchor coordinates (%.2f,%.2f) (%.2f,%.2f),"
										"segments/sec %d, min. segment length %.2f",
										(double)anchorA[X_AXIS], (double)anchorA[Y_AXIS],
										(double)anchorB[X_AXIS], (double)anchorB[Y_AXIS],
										(int)segmentsPerSecond, (double)minSegmentLength);
		}
		return seen;
	}
	else
	{
		return Kinematics::Configure(mCode, gb, reply, error);
	}
}

// Calculate the square of the line length from an achnor from a Cartesian coordinate
inline float VerticalPlotterKinematics::LineLengthSquared(const float machinePos[3], const float anchor[2]) const noexcept
{
	return fsquare(anchor[Y_AXIS] - machinePos[Y_AXIS]) + fsquare(anchor[X_AXIS] - machinePos[X_AXIS]);
}

// Convert Cartesian coordinates to motor positions measured in steps from reference position
// 'machinePos' is a set of axis and extruder positions to convert
// 'stepsPerMm' is as configured in M92. On a Scara or polar machine this would actually be steps per degree.
// 'numAxes' is the number of machine axes to convert, which will always be at least 3
// 'motorPos' is the output vector of motor positions
// Return true if successful, false if we were unable to convert
bool VerticalPlotterKinematics::CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const noexcept
{
	const float aSquared = LineLengthSquared(machinePos, anchorA);
	const float bSquared = LineLengthSquared(machinePos, anchorB);

	if (aSquared > 0.0 && bSquared > 0.0)
	{
		motorPos[A_AXIS] = lrintf(sqrtf(aSquared) * stepsPerMm[A_AXIS]);
		motorPos[B_AXIS] = lrintf(sqrtf(bSquared) * stepsPerMm[B_AXIS]);
	}
	else {
		return false;
	}

	// Transform remaining axes linearly
	for (size_t axis = Z_AXIS; axis < numVisibleAxes; ++axis)
	{
		motorPos[axis] = lrintf(machinePos[axis] * stepsPerMm[axis]);
	}

	return true;
}

// Convert motor positions (measured in steps from reference position) to Cartesian coordinates
// 'motorPos' is the input vector of motor positions
// 'stepsPerMm' is as configured in M92. On a Scara or polar machine this would actually be steps per degree.
// 'numDrives' is the number of machine drives to convert, which will always be at least 3
// 'machinePos' is the output set of converted axis and extruder positions
void VerticalPlotterKinematics::MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const noexcept
{
	InverseTransform(motorPos[A_AXIS]/stepsPerMm[A_AXIS], motorPos[B_AXIS]/stepsPerMm[B_AXIS], machinePos);
}

// Calculate the Cartesian coordinates from the motor coordinates
void VerticalPlotterKinematics::InverseTransform(float La, float Lb, float machinePos[3]) const noexcept
{
	float La2 = fsquare(La);
	float Lb2 = fsquare(Lb);

	float Q = (La2 - Lb2 - Xa2 - Ya2 + Xb2 + Yb2) / (2*Xb - 2*Xa);
	float Q2 = fsquare(Q);
	float M = (Yb + Ya)/(Xb-Xa);

	float a = 2;
	float b = 2*Xa*M - 2*Q*M - 2*Ya;
	float b2 = fsquare(b);
	float c = Q2 - La2 - 2*Xa*Q + Xa2 + Ya2;

	float Ys1 = (-b + sqrtf(b2 - 4*a*c)) / (2*a);
	float Ys2 = (-b + sqrtf(b2 - 4*a*c)) / (2*a);

	float Ys = 0;

	if (Ys1 < Ys2) {
		Ys = Ys1;
	}
	else {
		Ys = Ys2;
	}

	float Xs = Q - M*Ys;

	machinePos[0] = Xs;
	machinePos[1] = Ys;
	machinePos[2] = 0;

	debugPrintf("Motor %.2f,%.2f to Cartesian %.2f,%.2f,%.2f\n", (double)La, (double)Lb, (double)machinePos[0], (double)machinePos[1], (double)machinePos[2]);

}

// Return the initial Cartesian coordinates we assume after switching to this kinematics
void VerticalPlotterKinematics::GetAssumedInitialPosition(size_t numAxes, float positions[]) const noexcept
{
	for (size_t i = 0; i < numAxes; ++i)
	{
		positions[i] = 0.0;
	}
}

// Return the axes that we can assume are homed after executing a G92 command to set the specified axis coordinates
AxesBitmap VerticalPlotterKinematics::AxesAssumedHomed(AxesBitmap g92Axes) const noexcept
{
	// If both X and Y have been specified then we know the position, otherwise we don't
	if ((g92Axes & XyAxes) != XyAxes)
	{
		g92Axes &= ~XyAxes;
	}
	return g92Axes;
}

// Return the set of axes that must be homed prior to regular movement of the specified axes
AxesBitmap VerticalPlotterKinematics::MustBeHomedAxes(AxesBitmap axesMoving, bool disallowMovesBeforeHoming) const noexcept
{
	if (axesMoving.Intersects(XyAxes))
	{
		axesMoving |= XyAxes;
	}
	return axesMoving;
}

// This function is called when a request is made to home the axes in 'toBeHomed' and the axes in 'alreadyHomed' have already been homed.
// If we can proceed with homing some axes, return the name of the homing file to be called. Optionally, update 'alreadyHomed' to indicate
// that some additional axes should be considered not homed.
// If we can't proceed because other axes need to be homed first, return nullptr and pass those axes back in 'mustBeHomedFirst'.
AxesBitmap VerticalPlotterKinematics::GetHomingFileName(AxesBitmap toBeHomed, AxesBitmap alreadyHomed, size_t numVisibleAxes, const StringRef& filename) const noexcept
{
	filename.copy("homeall.g");
	return AxesBitmap();
}

// This function is called from the step ISR when an endstop switch is triggered during homing.
// Return true if the entire homing move should be terminated, false if only the motor associated with the endstop switch should be stopped.
bool VerticalPlotterKinematics::QueryTerminateHomingMove(size_t axis) const noexcept
{
	return false;
}

// This function is called from the step ISR when an endstop switch is triggered during homing after stopping just one motor or all motors.
// Take the action needed to define the current position, normally by calling dda.SetDriveCoordinate() and return false.
void VerticalPlotterKinematics::OnHomingSwitchTriggered(size_t axis, bool highEnd, const float stepsPerMm[], DDA& dda) const noexcept
{
	// Trigger based homing is not supported
}

// Limit the speed and acceleration of a move to values that the mechanics can handle.
// The speeds in Cartesian space have already been limited.
void VerticalPlotterKinematics::LimitSpeedAndAcceleration(DDA& dda, const float *normalisedDirectionVector, size_t numVisibleAxes, bool continuousRotationShortcut) const noexcept
{
	// Limit the speed in the XY plane to the lower of the X and Y maximum speeds, and similarly for the acceleration
	const float xyFactor = sqrtf(fsquare(normalisedDirectionVector[X_AXIS]) + fsquare(normalisedDirectionVector[Y_AXIS]));
	if (xyFactor > 0.01)
	{
		const Platform& platform = reprap.GetPlatform();
		const float maxSpeed = min<float>(platform.MaxFeedrate(X_AXIS), platform.MaxFeedrate(Y_AXIS));
		const float maxAcceleration = min<float>(platform.Acceleration(X_AXIS), platform.Acceleration(Y_AXIS));
		dda.LimitSpeedAndAcceleration(maxSpeed/xyFactor, maxAcceleration/xyFactor);
	}
}

// Return a bitmap of axes that move linearly in response to the correct combination of linear motor movements.
// This is called to determine whether we can babystep the specified axis independently of regular motion.
AxesBitmap VerticalPlotterKinematics::GetLinearAxes() const noexcept
{
	return AxesBitmap();
}

// End
