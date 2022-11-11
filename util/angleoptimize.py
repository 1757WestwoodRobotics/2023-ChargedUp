from math import tau, floor
from wpimath.geometry import Rotation2d


def optimizeAngle(currentAngle: Rotation2d, targetAngle: Rotation2d) -> Rotation2d:
    currentAngle = currentAngle.radians()

    closestFullRotation = (
        floor(abs(currentAngle / tau)) * (-1 if currentAngle < 0 else 1) * tau
    )

    currentOptimalAngle = targetAngle.radians() + closestFullRotation - currentAngle

    potentialNewAngles = [
        currentOptimalAngle,
        currentOptimalAngle - tau,
        currentOptimalAngle + tau,
    ]  # closest other options

    deltaAngle = tau  # max possible error, a full rotation!
    for potentialAngle in potentialNewAngles:
        if abs(deltaAngle) > abs(potentialAngle):
            deltaAngle = potentialAngle

    return Rotation2d(deltaAngle + currentAngle)
