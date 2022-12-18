from ctre import ErrorCode
from wpilib import DataLogManager


def ctreCheckError(name: str, errorCode: ErrorCode) -> bool:
    if (errorCode is not None) and (errorCode != ErrorCode.OK):
        DataLogManager.log(f"ERROR: {name}: {errorCode}")
        return False
    return True
