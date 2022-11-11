from ctre import ErrorCode


def ctreCheckError(name: str, errorCode: ErrorCode) -> bool:
    if (errorCode is not None) and (errorCode != ErrorCode.OK):
        print(f"ERROR: {name}: {errorCode}")
        return False
    return True
