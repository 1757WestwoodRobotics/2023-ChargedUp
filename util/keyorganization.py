class OptionalValueKeys:
    def __init__(self, baseKey: str) -> None:
        self.baseKey = baseKey
        self.valueKey = baseKey + "/value"
        self.validKey = baseKey + "/valid"
